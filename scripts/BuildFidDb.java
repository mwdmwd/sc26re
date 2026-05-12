/* Build a Ghidra Function ID database from an imported seed corpus.
 *
 * Expected project layout:
 *   /fid-programs/<compiler>/<library-family>/<version>/<variant>/*.elf
 *   /fid-archives/<compiler>/<library-family>/<version>/<variant>/*.a/*.obj
 */
//@category FunctionID
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.Set;
import java.util.TreeSet;
import java.util.regex.Pattern;
import java.util.regex.PatternSyntaxException;

import ghidra.app.script.GhidraScript;
import ghidra.feature.fid.db.FidDB;
import ghidra.feature.fid.db.FidFile;
import ghidra.feature.fid.db.FidFileManager;
import ghidra.feature.fid.db.FunctionRecord;
import ghidra.feature.fid.db.LibraryRecord;
import ghidra.feature.fid.service.FidPopulateResult;
import ghidra.feature.fid.service.FidPopulateResult.Disposition;
import ghidra.feature.fid.service.FidService;
import ghidra.feature.fid.service.Location;
import ghidra.framework.model.DomainObject;
import ghidra.framework.model.DomainFile;
import ghidra.framework.model.DomainFolder;
import ghidra.program.database.ProgramContentHandler;
import ghidra.program.model.lang.LanguageID;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.Program;
import ghidra.program.model.mem.MemoryAccessException;
import ghidra.util.exception.CancelledException;
import ghidra.util.exception.VersionException;
import ghidra.util.task.TaskMonitor;

public class BuildFidDb extends GhidraScript {
	private FidService service;
	private LanguageID languageID;
	private SymbolFilters ignoredSymbolFilters;
	private List<String> ignoredSymbols;
	private PrintWriter log;
	private boolean closeLog;

	@Override
	protected void run() throws Exception {
		String[] args = getScriptArgs();
		if (args.length < 3 || args.length > 5) {
			throw new IllegalArgumentException(
				"usage: BuildFidDb.java <out.fidb> <project-root-folder[,folder...]> " +
				"<language-id> [ignored-symbols-file|-] [log-file|-]");
		}

		File fidFile = new File(args[0]).getCanonicalFile();
		String rootPath = args[1];
		languageID = new LanguageID(args[2]);
		ignoredSymbolFilters = (args.length >= 4) ? readIgnoredSymbolFilters(args[3]) : null;

		if (args.length >= 5 && !args[4].equals("-")) {
			File logFile = new File(args[4]);
			File logParent = logFile.getParentFile();
			if (logParent != null) {
				logParent.mkdirs();
			}
			log = new PrintWriter(new FileWriter(logFile));
			closeLog = true;
		}
		else {
			log = new PrintWriter(System.out, true);
			closeLog = false;
		}

		try {
			if (fidFile.exists() && !fidFile.delete()) {
				throw new IllegalStateException("failed to replace " + fidFile);
			}
			File parent = fidFile.getParentFile();
			if (parent != null) {
				parent.mkdirs();
			}

			FidFileManager manager = FidFileManager.getInstance();
			manager.createNewFidDatabase(fidFile);
			manager.addUserFidFile(fidFile);

			FidFile target = findUserFidFile(manager, fidFile);
			if (target == null) {
				throw new IllegalStateException("Ghidra did not register " + fidFile);
			}

			service = new FidService();
			try (FidDB db = target.getFidDB(true)) {
				ArrayList<DomainFolder> roots = new ArrayList<>();
				for (String oneRootPath : rootPath.split(",")) {
					oneRootPath = oneRootPath.trim();
					if (oneRootPath.isEmpty()) {
						continue;
					}
					DomainFolder root = getFolder(getProjectRootFolder(), oneRootPath);
					if (root == null) {
						throw new IllegalArgumentException("missing project folder " + oneRootPath);
					}
					roots.add(root);
				}

				ignoredSymbols = expandIgnoredSymbols(roots);
				if (ignoredSymbols == null) {
					log.println("ignored symbols: none");
				}
				else {
					log.println("ignored symbols: " + ignoredSymbols.size());
				}

				for (DomainFolder root : roots) {
					populateAll(db, root);
				}
				db.saveDatabase("Saving FIDB", monitor);
			}
		}
		finally {
			if (log != null) {
				log.flush();
				if (closeLog) {
					log.close();
				}
			}
		}
	}

	private FidFile findUserFidFile(FidFileManager manager, File fidFile) throws Exception {
		for (FidFile f : manager.getUserAddedFiles()) {
			if (new File(f.getPath()).getCanonicalFile().equals(fidFile)) {
				return f;
			}
		}
		return null;
	}

	private SymbolFilters readIgnoredSymbolFilters(String path) throws Exception {
		if (path.equals("-")) {
			return null;
		}

		SymbolFilters filters = new SymbolFilters();
		try (BufferedReader reader = new BufferedReader(new FileReader(path))) {
			String line;
			while ((line = reader.readLine()) != null) {
				line = line.trim();
				if (line.isEmpty() || line.startsWith("#")) {
					continue;
				}
				filters.add(line);
			}
		}
		return filters;
	}

	private DomainFolder getFolder(DomainFolder folder, String path) {
		for (String part : path.split("[/\\\\]")) {
			if (part.isEmpty()) {
				continue;
			}
			folder = folder.getFolder(part);
			if (folder == null) {
				return null;
			}
		}
		return folder;
	}

	private void populateAll(FidDB db, DomainFolder root) throws CancelledException, IOException, MemoryAccessException, VersionException {
		for (DomainFolder compiler : root.getFolders()) {
			for (DomainFolder family : compiler.getFolders()) {
				for (DomainFolder version : family.getFolders()) {
					for (DomainFolder variant : version.getFolders()) {
						monitor.checkCancelled();
						populateOne(db, compiler, family, version, variant);
					}
				}
			}
		}
	}

	private void populateOne(FidDB db, DomainFolder compiler, DomainFolder family, DomainFolder version, DomainFolder variant) throws CancelledException, IOException, MemoryAccessException, VersionException {
		ArrayList<DomainFile> programs = new ArrayList<>();
		findPrograms(programs, variant);
		if (programs.isEmpty()) {
			log.println("skip empty " + variant.getPathname());
			return;
		}

		String libraryName = family.getName();
		String libraryVersion = version.getName();
		String libraryVariant = compiler.getName() + ":" + variant.getName();
		log.println(libraryName + ":" + libraryVersion + ":" + libraryVariant);

		FidPopulateResult result = service.createNewLibraryFromPrograms(db, libraryName,
			libraryVersion, libraryVariant, programs, null, languageID, null, ignoredSymbols,
			TaskMonitor.DUMMY);
		report(result);
		markIgnoredFunctionsAutoFail(db, result);
	}

	private void findPrograms(ArrayList<DomainFile> programs, DomainFolder folder)
			throws CancelledException {
		for (DomainFile file : folder.getFiles()) {
			monitor.checkCancelled();
			if (file.getContentType().equals(ProgramContentHandler.PROGRAM_CONTENT_TYPE)) {
				programs.add(file);
			}
		}
		for (DomainFolder child : folder.getFolders()) {
			findPrograms(programs, child);
		}
	}

	private List<String> expandIgnoredSymbols(List<DomainFolder> roots)
			throws CancelledException, IOException, VersionException {
		if (ignoredSymbolFilters == null || ignoredSymbolFilters.isEmpty()) {
			return null;
		}

		Set<String> symbols = new HashSet<>(ignoredSymbolFilters.exactSymbols);
		if (ignoredSymbolFilters.hasPatterns()) {
			ArrayList<DomainFile> programs = new ArrayList<>();
			for (DomainFolder root : roots) {
				findPrograms(programs, root);
			}
			for (DomainFile programFile : programs) {
				collectIgnoredSymbols(symbols, programFile);
			}
		}

		if (symbols.isEmpty()) {
			return null;
		}
		return new LinkedList<>(new TreeSet<>(symbols));
	}

	private void collectIgnoredSymbols(Set<String> symbols, DomainFile programFile)
			throws CancelledException, IOException, VersionException {
		DomainObject object = programFile.getDomainObject(this, false, false, monitor);
		try {
			if (!(object instanceof Program)) {
				return;
			}
			Program program = (Program) object;
			for (Function function : program.getFunctionManager().getFunctions(true)) {
				monitor.checkCancelled();
				String name = function.getName();
				if (ignoredSymbolFilters.matches(name)) {
					symbols.add(name);
				}
			}
		}
		finally {
			if (object != null) {
				object.release(this);
			}
		}
	}

	private void markIgnoredFunctionsAutoFail(FidDB db, FidPopulateResult result)
			throws IOException {
		if (result == null || ignoredSymbols == null || ignoredSymbols.isEmpty()) {
			return;
		}

		int marked = 0;
		LibraryRecord library = result.getLibraryRecord();
		for (String symbol : ignoredSymbols) {
			List<FunctionRecord> records = db.findFunctionsByLibraryAndName(library, symbol);
			service.markRecordsAutoFail(records, true);
			marked += records.size();
		}
		log.println("  auto-fail records: " + marked);
	}

	private void report(FidPopulateResult result) {
		if (result == null) {
			log.println("  no result");
			return;
		}

		log.println("  attempted: " + result.getTotalAttempted());
		log.println("  added:     " + result.getTotalAdded());
		log.println("  excluded:  " + result.getTotalExcluded());
		for (Entry<Disposition, Integer> entry : result.getFailures().entrySet()) {
			if (entry.getKey() != Disposition.INCLUDED) {
				log.println("  " + entry.getKey() + ": " + entry.getValue());
			}
		}

		TreeSet<String> unresolved = new TreeSet<>();
		for (Location location : result.getUnresolvedSymbols()) {
			unresolved.add(location.getFunctionName());
		}
		if (!unresolved.isEmpty()) {
			log.println("  unresolved symbols:");
			for (String symbol : unresolved) {
				log.println("    " + symbol);
			}
		}
	}

	private static class SymbolFilters {
		private Set<String> exactSymbols = new HashSet<>();
		private List<Pattern> patterns = new LinkedList<>();

		private void add(String line) {
			if (line.startsWith("/") && line.endsWith("/") && line.length() > 1) {
				addRegex(line.substring(1, line.length() - 1));
			}
			else if (line.indexOf('*') >= 0 || line.indexOf('?') >= 0) {
				addRegex(globToRegex(line));
			}
			else {
				exactSymbols.add(line);
			}
		}

		private void addRegex(String regex) {
			try {
				patterns.add(Pattern.compile(regex));
			}
			catch (PatternSyntaxException e) {
				throw new IllegalArgumentException("bad symbol regex: " + regex, e);
			}
		}

		private boolean isEmpty() {
			return exactSymbols.isEmpty() && patterns.isEmpty();
		}

		private boolean hasPatterns() {
			return !patterns.isEmpty();
		}

		private boolean matches(String name) {
			if (exactSymbols.contains(name)) {
				return true;
			}
			for (Pattern pattern : patterns) {
				if (pattern.matcher(name).matches()) {
					return true;
				}
			}
			return false;
		}

		private static String globToRegex(String glob) {
			StringBuilder regex = new StringBuilder("^");
			for (int i = 0; i < glob.length(); i++) {
				char ch = glob.charAt(i);
				switch (ch) {
					case '*':
						regex.append(".*");
						break;
					case '?':
						regex.append('.');
						break;
					case '.':
					case '\\':
					case '+':
					case '(':
					case ')':
					case '^':
					case '$':
					case '|':
					case '{':
					case '}':
					case '[':
					case ']':
						regex.append('\\').append(ch);
						break;
					default:
						regex.append(ch);
						break;
				}
			}
			regex.append('$');
			return regex.toString();
		}
	}
}
