# SPDX-License-Identifier: AGPL-3.0-or-later

cmake_minimum_required(VERSION 3.20)

# Zephyr 4.4 is supported by SDK 1.0.x. ZEPHYR_SDK_VERSION pins the fallback download,
# not the minimum version of an SDK that is already installed.
set(CMAKE_FIND_PACKAGE_SORT_ORDER NATURAL)
set(CMAKE_FIND_PACKAGE_SORT_DIRECTION DEC)

# Probe with an impossible version to enumerate packages without loading one.
# Use the same search locations as Zephyr, including CMake's package registry
# and CMAKE_PREFIX_PATH. An explicit SDK_ROOT limits the search to that path.
if(SDK_ROOT)
  find_package(Zephyr-sdk 0.0.0 EXACT QUIET CONFIG
               PATHS "${SDK_ROOT}" NO_DEFAULT_PATH)
else()
  find_package(Zephyr-sdk 0.0.0 EXACT QUIET CONFIG PATHS
               /usr /usr/local /opt
               "$ENV{HOME}" "$ENV{HOME}/.local" "$ENV{HOME}/.local/opt"
               "$ENV{HOME}/bin")
endif()

set(selected_sdk "")
set(selected_version 0.0.0)
foreach(version config IN ZIP_LISTS
        Zephyr-sdk_CONSIDERED_VERSIONS Zephyr-sdk_CONSIDERED_CONFIGS)
  if(version VERSION_LESS 1.0.0 OR NOT version VERSION_LESS 1.1.0)
    continue()
  endif()

  get_filename_component(config_dir "${config}" DIRECTORY)
  get_filename_component(sdk_dir "${config_dir}/.." ABSOLUTE)
  if(NOT EXISTS "${sdk_dir}/sdk_version" OR
     NOT EXISTS "${sdk_dir}/cmake/zephyr/host-tools.cmake" OR
     NOT EXISTS "${sdk_dir}/cmake/zephyr/gnu/generic.cmake")
    continue()
  endif()
  execute_process(COMMAND test -x
                  "${sdk_dir}/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc"
                  RESULT_VARIABLE compiler_missing)
  if(NOT compiler_missing EQUAL 0)
    continue()
  endif()

  if(version VERSION_GREATER selected_version)
    set(selected_sdk "${sdk_dir}")
    set(selected_version "${version}")
  endif()
endforeach()

if(SDK_REQUIRED)
  if(NOT selected_sdk)
    message(FATAL_ERROR
      "No usable Zephyr SDK 1.0.x with a GNU ARM toolchain found at ${SDK_ROOT}. "
      "Install the SDK and its arm-zephyr-eabi toolchain, or unset "
      "ZEPHYR_SDK_DIR and ZEPHYR_SDK_INSTALL_DIR to use the local fallback.")
  endif()
  message(STATUS "Using Zephyr SDK ${selected_version}: ${selected_sdk}")
elseif(selected_sdk)
  # Unlike message(), this writes only the selected path to stdout for Make.
  execute_process(COMMAND "${CMAKE_COMMAND}" -E echo "${selected_sdk}")
endif()
