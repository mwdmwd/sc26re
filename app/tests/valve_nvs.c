/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "imu_bias.h"
#include "valve_nvs.h"

#define ATE_SIZE 8U
#define CLOSE_OFFSET (VALVE_NVS_SECTOR_SIZE - ATE_SIZE)
#define FIRST_ATE_OFFSET (VALVE_NVS_SECTOR_SIZE - (2U * ATE_SIZE))
#define SPECIAL_ID 0xffffU
#define STOCK_FLASH_SIZE (512U * 1024U)
#define STOCK_NVS_OFFSET 0x7d000U

struct memory_nvs
{
	uint8_t data[VALVE_NVS_SIZE];
	size_t reads;
	int fail_after;
};

struct fixture_builder
{
	struct memory_nvs *image;
	uint16_t sector;
	uint16_t data_offset;
	uint16_t ate_offset;
};

struct expected_setting
{
	const char *name;
	size_t len;
};

static const struct expected_setting calibration_settings[] = {
	{ "cal/trg_l", 7U },  { "cal/trg_r", 7U }, { "cal/joy_l", 18U },
	{ "cal/joy_r", 18U }, { "cal/prs_l", 8U }, { "cal/prs_r", 8U },
};

#define CHECK(condition) \
	do \
	{ \
		if(!(condition)) \
		{ \
			fprintf(stderr, "%s:%d: check failed: %s\n", __func__, __LINE__, #condition); \
			return false; \
		} \
	} while(0)

static int memory_read(const void *context, uint32_t offset, void *data, size_t len)
{
	struct memory_nvs *image = (struct memory_nvs *)context;

	if(image->fail_after == 0)
	{
		return -EIO;
	}
	if(image->fail_after > 0)
	{
		--image->fail_after;
	}
	if(offset > sizeof(image->data) || len > sizeof(image->data) - offset)
	{
		return -EIO;
	}
	memcpy(data, &image->data[offset], len);
	++image->reads;
	return 0;
}

static void image_reset(struct memory_nvs *image)
{
	memset(image->data, 0xff, sizeof(image->data));
	image->reads = 0U;
	image->fail_after = -1;
}

static void put_le16(uint8_t *data, uint16_t value)
{
	data[0] = (uint8_t)value;
	data[1] = (uint8_t)(value >> 8U);
}

static uint8_t fixture_crc8(const uint8_t *data)
{
	uint8_t crc = 0xffU;

	for(size_t byte = 0; byte < ATE_SIZE - 1U; ++byte)
	{
		crc ^= data[byte];
		for(unsigned int bit = 0; bit < 8U; ++bit)
		{
			crc = (crc & 0x80U) != 0U ? (uint8_t)((crc << 1U) ^ 0x07U) : (uint8_t)(crc << 1U);
		}
	}
	return crc;
}

static uint8_t *fixture_ate(struct memory_nvs *image, uint16_t sector, uint16_t ate_offset,
                            uint16_t id, uint16_t data_offset, uint16_t len, uint8_t part)
{
	uint8_t *ate = &image->data[(size_t)sector * VALVE_NVS_SECTOR_SIZE + ate_offset];

	put_le16(&ate[0], id);
	put_le16(&ate[2], data_offset);
	put_le16(&ate[4], len);
	ate[6] = part;
	ate[7] = fixture_crc8(ate);
	return ate;
}

static void fixture_open_sector(struct fixture_builder *builder, struct memory_nvs *image,
                                uint16_t sector)
{
	builder->image = image;
	builder->sector = sector;
	builder->data_offset = 0U;
	builder->ate_offset = FIRST_ATE_OFFSET;
	(void)fixture_ate(image, sector, builder->ate_offset, SPECIAL_ID, 0U, 0U, 0xffU);
	builder->ate_offset -= ATE_SIZE;
}

static uint16_t fixture_write(struct fixture_builder *builder, uint16_t id, const void *data,
                              uint16_t len)
{
	uint16_t ate_offset = builder->ate_offset;
	size_t base = (size_t)builder->sector * VALVE_NVS_SECTOR_SIZE;

	if(len != 0U)
	{
		memcpy(&builder->image->data[base + builder->data_offset], data, len);
	}
	(void)fixture_ate(builder->image, builder->sector, ate_offset, id, builder->data_offset, len,
	                  0xffU);
	builder->data_offset = (uint16_t)((builder->data_offset + len + 3U) & ~3U);
	builder->ate_offset -= ATE_SIZE;
	return ate_offset;
}

static void fixture_close_sector(struct fixture_builder *builder)
{
	(void)fixture_ate(builder->image, builder->sector, CLOSE_OFFSET, SPECIAL_ID,
	                  (uint16_t)(builder->ate_offset + ATE_SIZE), 0U, 0xffU);
}

static uint32_t fnv1a32(const struct memory_nvs *image)
{
	uint32_t hash = 2166136261U;

	for(size_t i = 0; i < sizeof(image->data); ++i)
	{
		hash = (hash ^ image->data[i]) * 16777619U;
	}
	return hash;
}

static bool test_empty_image(void)
{
	struct memory_nvs image;
	struct valve_nvs nvs;
	uint8_t value;
	uint32_t before;

	image_reset(&image);
	before = fnv1a32(&image);
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == 0);
	CHECK(valve_nvs_read(&nvs, 1U, &value, sizeof(value)) == -ENOENT);
	CHECK(fnv1a32(&image) == before);
	CHECK(image.reads > 0U);
	return true;
}

static bool test_latest_partial_and_delete(void)
{
	static const uint8_t old_value[] = { 1U, 2U, 3U };
	static const uint8_t new_value[] = { 4U, 5U, 6U, 7U, 8U, 9U };
	struct fixture_builder builder;
	struct memory_nvs image;
	struct valve_nvs nvs;
	uint8_t value[sizeof(new_value)] = { 0U };

	image_reset(&image);
	fixture_open_sector(&builder, &image, 0U);
	(void)fixture_write(&builder, 42U, old_value, sizeof(old_value));
	(void)fixture_write(&builder, 42U, new_value, sizeof(new_value));
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == 0);
	CHECK(valve_nvs_read(&nvs, 42U, value, 2U) == sizeof(new_value));
	CHECK(memcmp(value, new_value, 2U) == 0);
	memset(value, 0, sizeof(value));
	CHECK(valve_nvs_read(&nvs, 42U, value, sizeof(value)) == sizeof(new_value));
	CHECK(memcmp(value, new_value, sizeof(value)) == 0);
	CHECK(valve_nvs_read(&nvs, 42U, NULL, 0U) == sizeof(new_value));

	(void)fixture_write(&builder, 42U, NULL, 0U);
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == 0);
	CHECK(valve_nvs_read(&nvs, 42U, value, sizeof(value)) == -ENOENT);
	return true;
}

static bool test_rollover_and_wrap(void)
{
	static const uint8_t old_value[] = { 0x11U, 0x22U };
	static const uint8_t new_value[] = { 0x33U, 0x44U, 0x55U };
	struct fixture_builder old_sector;
	struct fixture_builder active_sector;
	struct memory_nvs image;
	struct valve_nvs nvs;
	uint8_t value[sizeof(new_value)];

	image_reset(&image);
	fixture_open_sector(&old_sector, &image, 0U);
	(void)fixture_write(&old_sector, 100U, old_value, sizeof(old_value));
	(void)fixture_write(&old_sector, 200U, old_value, sizeof(old_value));
	fixture_close_sector(&old_sector);
	fixture_open_sector(&active_sector, &image, 1U);
	(void)fixture_write(&active_sector, 100U, new_value, sizeof(new_value));
	(void)fixture_write(&active_sector, 200U, NULL, 0U);
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == 0);
	CHECK(valve_nvs_read(&nvs, 100U, value, sizeof(value)) == sizeof(new_value));
	CHECK(memcmp(value, new_value, sizeof(value)) == 0);
	CHECK(valve_nvs_read(&nvs, 200U, value, sizeof(value)) == -ENOENT);

	image_reset(&image);
	fixture_open_sector(&old_sector, &image, 2U);
	(void)fixture_write(&old_sector, 300U, old_value, sizeof(old_value));
	fixture_close_sector(&old_sector);
	fixture_open_sector(&active_sector, &image, 0U);
	(void)fixture_write(&active_sector, 301U, new_value, sizeof(new_value));
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == 0);
	CHECK(valve_nvs_read(&nvs, 300U, value, sizeof(value)) == sizeof(old_value));
	CHECK(memcmp(value, old_value, sizeof(old_value)) == 0);
	CHECK(valve_nvs_read(&nvs, 301U, value, sizeof(value)) == sizeof(new_value));
	CHECK(memcmp(value, new_value, sizeof(new_value)) == 0);
	return true;
}

static bool test_torn_entries_are_ignored_and_unsupported_format_fails(void)
{
	static const uint8_t old_value[] = { 0xa1U, 0xa2U };
	static const uint8_t new_value[] = { 0xb1U, 0xb2U };
	struct fixture_builder builder;
	struct memory_nvs image;
	struct valve_nvs nvs;
	uint16_t ate_offset;
	uint8_t value[sizeof(old_value)];
	uint8_t *ate;

	image_reset(&image);
	fixture_open_sector(&builder, &image, 0U);
	(void)fixture_write(&builder, 50U, old_value, sizeof(old_value));
	ate_offset = fixture_write(&builder, 50U, new_value, sizeof(new_value));
	image.data[ate_offset + 7U] ^= 1U;

	ate_offset = builder.ate_offset;
	(void)fixture_ate(&image, 0U, ate_offset, 52U, ate_offset - 4U, 8U, 0xffU);
	builder.ate_offset -= ATE_SIZE;

	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == 0);
	CHECK(valve_nvs_read(&nvs, 50U, value, sizeof(value)) == sizeof(old_value));
	CHECK(memcmp(value, old_value, sizeof(value)) == 0);
	CHECK(valve_nvs_read(&nvs, 52U, value, sizeof(value)) == -ENOENT);

	image_reset(&image);
	fixture_open_sector(&builder, &image, 0U);
	ate_offset = fixture_write(&builder, 51U, new_value, sizeof(new_value));
	ate = &image.data[ate_offset];
	ate[6] = 0U;
	ate[7] = fixture_crc8(ate);
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == -EIO);
	return true;
}

static bool test_bad_layouts_fail_closed(void)
{
	struct fixture_builder builder;
	struct memory_nvs image;
	struct valve_nvs nvs;
	static const uint8_t value[] = { 1U, 2U, 3U, 4U };

	image_reset(&image);
	for(uint16_t sector = 0U; sector < VALVE_NVS_SECTOR_COUNT; ++sector)
	{
		fixture_open_sector(&builder, &image, sector);
		fixture_close_sector(&builder);
	}
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == -EIO);

	image_reset(&image);
	image.data[CLOSE_OFFSET] = 0U;
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == -EIO);

	image_reset(&image);
	fixture_open_sector(&builder, &image, 0U);
	(void)fixture_write(&builder, 1U, value, sizeof(value));
	(void)fixture_write(&builder, 2U, value, sizeof(value));
	fixture_close_sector(&builder);
	(void)fixture_ate(&image, 0U, CLOSE_OFFSET, SPECIAL_ID, FIRST_ATE_OFFSET - ATE_SIZE, 0U, 0xffU);
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == -EIO);
	(void)fixture_ate(&image, 0U, CLOSE_OFFSET, SPECIAL_ID, 0U, 0U, 0xffU);
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == -EIO);

	image_reset(&image);
	fixture_open_sector(&builder, &image, 0U);
	(void)fixture_write(&builder, 1U, value, sizeof(value));
	(void)fixture_ate(&image, 0U, builder.ate_offset, 2U, 0U, 0U, 0xffU);
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == -EIO);

	image_reset(&image);
	fixture_open_sector(&builder, &image, 0U);
	image.data[VALVE_NVS_SECTOR_SIZE] = 0U;
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == -EIO);
	image.data[VALVE_NVS_SECTOR_SIZE] = 0xffU;
	image.data[2U * VALVE_NVS_SECTOR_SIZE] = 0U;
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == -EIO);

	image_reset(&image);
	fixture_open_sector(&builder, &image, 0U);
	image.data[0] = 0U;
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == 0);
	CHECK(valve_nvs_read(&nvs, 1U, NULL, 0U) == -ENOENT);

	image_reset(&image);
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == 0);
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data) - 1U) == -EINVAL);
	CHECK(valve_nvs_read(&nvs, 1U, NULL, 0U) == -EACCES);
	image.reads = 0U;

	image_reset(&image);
	image.fail_after = 0;
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == -EIO);
	image.fail_after = -1;
	image.reads = 0U;
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data) - 1U) == -EINVAL);
	CHECK(image.reads == 0U);

	image_reset(&image);
	fixture_open_sector(&builder, &image, 0U);
	(void)fixture_write(&builder, 1U, value, sizeof(value));
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == 0);
	image.fail_after = 0;
	CHECK(valve_nvs_read(&nvs, 1U, NULL, 0U) == -EIO);
	return true;
}

static bool verify_calibration_image(struct memory_nvs *image)
{
	struct valve_nvs nvs;

	CHECK(valve_nvs_open(&nvs, memory_read, image, sizeof(image->data)) == 0);
	for(size_t setting = 0;
	    setting < sizeof(calibration_settings) / sizeof(calibration_settings[0]); ++setting)
	{
		uint8_t value[32];

		CHECK(valve_nvs_read_setting(&nvs, calibration_settings[setting].name, value,
		                             sizeof(value)) == (int)calibration_settings[setting].len);
	}
	return true;
}

static bool test_calibration_fixture(void)
{
	struct fixture_builder builder;
	struct memory_nvs image;
	uint8_t value[32];

	image_reset(&image);
	fixture_open_sector(&builder, &image, 0U);
	for(size_t setting = 0;
	    setting < sizeof(calibration_settings) / sizeof(calibration_settings[0]); ++setting)
	{
		uint16_t id = (uint16_t)(0x8001U + setting);

		memset(value, (int)(setting + 1U), calibration_settings[setting].len);
		(void)fixture_write(&builder, id, calibration_settings[setting].name,
		                    (uint16_t)strlen(calibration_settings[setting].name));
		(void)fixture_write(&builder, (uint16_t)(id + 0x4000U), value,
		                    (uint16_t)calibration_settings[setting].len);
	}
	put_le16(value,
	         (uint16_t)(0x8000U + sizeof(calibration_settings) / sizeof(calibration_settings[0])));
	(void)fixture_write(&builder, 0x8000U, value, sizeof(uint16_t));
	CHECK(image.data[FIRST_ATE_OFFSET + 7U] == 0x5cU);
	return verify_calibration_image(&image);
}

static bool test_settings_name_metadata_and_gyro_bias(void)
{
	static const char gyro_bias_name[] = "cal/sensors/gyroscope/bias";
	static const uint8_t encoded_bias[3 * sizeof(float)] = {
		0xcdU, 0xccU, 0xccU, 0x3dU, 0xcdU, 0xccU, 0x4cU, 0xbeU, 0x00U, 0x00U, 0x00U, 0x3fU,
	};
	struct fixture_builder builder;
	struct memory_nvs image;
	struct valve_nvs nvs;
	uint8_t value[sizeof(encoded_bias)];
	uint8_t last_name_id[sizeof(uint16_t)];
	float valid_bias[3] = { 0.1f, -0.2f, 0.5f };
	float invalid_bias[3] = { 0.0f, 0.0f, 0.0f };

	image_reset(&image);
	fixture_open_sector(&builder, &image, 0U);
	(void)fixture_write(&builder, 0x8055U, gyro_bias_name, (uint16_t)strlen(gyro_bias_name));
	(void)fixture_write(&builder, 0xc055U, encoded_bias, sizeof(encoded_bias));
	put_le16(last_name_id, 0x8055U);
	(void)fixture_write(&builder, 0x8000U, last_name_id, sizeof(last_name_id));
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == 0);
	CHECK(valve_nvs_read_setting(&nvs, gyro_bias_name, value, sizeof(value)) ==
	      sizeof(encoded_bias));
	CHECK(memcmp(value, encoded_bias, sizeof(value)) == 0);

	/* A present but truncated OFW vector is reported at its actual size. */
	(void)fixture_write(&builder, 0xc055U, encoded_bias, sizeof(encoded_bias) - sizeof(float));
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == 0);
	CHECK(valve_nvs_read_setting(&nvs, gyro_bias_name, value, sizeof(value)) ==
	      sizeof(encoded_bias) - sizeof(float));

	CHECK(imu_gyro_bias_valid(valid_bias));
	invalid_bias[0] = NAN;
	CHECK(!imu_gyro_bias_valid(invalid_bias));
	invalid_bias[0] = INFINITY;
	CHECK(!imu_gyro_bias_valid(invalid_bias));
	invalid_bias[0] = IMU_GYRO_BIAS_MAX_RAD_S + 0.01f;
	CHECK(!imu_gyro_bias_valid(invalid_bias));
	return true;
}

static bool test_settings_name_boundaries_and_duplicates(void)
{
	static const char name[] = "cal/sensors/gyroscope/bias";
	static const uint8_t first_value[] = { 1U, 2U, 3U };
	static const uint8_t second_value[] = { 4U, 5U, 6U };
	struct fixture_builder builder;
	struct memory_nvs image;
	struct valve_nvs nvs;
	uint8_t metadata[sizeof(uint16_t)];
	uint8_t value[sizeof(first_value)];

	image_reset(&image);
	fixture_open_sector(&builder, &image, 0U);
	(void)fixture_write(&builder, 0x8001U, name, (uint16_t)strlen(name));
	(void)fixture_write(&builder, 0xc001U, first_value, sizeof(first_value));
	(void)fixture_write(&builder, 0x8002U, name, (uint16_t)strlen(name));
	(void)fixture_write(&builder, 0xc002U, second_value, sizeof(second_value));
	put_le16(metadata, 0x8002U);
	(void)fixture_write(&builder, 0x8000U, metadata, sizeof(metadata));
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == 0);
	CHECK(valve_nvs_read_setting(&nvs, name, value, sizeof(value)) == -EIO);

	image_reset(&image);
	fixture_open_sector(&builder, &image, 0U);
	(void)fixture_write(&builder, 0xbffeU, name, (uint16_t)strlen(name));
	(void)fixture_write(&builder, 0xfffeU, first_value, sizeof(first_value));
	put_le16(metadata, 0xbffeU);
	(void)fixture_write(&builder, 0x8000U, metadata, sizeof(metadata));
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == 0);
	CHECK(valve_nvs_read_setting(&nvs, name, value, sizeof(value)) == sizeof(first_value));
	CHECK(memcmp(value, first_value, sizeof(value)) == 0);

	put_le16(metadata, 0xbfffU);
	(void)fixture_write(&builder, 0x8000U, metadata, sizeof(metadata));
	CHECK(valve_nvs_open(&nvs, memory_read, &image, sizeof(image.data)) == 0);
	CHECK(valve_nvs_read_setting(&nvs, name, value, sizeof(value)) == -EIO);
	return true;
}

static bool verify_legacy_fixture_values(struct memory_nvs *image)
{
	struct valve_nvs nvs;
	uint8_t name_count[2];

	CHECK(valve_nvs_open(&nvs, memory_read, image, sizeof(image->data)) == 0);
	CHECK(valve_nvs_read(&nvs, 0x8000U, name_count, sizeof(name_count)) == sizeof(name_count));
	CHECK(name_count[0] == 0x06U && name_count[1] == 0x80U);
	CHECK(valve_nvs_read(&nvs, 0x1234U, NULL, 0U) == -ENOENT);

	for(size_t setting = 0;
	    setting < sizeof(calibration_settings) / sizeof(calibration_settings[0]); ++setting)
	{
		uint8_t expected = setting == 0U ? 0xa5U : (uint8_t)(setting + 1U);
		uint8_t value[32];

		CHECK(valve_nvs_read_setting(&nvs, calibration_settings[setting].name, value,
		                             sizeof(value)) == (int)calibration_settings[setting].len);
		for(size_t byte = 0; byte < calibration_settings[setting].len; ++byte)
		{
			CHECK(value[byte] == expected);
		}
	}
	return true;
}

static bool load_stock_image(const char *path, struct memory_nvs *image)
{
	FILE *file = fopen(path, "rb");
	long size;
	long offset;

	if(file == NULL)
	{
		perror(path);
		return false;
	}
	if(fseek(file, 0L, SEEK_END) != 0 || (size = ftell(file)) < 0L)
	{
		fclose(file);
		return false;
	}
	if(size == (long)sizeof(image->data))
	{
		offset = 0L;
	}
	else if(size == STOCK_FLASH_SIZE)
	{
		offset = STOCK_NVS_OFFSET;
	}
	else
	{
		fprintf(stderr, "%s: expected a 12 KiB NVS image or 512 KiB flash dump\n", path);
		fclose(file);
		return false;
	}
	if(fseek(file, offset, SEEK_SET) != 0 ||
	   fread(image->data, 1U, sizeof(image->data), file) != sizeof(image->data))
	{
		fclose(file);
		return false;
	}
	fclose(file);
	image->reads = 0U;
	image->fail_after = -1;
	return true;
}

int main(int argc, char **argv)
{
	static const struct
	{
		const char *name;
		bool (*run)(void);
	} tests[] = {
		{ "empty image", test_empty_image },
		{ "latest, partial read, and delete", test_latest_partial_and_delete },
		{ "sector rollover and wrap", test_rollover_and_wrap },
		{ "torn entries and unsupported format",
		  test_torn_entries_are_ignored_and_unsupported_format_fails },
		{ "bad layouts", test_bad_layouts_fail_closed },
		{ "calibration fixture", test_calibration_fixture },
		{ "settings metadata and gyro bias", test_settings_name_metadata_and_gyro_bias },
		{ "settings name boundaries and duplicates", test_settings_name_boundaries_and_duplicates },
	};

	for(size_t test = 0; test < sizeof(tests) / sizeof(tests[0]); ++test)
	{
		if(!tests[test].run())
		{
			return EXIT_FAILURE;
		}
		printf("PASS: %s\n", tests[test].name);
	}

	for(int arg = 1; arg < argc; ++arg)
	{
		struct memory_nvs image;
		bool legacy_fixture = false;

		if(strcmp(argv[arg], "--legacy-fixture") == 0)
		{
			legacy_fixture = true;
			++arg;
			if(arg == argc)
			{
				fprintf(stderr, "--legacy-fixture requires an image path\n");
				return EXIT_FAILURE;
			}
		}

		if(!load_stock_image(argv[arg], &image) ||
		   !verify_calibration_image(&image) ||
		   (legacy_fixture && !verify_legacy_fixture_values(&image)))
		{
			return EXIT_FAILURE;
		}
		printf("PASS: %sNVS compatibility: %s\n", legacy_fixture ? "legacy " : "stock ", argv[arg]);
	}

	return EXIT_SUCCESS;
}
