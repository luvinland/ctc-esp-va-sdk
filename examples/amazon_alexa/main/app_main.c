// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <esp_event_loop.h>
#include <esp_pm.h>
#include <nvs_flash.h>

#include <conn_mgr_prov.h>
#include <conn_mgr_prov_mode_ble.h>

#include <voice_assistant.h>
#include <alexa.h>
#include <alexa_local_config.h>

#include <va_mem_utils.h>
#include <scli.h>
#include <va_diag_cli.h>
#include <wifi_cli.h>
#include <media_hal.h>
#include <tone.h>
#include <avs_config.h>
#include <speech_recognizer.h>
#include "va_board.h"
#include "app_auth.h"
#include "app_wifi.h"

#ifdef CONFIG_ALEXA_ENABLE_EQUALIZER
#include "alexa_equalizer.h"
#endif

#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

#ifdef CONFIG_ALEXA_ENABLE_OTA
#include "app_ota.h"
#endif

#include "app_defs.h"

#if defined(BLYNK_APPS)
#include "blynk.h"
#endif

#if defined(FACTORY_RESET)
#include "va_nvs_utils.h"
#endif

#define SOFTAP_SSID_PREFIX  "ESP-Alexa-"

static const char *TAG = "[app_main]";

#if defined(CTC_REV01)
#define TRI_LED 14
#define RES_LED 13

static void ctc_led_init(void)
{
    gpio_pad_select_gpio(TRI_LED);
    gpio_set_direction(TRI_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(TRI_LED, 0);

    gpio_pad_select_gpio(RES_LED);
    gpio_set_direction(RES_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(RES_LED, 0);
}
#endif

#if defined(CTC_CS48L32)
#include "esp_spiffs.h"
#include "wmfwparse.h"
#include "driver/spi_master.h"

size_t dspBase = 0;
size_t pmBase = 0;
size_t zmBase = 0;
size_t xmBaseUnpacked = 0;
size_t xmBasePacked = 0;
size_t ymBaseUnpacked = 0;
size_t ymBasePacked = 0;

AlgorithmIDBlockFormat * algorithmIdBlocks = NULL;

esp_err_t ctc_spiffs_init(void)
{
	ESP_LOGI(TAG, "Initializing SPIFFS");

	esp_vfs_spiffs_conf_t conf = {
		.base_path = "/spiffs",
		.partition_label = NULL,
		.max_files = 5,
		.format_if_mount_failed = true
	};

	// Note: esp_vfs_spiffs_register is an all-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
		}
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(NULL, &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
	}

	return ret;
}

static void select_core(void)
{
	size_t dspBaseAddress = BASE_CORE1_DSP; 
	size_t dspBaseMemAddress = BASE_CORE1_MEM;
	size_t memBaseAddr;

	dspBase = dspBaseAddress;
	memBaseAddr = dspBaseMemAddress;
	pmBase = memBaseAddr + OFFSET_PM_START;
	zmBase = memBaseAddr + OFFSET_ZM_START;
	xmBaseUnpacked = memBaseAddr + OFFSET_XM_UNPACKED_START;
	xmBasePacked = memBaseAddr + OFFSET_XM_PACKED_START;
	ymBaseUnpacked = memBaseAddr + OFFSET_YM_UNPACKED_START;
	ymBasePacked = memBaseAddr + OFFSET_YM_PACKED_START;
}

#define GPIO_MOSI		25
#define GPIO_MISO		27
#define GPIO_SCLK		33
#define GPIO_CS			26
#define GPIO_CS_RESET	22
#define GPIO_AK_PDN		4

spi_device_handle_t g_spi = NULL;

static const uint32_t cs48l32_spi_padding = 0x0;

#if defined(CTC_CS48L32_TUNE_1ST)
#define CS48L32_REG_TYPE_CONFIG			0
#define CS48L32_REG_TYPE_DSP_PROGRAM	1
#define CS48L32_REG_TYPE_TUNE_1ST		2
#define CS48L32_REG_TYPE_DSP_START		3
#if defined(CTC_CS48L32_FLL_ASP1_BCLK)
#define CS48L32_REG_TYPE_FLL_CHANGE		4
#endif
#if defined(CTC_CS48L32_CHECK_REG)
#define CS48L32_REG_TYPE_CHECK_REG		5
#endif
#if defined(CTC_CS48L32_MUTE_CONTROL)
#define CS48L32_REG_TYPE_MUTE_CONTROL	6
#endif
#else
#define CS48L32_REG_TYPE_CONFIG			0
#define CS48L32_REG_TYPE_DSP_PROGRAM	1
#define CS48L32_REG_TYPE_DSP_START		2
#if defined(CTC_CS48L32_FLL_ASP1_BCLK)
#define CS48L32_REG_TYPE_FLL_CHANGE		3
#endif
#endif

#define CS48L32_CONFIG_REG	(146)
static const uint32_t cs48l32_config[CS48L32_CONFIG_REG][2] =
{
	{0x20,		0x5A000000},
	{0x1C00,	0x0000},
	{0x1C04,	0x88610004},
	{0x1C08,	0x10000},
	{0x1C0C,	0x21F05001},
	{0x1C00,	0x0006},
	{0x1C00,	0x0007},
	{0x1C00,	0x0005},
	{0x1404,	0x0444},
	{0x1400,	0x0042},
	{0x1424,	0x0012},
	{0x1420,	0x0012},
#if defined(EMO_ROBOT_16khz)
	{0xA800,	0x0800},
#else
	{0xA800,	0x1000},
#endif
	{0x2000,	0x0007},
	{0x2410,	0x00E7},
	{0x2418,	0x0223},	// Jace. MICBIAS1A
	{0x4008,	0x0000},
	{0x4020,	0x20020},
	{0x4060,	0x20020},
	{0x4024,	0x0000},
	{0x4044,	0x0000},
	{0x4000,	0x0003},
#if defined(CTC_CS48L32_TUNE_2ND_COWAY)
	{0x4028,	0x8000BC},	// IN1L_MUTE=0, IN1L_VOL=0dB, IN1L_PGA_VOL=30dB
	{0x4048,	0x8000BC},	// IN1L_MUTE=0, IN1L_VOL=0dB, IN1L_PGA_VOL=30dB
#else
#if defined(CTC_CS48L32_TUNE_1ST)
	{0x4028,	0x8000BC},	// IN1L_MUTE=0, IN1L_VOL=0dB, IN1L_PGA_VOL=30dB
	{0x4048,	0x8000BC},	// IN1L_MUTE=0, IN1L_VOL=0dB, IN1L_PGA_VOL=30dB
#else
	{0x4028,	0x8000A6},
	{0x4048,	0x8000A6},
#endif
#endif
	{0xC10,		0xE1000000},
	{0xC14,		0xE1000000},
	{0xC18,		0xE1000000},
	{0xC1C,		0xE1000000},
	{0xC20,		0xE1000000},
	{0xC24,		0xE1000000},
	{0xC28,		0xE1000000},
	{0xC2C,		0xE1000000},
#if defined(EMO_ROBOT_16khz)
	{0x6004,	0x0112},
#else
	{0x6004,	0x0221},
#endif
	{0x6008,	0x20200200},
	{0x6040,	0x0010},
	{0x6030,	0x0010},
	{0x6000,	0x30003},
#if defined(EMO_ROBOT_16khz)
	{0x6084,	0x0112},
#else
	{0x6084,	0x0221},
#endif
	{0x6088,	0x20200233},
	{0x60C0,	0x0010},
	{0x60B0,	0x0010},
	{0x6080,	0x0003},
	{0x608C,	0x0000},
	{0xA400,	0x1000},
	{0xA404,	0x0C03},
	{0x8B80,	0x800020},
	{0x8B84,	0x800021},
#if defined(CTC_CS48L32_TUNE_1ST)
	{0x8300,	0x6200B8},
	{0x8310,	0x6200B8},
#else
	{0x8300,	0x6E80B8},
	{0x8310,	0x6E80B8},
#endif
#if defined(EMO_ROBOT_16khz)
	{0x89C0,	0x0000},
	{0x89D0,	0x0000},
	{0x9000,	0x8000B8},
	{0x9020,	0x8000B8},
#else
	{0x89C0,	0x00B8},
	{0x89D0,	0x00B8},
	{0x9000,	0x80009C},
	{0x9020,	0x80009D},
#endif
	{0x9040,	0x800010},
	{0x9050,	0x800011},
#if defined(EMO_ROBOT_16khz)
	{0x89A0,	0x0000},
	{0x89B0,	0x0000},
	{0x8200,	0x800102},
	{0x8210,	0x800102},
#else
	{0x89A0,	0x0102},
	{0x89B0,	0x0103},
	{0x8200,	0x80009A},
	{0x8210,	0x80009A},	// Jace. 200110. Cirrus's DSP output pin 4 have delay result.
#endif
	{0xA808,	0x0001},
	{0x1700C,	0x0003},
	{0x17010,	0x0003},
	{0x17014,	0x0003},
	{0x17018,	0x0003},
	{0x1701C,	0x0003},
	{0x17020,	0x0003},
	{0x17024,	0x0003},
	{0x17028,	0x0003},
	{0x1702C,	0x0003},
	{0x17030,	0x0003},
	{0x17034,	0x0003},
	{0x17038,	0x0003},
	{0x1703C,	0x0003},
	{0x17040,	0x0003},
	{0x17044,	0x0003},
	{0x17048,	0x0003},
	{0x1704C,	0x0003},
	{0x17050,	0x0003},
	{0x17054,	0x0003},
	{0x17058,	0x0003},
	{0x1705C,	0x0003},
	{0x17060,	0x0003},
	{0x17064,	0x0003},
	{0x17068,	0x0003},
	{0x1706C,	0x0003},
	{0x17070,	0x0003},
	{0x17074,	0x0003},
	{0x17078,	0x0003},
	{0x1707C,	0x0003},
	{0x17080,	0x0003},
	{0x17084,	0x0003},
	{0x17088,	0x0003},
	{0x1708C,	0x0003},
	{0x17090,	0x0003},
	{0x17094,	0x0003},
	{0x17098,	0x0003},
	{0x1709C,	0x0003},
	{0x170A0,	0x0003},
	{0x170A4,	0x0003},
	{0x170A8,	0x0003},
	{0x170AC,	0x0003},
	{0x170B0,	0x0003},
	{0x2B80000,	0x1893},
	{0x2B80008,	0x1893},
	{0x2BC3140,	0x5555},
	{0x2BC3140,	0xAAAA},
	{0x2BC3140,	0x80AAAA},
	{0x2BC3000,	0xFFFFFF},
	{0x2BC3004,	0xFFFFFF},
	{0x2BC3008,	0xFF0000},
	{0x2BC300C,	0xFFFFFF},
	{0x2BC3014,	0xFFFFFF},
	{0x2BC3018,	0x0000},
	{0x2BC301C,	0x0000},
	{0x2BC3020,	0x0000},
	{0x2BC3024,	0x000F},
	{0x2BC302C,	0x0000},
	{0x2BC3030,	0x0000},
	{0x2BC3034,	0x0000},
	{0x2BC3038,	0x0000},
	{0x2BC303C,	0x0000},
	{0x2BC3044,	0x0000},
	{0x2BC3048,	0x0000},
	{0x2BC304C,	0x0000},
	{0x2BC3050,	0x0000},
	{0x2BC3054,	0x0000},
	{0x2BC305C,	0x0000},
	{0x18014,	0x0008},
	{0x18130,	0xFF00000C},
	{0x2B80080,	0x0001},
	{0x2B80088,	0x0001},
	{0x2B80090,	0x0001},
	{0x2B80098,	0x0001},
	{0x2B800A0,	0x0001},
	{0x2B800A8,	0x0001},
	{0x2B800B0,	0x0001},
	{0x2B800B8,	0x0001},
	{0x2B80280,	0x0001},
	{0x2B80288,	0x0001},
	{0x2B80290,	0x0001},
	{0x2B80298,	0x0001},
	{0x2B802A0,	0x0001},
	{0x2B802A8,	0x0001},
	{0x2B802B0,	0x0001},
	{0x2B802B8,	0x0001},
	{0x4014,	0x20000000}
};

#define CS48L32_DSP_PROGRAM_REG	(84)
static const uint32_t cs48l32_dsp_program[CS48L32_DSP_PROGRAM_REG][2] =
{
	{0x82BC1000,	0x0},
	{0x2BC1000,		0x0000},
	{0x82BC7000,	0x0},
	{0x8001700C,	0x0},
	{0x80017010,	0x0},
	{0x80017014,	0x0},
	{0x80017018,	0x0},
	{0x8001701C,	0x0},
	{0x80017020,	0x0},
	{0x80017024,	0x0},
	{0x80017028,	0x0},
	{0x8001702C,	0x0},
	{0x80017030,	0x0},
	{0x80017034,	0x0},
	{0x80017038,	0x0},
	{0x8001703C,	0x0},
	{0x80017040,	0x0},
	{0x80017044,	0x0},
	{0x80017048,	0x0},
	{0x8001704C,	0x0},
	{0x80017050,	0x0},
	{0x80017054,	0x0},
	{0x80017058,	0x0},
	{0x8001705C,	0x0},
	{0x80017060,	0x0},
	{0x80017064,	0x0},
	{0x80017068,	0x0},
	{0x8001706C,	0x0},
	{0x1700C,		0x0003},
	{0x17010,		0x0003},
	{0x17014,		0x0003},
	{0x17018,		0x0003},
	{0x1701C,		0x0003},
	{0x17020,		0x0003},
	{0x17024,		0x0003},
	{0x17028,		0x0003},
	{0x1702C,		0x0003},
	{0x17030,		0x0003},
	{0x17034,		0x0003},
	{0x17038,		0x0003},
	{0x1703C,		0x0003},
	{0x17040,		0x0003},
	{0x17044,		0x0003},
	{0x17048,		0x0003},
	{0x1704C,		0x0003},
	{0x17050,		0x0003},
	{0x17054,		0x0003},
	{0x17058,		0x0003},
	{0x1705C,		0x0003},
	{0x17060,		0x0003},
	{0x17064,		0x0003},
	{0x17068,		0x0003},
	{0x1706C,		0x0003},
	{0x80017070,	0x0},
	{0x80017074,	0x0},
	{0x80017078,	0x0},
	{0x8001707C,	0x0},
	{0x80017080,	0x0},
	{0x80017084,	0x0},
	{0x80017088,	0x0},
	{0x8001708C,	0x0},
	{0x80017090,	0x0},
	{0x17070,		0x0003},
	{0x17074,		0x0003},
	{0x17078,		0x0003},
	{0x1707C,		0x0003},
	{0x17080,		0x0003},
	{0x17084,		0x0003},
	{0x17088,		0x0003},
	{0x1708C,		0x0003},
	{0x17090,		0x0003},
	{0x82B80008,	0x0},
	{0x82BC3140,	0x0},
	{0x2BC3008,		0x0000},
	{0x2BC300C,		0xC047F},
	{0x2BC3024,		0x000F},
	{0x2BC300C,		0xC0470},
	{0x2BC3024,		0x000C},
	{0x2BC3014,		0x0000},
	{0x2BC302C,		0x0000},
	{0x2BC3014,		0x0000},
	{0x2BC302C,		0x0000},
	{0x82BC1000,	0x0},
	{0x82BC1000,	0x0}
};

#if defined(CTC_CS48L32_TUNE_1ST)
#define CS48L32_TUNE_1ST_REG	(211)
static const uint32_t cs48l32_tune_1st[CS48L32_TUNE_1ST_REG][2] =
{
	{0x2800424,	0x0001},
	{0x2800428,	0x0000},
	{0x342D4A8,	0x0001},
	{0x342D4AC,	0x80000},
	{0x342D5AC,	0x80000},
#if defined(CTC_CS48L32_TUNE_2ND_COWAY)
	{0x342D5B0,	0x2CFCC0},
#else
	{0x342D5B0,	0x168C0C},
#endif
	{0x342D5B4,	0x0001},
	{0x342D5B8,	0x0001},
	{0x342D5BC,	0x0040},
	{0x342D5C0,	0x4000},
	{0x342D5C4,	0x4000},
	{0x342D5C8,	0x666666},
	{0x342D5CC,	0x0001},
	{0x342D5D0,	0x333333},
	{0x342D5D4,	0x0001},
	{0x342D5D8,	0x0001},
	{0x342D5DC,	0x0000},
	{0x342D5E4,	0x1030E},
	{0x342D5E8,	0x1062},
	{0x342D5EC,	0x1388},
	{0x342D5F0,	0x0000},
	{0x342D5F4,	0x2D6A16},
	{0x342D5F8,	0x2D6A16},
	{0x342D5FC,	0x404EA},
	{0x342D600,	0x2D17C2},
	{0x342D604,	0x0001},
	{0x342D608,	0x0000},
	{0x342D60C,	0x0000},
	{0x342D610,	0x0000},
	{0x342D614,	0x0000},
	{0x342D618,	0x0000},
	{0x342D61C,	0x0000},
	{0x342D620,	0x10000},
	{0x342D624,	0x10000},
	{0x342D628,	0x10000},
	{0x342D62C,	0x10000},
	{0x342D630,	0x400000},
	{0x342D634,	0x0000},
	{0x342D638,	0x0000},
	{0x342D63C,	0x0000},
	{0x342D640,	0x0000},
	{0x342D644,	0x400000},
	{0x342D648,	0x0000},
	{0x342D64C,	0x0000},
	{0x342D650,	0x0000},
	{0x342D654,	0x0000},
	{0x342D658,	0x400000},
	{0x342D65C,	0x0000},
	{0x342D660,	0x0000},
	{0x342D664,	0x0000},
	{0x342D668,	0x0000},
	{0x342D66C,	0x400000},
	{0x342D670,	0x0000},
	{0x342D674,	0x0000},
	{0x342D678,	0x0000},
	{0x342D67C,	0x0000},
	{0x342D6A0,	0x0000},
	{0x342D6A4,	0x76147B},
	{0x342D6A8,	0x500000},
	{0x342D6AC,	0x3E51EC},
	{0x342D6B0,	0x147AE1},
	{0x342D758,	0x80000},
#if defined(CTC_CS48L32_TUNE_2ND_COWAY)
	{0x342D75C,	0x2CFCC0},
#else
	{0x342D75C,	0x168C0C},
#endif
	{0x342D760,	0x0001},
	{0x342D764,	0x0001},
	{0x342D768,	0x0040},
	{0x342D76C,	0x4000},
	{0x342D770,	0x4000},
	{0x342D774,	0x666666},
	{0x342D778,	0x0001},
	{0x342D77C,	0x333333},
	{0x342D780,	0x0001},
	{0x342D784,	0x0001},
	{0x342D788,	0x0000},
	{0x342D790,	0x1030E},
	{0x342D794,	0x1062},
	{0x342D798,	0x1388},
	{0x342D79C,	0x0000},
	{0x342D7A0,	0x2D6A16},
	{0x342D7A4,	0x2D6A16},
	{0x342D7A8,	0x404EA},
	{0x342D7AC,	0x2D17C2},
	{0x342D7B0,	0x0001},
	{0x342D7B4,	0x0000},
	{0x342D7B8,	0x0000},
	{0x342D7BC,	0x0000},
	{0x342D7C0,	0x0000},
	{0x342D7C4,	0x0000},
	{0x342D7C8,	0x0000},
	{0x342D7CC,	0x10000},
	{0x342D7D0,	0x10000},
	{0x342D7D4,	0x10000},
	{0x342D7D8,	0x10000},
	{0x342D7DC,	0x400000},
	{0x342D7E0,	0x0000},
	{0x342D7E4,	0x0000},
	{0x342D7E8,	0x0000},
	{0x342D7EC,	0x0000},
	{0x342D7F0,	0x400000},
	{0x342D7F4,	0x0000},
	{0x342D7F8,	0x0000},
	{0x342D7FC,	0x0000},
	{0x342D800,	0x0000},
	{0x342D804,	0x400000},
	{0x342D808,	0x0000},
	{0x342D80C,	0x0000},
	{0x342D810,	0x0000},
	{0x342D814,	0x0000},
	{0x342D818,	0x400000},
	{0x342D81C,	0x0000},
	{0x342D820,	0x0000},
	{0x342D824,	0x0000},
	{0x342D828,	0x0000},
	{0x342D84C,	0x0000},
	{0x342D850,	0x76147B},
	{0x342D854,	0x500000},
	{0x342D858,	0x3E51EC},
	{0x342D85C,	0x147AE1},
	{0x342F008,	0x80000},
#if defined(CTC_CS48L32_TUNE_2ND_COWAY)
	{0x342F00C,	0xB4CE0},
#else
	{0x342F00C,	0xFF64C},
#endif
	{0x342F010,	0x80000},
	{0x342F014,	0x80000},
	{0x342F018,	0x0000},
	{0x342F01C,	0x0001},
	{0x342F024,	0x0000},
	{0x342F028,	0x0001},
	{0x342F030,	0x0020},
	{0x342F034,	0x012C},
	{0x342F038,	0x8000},
	{0x342F03C,	0x0009},
	{0x342F040,	0x0000},
	{0x342F044,	0x0020},
	{0x342F048,	0x012C},
	{0x342F050,	0x0009},
	{0x342F05C,	0x66666},
	{0x342F060,	0x0001},
	{0x342F074,	0x16D5D},
	{0x342F078,	0x4189},
	{0x342F080,	0x2DC5D},
	{0x342F084,	0x0001},
	{0x342F088,	0x2C1C2C},
	{0x342F08C,	0x0001},
	{0x342F090,	0x0001},
	{0x342F094,	0xFFFFEC},
	{0x342F098,	0x001E},
	{0x342F09C,	0xA0000},
	{0x342F0A0,	0x0001},
	{0x342F0A4,	0x200000},
	{0x342F0A8,	0x200000},
	{0x342F0AC,	0x0000},
	{0x342F0B0,	0x0001},
	{0x342F0B4,	0x0001},
	{0x342F0B8,	0x0000},
	{0x342F0BC,	0x0000},
	{0x342F0C0,	0x0000},
	{0x342F0C4,	0x0001},
	{0x342F0C8,	0x0037},
	{0x342F0CC,	0x154D},
	{0x342F0D0,	0x0001},
	{0x342F0D4,	0x0000},
	{0x342F0D8,	0x287A27},
	{0x342F0DC,	0x5A7EFA},
	{0x342F0E0,	0x404DE},
	{0x342F0E4,	0x3298B0},
	{0x342F0E8,	0x0001},
	{0x342F0EC,	0x0000},
	{0x342F0F0,	0x0000},
	{0x342F0F4,	0x0000},
	{0x342F0F8,	0x0000},
	{0x342F0FC,	0x0000},
	{0x342F100,	0x0000},
	{0x342F104,	0x10000},
	{0x342F108,	0x10000},
	{0x342F10C,	0x10000},
	{0x342F110,	0x10000},
	{0x342F114,	0x400000},
	{0x342F118,	0x0000},
	{0x342F11C,	0x0000},
	{0x342F120,	0x0000},
	{0x342F124,	0x0000},
	{0x342F128,	0x400000},
	{0x342F12C,	0x0000},
	{0x342F130,	0x0000},
	{0x342F134,	0x0000},
	{0x342F138,	0x0000},
	{0x342F13C,	0x400000},
	{0x342F140,	0x0000},
	{0x342F144,	0x0000},
	{0x342F148,	0x0000},
	{0x342F14C,	0x0000},
	{0x342F150,	0x400000},
	{0x342F154,	0x0000},
	{0x342F158,	0x0000},
	{0x342F15C,	0x0000},
	{0x342F160,	0x0000},
	{0x342F164,	0x0000},
	{0x342F1FC,	0x0001},
	{0x342F200,	0x80000},
	{0x342F204,	0x80000},
	{0x342F23C,	0x0000},
	{0x342F240,	0x0000},
	{0x342F244,	0x0000},
	{0x342F248,	0x0000},
	{0x342F24C,	0x0000},
	{0x342F250,	0x0000},
	{0x342F254,	0x0000},
	{0x342F258,	0x0000},
	{0x342F25C,	0x0000},
	{0x342F260,	0x0000},
	{0x342F264,	0x0000},
	{0x342F268,	0x0000}
};
#endif

#if defined(CTC_CS48L32_WMFW_20190826)
#define CS48L32_DSP_START_REG	(161)
static const uint32_t cs48l32_dsp_start[CS48L32_DSP_START_REG][2] =
{
	{0x1700C,	0x0003},
	{0x17010,	0x0003},
	{0x17014,	0x0003},
	{0x17018,	0x0003},
	{0x1701C,	0x0003},
	{0x17020,	0x0003},
	{0x17024,	0x0003},
	{0x17028,	0x0003},
	{0x1702C,	0x0003},
	{0x17030,	0x0003},
	{0x17034,	0x0003},
	{0x17038,	0x0003},
	{0x1703C,	0x0003},
	{0x17040,	0x0003},
	{0x17044,	0x0003},
	{0x17048,	0x0003},
	{0x1704C,	0x0003},
	{0x17050,	0x0003},
	{0x17054,	0x0003},
	{0x17058,	0x0003},
	{0x1705C,	0x0003},
	{0x17060,	0x0003},
	{0x17064,	0x0003},
	{0x17068,	0x0003},
	{0x1706C,	0x0003},
	{0x17070,	0x0003},
	{0x17074,	0x0003},
	{0x17078,	0x0003},
	{0x1707C,	0x0003},
	{0x17080,	0x0003},
	{0x17084,	0x0003},
	{0x17088,	0x0003},
	{0x1708C,	0x0003},
	{0x17090,	0x0003},
	{0x2BC1000,	0x0001},
	{0x342E868,	0x000D},
	{0x342E868,	0x000D},
	{0x342E404,	0x000E},
	{0x342E404,	0x000E},
	{0x342E404,	0x0000},
	{0x342E404,	0x0002},
	{0x342E6D8,	0x0001},
	{0x342E400,	0x0014},
	{0x342E6DC,	0x333333},
	{0x342E400,	0x0014},
	{0x342E6F0,	0x1030E},
	{0x342E400,	0x0014},
	{0x342E6F4,	0x1062},
	{0x342E400,	0x0014},
	{0x342E6F8,	0x1388},
	{0x342E400,	0x0014},
	{0x342E6D8,	0x0001},
	{0x342E400,	0x0014},
	{0x342E6DC,	0x333333},
	{0x342E400,	0x0014},
	{0x342E6F0,	0x1030E},
	{0x342E400,	0x0014},
	{0x342E6F4,	0x1062},
	{0x342E400,	0x0014},
	{0x342E6F8,	0x1388},
	{0x342E400,	0x0014},
	{0x342E6E0,	0x0001},
	{0x342E400,	0x0014},
	{0x342E6E4,	0x0001},
	{0x342E400,	0x0014},
	{0x342E6E0,	0x0001},
	{0x342E400,	0x0014},
	{0x342E6E4,	0x0001},
	{0x342E400,	0x0014},
	{0x342E6E8,	0x0000},
	{0x342E400,	0x0014},
	{0x342E6E8,	0x0000},
	{0x342E400,	0x0014},
	{0x342E6FC,	0x0000},
	{0x342E400,	0x0015},
	{0x342E704,	0x2D6A16},
	{0x342E400,	0x0015},
	{0x342E700,	0x2D6A16},
	{0x342E400,	0x0015},
	{0x342E70C,	0x2D17C2},
	{0x342E400,	0x0015},
	{0x342E708,	0x404EA},
	{0x342E400,	0x0015},
	{0x342E6FC,	0x0000},
	{0x342E400,	0x0015},
	{0x342E704,	0x2D6A16},
	{0x342E400,	0x0015},
	{0x342E700,	0x2D6A16},
	{0x342E400,	0x0015},
	{0x342E70C,	0x2D17C2},
	{0x342E400,	0x0015},
	{0x342E708,	0x404EA},
	{0x342E400,	0x0015},
	{0x342E718,	0x0000},
	{0x342E400,	0x0016},
	{0x342E714,	0x0000},
	{0x342E400,	0x0016},
	{0x342E710,	0x0001},
	{0x342E400,	0x0016},
	{0x342E718,	0x0000},
	{0x342E400,	0x0016},
	{0x342E714,	0x0000},
	{0x342E400,	0x0016},
	{0x342E710,	0x0001},
	{0x342E400,	0x0016},
	{0x342E71C,	0x0000},
	{0x342E72C,	0x10000},
	{0x342E73C,	0x400000},
	{0x342E71C,	0x0000},
	{0x342E72C,	0x10000},
	{0x342E73C,	0x400000},
	{0x342E720,	0x0000},
	{0x342E730,	0x10000},
	{0x342E750,	0x400000},
	{0x342E720,	0x0000},
	{0x342E730,	0x10000},
	{0x342E750,	0x400000},
	{0x342E724,	0x0000},
	{0x342E734,	0x10000},
	{0x342E764,	0x400000},
	{0x342E724,	0x0000},
	{0x342E734,	0x10000},
	{0x342E764,	0x400000},
	{0x342E728,	0x0000},
	{0x342E738,	0x10000},
	{0x342E778,	0x400000},
	{0x342E728,	0x0000},
	{0x342E738,	0x10000},
	{0x342E778,	0x400000},
	{0x342E78C,	0x0000},
	{0x342E400,	0x0017},
	{0x342E790,	0x0000},
	{0x342E400,	0x0017},
	{0x342E794,	0x0000},
	{0x342E400,	0x0017},
	{0x342E798,	0x0000},
	{0x342E400,	0x0017},
	{0x342E79C,	0x0000},
	{0x342E400,	0x0017},
	{0x342E7A0,	0x0000},
	{0x342E400,	0x0017},
	{0x342E7A4,	0x0000},
	{0x342E400,	0x0017},
	{0x342E7A8,	0x0000},
	{0x342E400,	0x0017},
	{0x342E78C,	0x0000},
	{0x342E400,	0x0017},
	{0x342E790,	0x0000},
	{0x342E400,	0x0017},
	{0x342E794,	0x0000},
	{0x342E400,	0x0017},
	{0x342E798,	0x0000},
	{0x342E400,	0x0017},
	{0x342E79C,	0x0000},
	{0x342E400,	0x0017},
	{0x342E7A0,	0x0000},
	{0x342E400,	0x0017},
	{0x342E7A4,	0x0000},
	{0x342E400,	0x0017},
	{0x342E7A8,	0x0000},
	{0x342E400,	0x0017}
};
#elif defined(CTC_CS48L32_WMFW_12062019)
#define CS48L32_DSP_START_REG	(207)
static const uint32_t cs48l32_dsp_start[CS48L32_DSP_START_REG][2] =
{
	{0x1700C,	0x0003},
	{0x17010,	0x0003},
	{0x17014,	0x0003},
	{0x17018,	0x0003},
	{0x1701C,	0x0003},
	{0x17020,	0x0003},
	{0x17024,	0x0003},
	{0x17028,	0x0003},
	{0x1702C,	0x0003},
	{0x17030,	0x0003},
	{0x17034,	0x0003},
	{0x17038,	0x0003},
	{0x1703C,	0x0003},
	{0x17040,	0x0003},
	{0x17044,	0x0003},
	{0x17048,	0x0003},
	{0x1704C,	0x0003},
	{0x17050,	0x0003},
	{0x17054,	0x0003},
	{0x17058,	0x0003},
	{0x1705C,	0x0003},
	{0x17060,	0x0003},
	{0x17064,	0x0003},
	{0x17068,	0x0003},
	{0x1706C,	0x0003},
	{0x17070,	0x0003},
	{0x17074,	0x0003},
	{0x17078,	0x0003},
	{0x1707C,	0x0003},
	{0x17080,	0x0003},
	{0x17084,	0x0003},
	{0x17088,	0x0003},
	{0x1708C,	0x0003},
	{0x17090,	0x0003},
	{0x2BC1000,	0x0001},
	{0x342F004,	0x000D},
	{0x342F004,	0x000D},
	{0x342F004,	0x000A},
	{0x342F004,	0x000A},
	{0x342F004,	0x000A},
	{0x342F004,	0x000A},
	{0x342F004,	0x000A},
	{0x342F004,	0x000A},
	{0x342F004,	0x000A},
	{0x342F004,	0x000A},
	{0x342F004,	0x000A},
	{0x342F004,	0x000A},
	{0x342F004,	0x000A},
	{0x342F004,	0x000A},
	{0x342F004,	0x000A},
	{0x342F004,	0x000A},
	{0x342D4A4,	0x000E},
	{0x342D4A4,	0x000E},
	{0x342D4A4,	0x0000},
	{0x342D4A4,	0x0002},
	{0x342D758,	0x80000},
	{0x342D4A0,	0x0011},
#if defined(CTC_CS48L32_TUNE_2ND_COWAY)
	{0x342D75C, 0x2CFCC0},
#else
#if defined(CTC_CS48L32_TUNE_1ST)
	{0x342D75C,	0x168C0C},
#else
	{0x342D75C,	0x80000},
#endif
#endif
	{0x342D4A0,	0x0011},
	{0x342D758,	0x80000},
	{0x342D4A0,	0x0011},
#if defined(CTC_CS48L32_TUNE_2ND_COWAY)
	{0x342D75C, 0x2CFCC0},
#else
#if defined(CTC_CS48L32_TUNE_1ST)
	{0x342D75C,	0x168C0C},
#else
	{0x342D75C,	0x80000},
#endif
#endif
	{0x342D4A0,	0x0011},
	{0x342D760,	0x0001},
	{0x342D4A0,	0x0012},
	{0x342D764,	0x0001},
	{0x342D4A0,	0x0012},
	{0x342D768,	0x0040},
	{0x342D4A0,	0x0013},
#if defined(CTC_CS48L32_TUNE_1ST)
	{0x342D76C,	0x4000},
#else
	{0x342D76C,	0x8000},
#endif
	{0x342D4A0,	0x0013},
#if defined(CTC_CS48L32_TUNE_1ST)
	{0x342D770,	0x4000},
#else
	{0x342D770,	0x0000},
#endif
	{0x342D4A0,	0x0013},
	{0x342D774,	0x666666},
	{0x342D4A0,	0x0013},
	{0x342D760,	0x0001},
	{0x342D4A0,	0x0012},
	{0x342D764,	0x0001},
	{0x342D4A0,	0x0012},
	{0x342D768,	0x0040},
	{0x342D4A0,	0x0013},
#if defined(CTC_CS48L32_TUNE_1ST)
	{0x342D76C,	0x4000},
#else
	{0x342D76C,	0x8000},
#endif
	{0x342D4A0,	0x0013},
#if defined(CTC_CS48L32_TUNE_1ST)
	{0x342D770,	0x4000},
#else
	{0x342D770,	0x0000},
#endif
	{0x342D4A0,	0x0013},
	{0x342D774,	0x666666},
	{0x342D4A0,	0x0013},
	{0x342D778,	0x0001},
	{0x342D4A0,	0x0014},
	{0x342D77C,	0x333333},
	{0x342D4A0,	0x0014},
	{0x342D790,	0x1030E},
	{0x342D4A0,	0x0014},
	{0x342D794,	0x1062},
	{0x342D4A0,	0x0014},
	{0x342D798,	0x1388},
	{0x342D4A0,	0x0014},
	{0x342D778,	0x0001},
	{0x342D4A0,	0x0014},
	{0x342D77C,	0x333333},
	{0x342D4A0,	0x0014},
	{0x342D790,	0x1030E},
	{0x342D4A0,	0x0014},
	{0x342D794,	0x1062},
	{0x342D4A0,	0x0014},
	{0x342D798,	0x1388},
	{0x342D4A0,	0x0014},
	{0x342D780,	0x0001},
	{0x342D4A0,	0x0014},
	{0x342D784,	0x0001},
	{0x342D4A0,	0x0014},
	{0x342D780,	0x0001},
	{0x342D4A0,	0x0014},
	{0x342D784,	0x0001},
	{0x342D4A0,	0x0014},
	{0x342D788,	0x0000},
	{0x342D4A0,	0x0014},
	{0x342D788,	0x0000},
	{0x342D4A0,	0x0014},
	{0x342D79C,	0x0000},
	{0x342D4A0,	0x0015},
	{0x342D7A4,	0x2D6A16},
	{0x342D4A0,	0x0015},
	{0x342D7A0,	0x2D6A16},
	{0x342D4A0,	0x0015},
	{0x342D7AC,	0x2D17C2},
	{0x342D4A0,	0x0015},
	{0x342D7A8,	0x404EA},
	{0x342D4A0,	0x0015},
	{0x342D79C,	0x0000},
	{0x342D4A0,	0x0015},
	{0x342D7A4,	0x2D6A16},
	{0x342D4A0,	0x0015},
	{0x342D7A0,	0x2D6A16},
	{0x342D4A0,	0x0015},
	{0x342D7AC,	0x2D17C2},
	{0x342D4A0,	0x0015},
	{0x342D7A8,	0x404EA},
	{0x342D4A0,	0x0015},
	{0x342D7B8,	0x0000},
	{0x342D4A0,	0x0016},
	{0x342D7B4,	0x0000},
	{0x342D4A0,	0x0016},
	{0x342D7B0,	0x0001},
	{0x342D4A0,	0x0016},
	{0x342D7B8,	0x0000},
	{0x342D4A0,	0x0016},
	{0x342D7B4,	0x0000},
	{0x342D4A0,	0x0016},
	{0x342D7B0,	0x0001},
	{0x342D4A0,	0x0016},
	{0x342D7BC,	0x0000},
	{0x342D7CC,	0x10000},
	{0x342D7DC,	0x400000},
	{0x342D7BC,	0x0000},
	{0x342D7CC,	0x10000},
	{0x342D7DC,	0x400000},
	{0x342D7C0,	0x0000},
	{0x342D7D0,	0x10000},
	{0x342D7F0,	0x400000},
	{0x342D7C0,	0x0000},
	{0x342D7D0,	0x10000},
	{0x342D7F0,	0x400000},
	{0x342D7C4,	0x0000},
	{0x342D7D4,	0x10000},
	{0x342D804,	0x400000},
	{0x342D7C4,	0x0000},
	{0x342D7D4,	0x10000},
	{0x342D804,	0x400000},
	{0x342D7C8,	0x0000},
	{0x342D7D8,	0x10000},
	{0x342D818,	0x400000},
	{0x342D7C8,	0x0000},
	{0x342D7D8,	0x10000},
	{0x342D818,	0x400000},
	{0x342D82C,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D830,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D834,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D838,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D83C,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D840,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D844,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D848,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D82C,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D830,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D834,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D838,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D83C,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D840,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D844,	0x0000},
	{0x342D4A0,	0x0017},
	{0x342D848,	0x0000},
	{0x342D4A0,	0x0017}
};
#endif

#if defined(CTC_CS48L32_FLL_ASP1_BCLK)
#if defined(EMO_ROBOT_16khz)
#define CS48L32_FLL_CHANGE_REG	(59)
#else
#define CS48L32_FLL_CHANGE_REG	(57)
#endif
static const uint32_t cs48l32_fll_change[CS48L32_FLL_CHANGE_REG][2] =
{
#if defined(EMO_ROBOT_16khz)
	{0x1C00,	0x0003},
	{0x1C04,	0x88608030},
	{0x1C08,	0x10000},
	{0x1C0C,	0x22F25002},
	{0x1C00,	0x0007},
	{0x1C00,	0x0005},
#else
	{0x1C04,	0x88608020},
	{0x1C08,	0x10000},
	{0x1C0C,	0x21F05001},
	{0x1C00,	0x0005},
#endif
	{0x1400,	0x0042},
	{0x1404,	0x0444},
	{0x1424,	0x0012},
	{0x1420,	0x0012},
#if defined(EMO_ROBOT_16khz)
	{0xA800,	0x0800},
#else
	{0xA800,	0x1000},
#endif
	{0x2000,	0x0007},
	{0x2410,	0x00E7},
	{0x2418,	0x0223},	// Jace. MICBIAS1A
	{0x4008,	0x0000},
	{0x4020,	0x20020},
	{0x4060,	0x20020},
	{0x4024,	0x0000},
	{0x4044,	0x0000},
	{0x4000,	0x0003},
#if defined(CTC_CS48L32_TUNE_2ND_COWAY)
	{0x4028,	0x8000BC},	// IN1L_MUTE=0, IN1L_VOL=0dB, IN1L_PGA_VOL=30dB
	{0x4048,	0x8000BC},	// IN1L_MUTE=0, IN1L_VOL=0dB, IN1L_PGA_VOL=30dB
#else
#if defined(CTC_CS48L32_TUNE_1ST)
	{0x4028,	0x8000BC},	// IN1L_MUTE=0, IN1L_VOL=0dB, IN1L_PGA_VOL=30dB
	{0x4048,	0x8000BC},	// IN1L_MUTE=0, IN1L_VOL=0dB, IN1L_PGA_VOL=30dB
#else
	{0x4028,	0x8000A6},
	{0x4048,	0x8000A6},
#endif
#endif
	{0xC10,		0xE1000000},
	{0xC14,		0xE1000000},
	{0xC18,		0xE1000000},
	{0xC1C,		0xE1000000},
	{0xC20,		0xE1000000},
	{0xC24,		0xE1000000},
	{0xC28,		0xE1000000},
	{0xC2C,		0xE1000000},
#if defined(EMO_ROBOT_16khz)
	{0x6004,	0x0112},
#else
	{0x6004,	0x0221},
#endif
	{0x6008,	0x20200200},
	{0x6040,	0x0010},
	{0x6030,	0x0010},
	{0x6000,	0x30003},
#if defined(EMO_ROBOT_16khz)
	{0x6084,	0x0112},
#else
	{0x6084,	0x0221},
#endif
	{0x6088,	0x20200233},
	{0x60C0,	0x0010},
	{0x60B0,	0x0010},
	{0x6080,	0x0003},
	{0x608C,	0x0000},
	{0xA400,	0x1000},
	{0xA404,	0x0C03},
	{0x8B80,	0x800020},
	{0x8B84,	0x800021},
#if defined(CTC_CS48L32_TUNE_1ST)
	{0x8300,	0x6200B8},
	{0x8310,	0x6200B8},
#else
	{0x8300,	0x6E80B8},
	{0x8310,	0x6E80B8},
#endif
#if defined(EMO_ROBOT_16khz)
	{0x89C0,	0x0000},
	{0x89D0,	0x0000},
	{0x9000,	0x8000B8},
	{0x9020,	0x8000B8},
#else
	{0x89C0,	0x00B8},
	{0x89D0,	0x00B8},
	{0x9000,	0x80009C},
	{0x9020,	0x80009D},
#endif
	{0x9040,	0x800010},
	{0x9050,	0x800011},
#if defined(EMO_ROBOT_16khz)
	{0x89A0,	0x0000},
	{0x89B0,	0x0000},
	{0x8200,	0x800102},
	{0x8210,	0x800102},
#else
	{0x89A0,	0x0102},
	{0x89B0,	0x0103},
	{0x8200,	0x80009A},
	{0x8210,	0x80009A},	// Jace. 200110. Cirrus's DSP output pin 4 have delay result.
#endif
	{0xA808,	0x0001},
	{0x4014,	0x20000000}
};
#endif

#if defined(CTC_CS48L32_CHECK_REG)
#define CS48L32_CHECK_REG	(5)
static const uint32_t cs48l32_check_reg[CS48L32_CHECK_REG][2] =
{
	{0x8342D5B0,	0x0},
	{0x8342D75C,	0x0},
	{0x8342D76C,	0x0},
	{0x8342D770,	0x0},
	{0x8342F09C,	0x0},
};
#endif

#if defined(CTC_CS48L32_MUTE_CONTROL)
#define CS48L32_MUTE_CONTROL	(8)
static const uint32_t cs48l32_mute_control[CS48L32_MUTE_CONTROL][2] =
{
	{0x8300,	0x6200B8}, // Unmute
	{0x8310,	0x6200B8},
#if defined(EMO_ROBOT_16khz)
	{0x9000,	0x8080B8},
	{0x9020,	0x8080B8},
#else
	{0x9000,	0x80009C},
	{0x9020,	0x80009D},
#endif
	{0x8300,	0x620000}, // Mute
	{0x8310,	0x620000},
	{0x9000,	0x800000},
	{0x9020,	0x800000}
};
#endif

static void swap_endianness(uint8_t* out, uint8_t* in, uint8_t size)
{
	for (uint8_t i = 0; i < size; i++)
	{
		out[size - 1 - i] = in[i];
	}
}

#if defined(CTC_CS48L32_SENSORY)
esp_err_t cs_spi_sensory_ready(void);

#define CS48L32_SENSORY_READY_REG	(4)
#if defined(CTC_CS48L32_WMFW_20190826)
static const uint32_t cs48l32_sensory_ready[CS48L32_SENSORY_READY_REG][2] =
{
	{0x82800488,	0x0},
	{0x18030,		0x0001},
	{0x2800480,		0x0020},
	{0x2800480,		0x0001}
};
#elif defined(CTC_CS48L32_WMFW_12062019)
static const uint32_t cs48l32_sensory_ready[CS48L32_SENSORY_READY_REG][2] =
{
	{0x82800450,	0x0},
	{0x18030,		0x0001},
	{0x2800448,		0x0020},
	{0x2800448,		0x0001}
};
#endif

esp_err_t cs_spi_sensory_ready(void)
{
	esp_err_t ret = ESP_OK;

	for(uint8_t i = 0; i < CS48L32_SENSORY_READY_REG; i++)
	{
		spi_transaction_t t;
		memset(&t, 0, sizeof(t));

		uint8_t* dataOut = (uint8_t*)malloc(12);
		assert(0 != dataOut);

		t.length = 96;

		swap_endianness(&dataOut[0], (uint8_t*)&cs48l32_sensory_ready[i][0], 4);
		swap_endianness(&dataOut[4], (uint8_t*)&cs48l32_spi_padding, 4);
		swap_endianness(&dataOut[8], (uint8_t*)&cs48l32_sensory_ready[i][1], 4);

		t.tx_buffer = dataOut;

		ret = spi_device_transmit(g_spi, &t);

		if (dataOut)
		{
			free(dataOut);
			dataOut = NULL;
		}
	}

	ESP_LOGE(TAG, "[CS48L32] Sensory ready");

	return ret;
}

#define CS48L32_SENSORY_DISABLE_REG	(2)
#if defined(CTC_CS48L32_WMFW_20190826)
static const uint32_t cs48l32_sensory_disable[CS48L32_SENSORY_DISABLE_REG][2] =
{
	{0x82800488,	0x0},
	{0x2800488,		0x0000}
};
#elif defined(CTC_CS48L32_WMFW_12062019)
static const uint32_t cs48l32_sensory_disable[CS48L32_SENSORY_DISABLE_REG][2] =
{
	{0x82800450,	0x0},
	{0x2800450,		0x0000}
};
#endif

static esp_err_t cs_spi_sensory_disable(void)
{
	esp_err_t ret = ESP_OK;

	for(uint8_t i = 0; i < CS48L32_SENSORY_DISABLE_REG; i++)
	{
		spi_transaction_t t;
		memset(&t, 0, sizeof(t));

		uint8_t* dataOut = (uint8_t*)malloc(12);
		assert(0 != dataOut);

		t.length = 96;

		swap_endianness(&dataOut[0], (uint8_t*)&cs48l32_sensory_disable[i][0], 4);
		swap_endianness(&dataOut[4], (uint8_t*)&cs48l32_spi_padding, 4);
		swap_endianness(&dataOut[8], (uint8_t*)&cs48l32_sensory_disable[i][1], 4);

		t.tx_buffer = dataOut;

		ret = spi_device_transmit(g_spi, &t);

		if (dataOut)
		{
			free(dataOut);
			dataOut = NULL;
		}
	}

	ESP_LOGE(TAG, "[CS48L32] Sensory disable");

	return ret;
}
#endif

static void ak_reset(void)
{
	gpio_pad_select_gpio(GPIO_AK_PDN);
	gpio_set_direction(GPIO_AK_PDN, GPIO_MODE_OUTPUT);

	gpio_set_level(GPIO_AK_PDN, 1);
	vTaskDelay(500 / portTICK_PERIOD_MS);
	gpio_set_level(GPIO_AK_PDN, 0);
	vTaskDelay(500 / portTICK_PERIOD_MS);
	gpio_set_level(GPIO_AK_PDN, 1);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
}

static void cs_reset(void)
{
	gpio_pad_select_gpio(GPIO_CS_RESET);
	gpio_set_direction(GPIO_CS_RESET, GPIO_MODE_OUTPUT);

	gpio_set_level(GPIO_CS_RESET, 1);
	vTaskDelay(500 / portTICK_PERIOD_MS);
	gpio_set_level(GPIO_CS_RESET, 0);
	vTaskDelay(500 / portTICK_PERIOD_MS);
	gpio_set_level(GPIO_CS_RESET, 1);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
}

static esp_err_t cs_spi_init(void)
{
	esp_err_t ret = ESP_OK;

	//Configuration for the SPI bus
	spi_bus_config_t buscfg = {
		.mosi_io_num = GPIO_MOSI,
		.miso_io_num = GPIO_MISO,
		.sclk_io_num = GPIO_SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	//Configuration for the SPI device on the other side of the bus
	spi_device_interface_config_t devcfg = {
		.command_bits		= 0,
		.address_bits		= 0,
		.dummy_bits			= 0,
		.clock_speed_hz		= 6250000,
		.duty_cycle_pos		= 128,        //50% duty cycle
		.mode				= 0,
		.spics_io_num		= GPIO_CS,
		.cs_ena_posttrans	= 3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
		.queue_size			= 3
	};

	//Initialize the SPI bus and add the device we want to send stuff to.
	ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
	assert(ret == ESP_OK);
	ret = spi_bus_add_device(HSPI_HOST, &devcfg, &g_spi);
	assert(ret == ESP_OK);

	return ret;
}

static esp_err_t cs_spi_deinit(void)
{
	esp_err_t ret = ESP_OK;

    ret = spi_bus_remove_device(g_spi);
	assert(ret == ESP_OK);
    ret = spi_bus_free(HSPI_HOST);
	assert(ret == ESP_OK);

	return ret;
}

static esp_err_t cs_spi_firmware_write(void)
{
	esp_err_t ret = ESP_OK;

#if defined(CTC_CS48L32_WMFW_20190826)
	const char *filename = "/spiffs/SCSH_COOKE_20190826.wmfw";
#elif defined(CTC_CS48L32_WMFW_12062019)
	const char *filename = "/spiffs/SCSH_COOKE_12062019.wmfw";
#endif

	// Use settings defined above to initialize and mount SPIFFS filesystem.
	ctc_spiffs_init();
	// Choose which ADSP2 core to download to
	select_core();

	// WMFW file
	ret = (esp_err_t)ProcessWMFWFile(filename);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "[ 0 ] process %s wmfw file error : %d", filename, ret);
	}
	else {
		ESP_LOGE(TAG, "[ 0 ] process %s wmfw file success", filename);
	}

	free(algorithmIdBlocks);
    // All done, unmount partition and disable SPIFFS
    ret = esp_vfs_spiffs_unregister(NULL);

	return ret;
}

static esp_err_t cs_spi_register_block_write(uint8_t reg_block)
{
	esp_err_t ret = ESP_OK;

	spi_transaction_t t;
	memset(&t, 0, sizeof(t));

	uint8_t* dataOut = (uint8_t*)malloc(28);
	assert(0 != dataOut);
	
	t.length = 224;
	
	swap_endianness(&dataOut[0], (uint8_t*)&cs48l32_dsp_start[reg_block][0], 4);
	swap_endianness(&dataOut[4], (uint8_t*)&cs48l32_spi_padding, 4);
	swap_endianness(&dataOut[8], (uint8_t*)&cs48l32_dsp_start[reg_block][1], 4);
	swap_endianness(&dataOut[12], (uint8_t*)&cs48l32_spi_padding, 4); // Block data is 0x0.
	swap_endianness(&dataOut[16], (uint8_t*)&cs48l32_spi_padding, 4);
	swap_endianness(&dataOut[20], (uint8_t*)&cs48l32_spi_padding, 4);
	swap_endianness(&dataOut[24], (uint8_t*)&cs48l32_spi_padding, 4);
	
	t.tx_buffer = dataOut;
	
	ret = spi_device_transmit(g_spi, &t);
	
	if (dataOut)
	{
		free(dataOut);
		dataOut = NULL;
	}

	return ret;
}

static esp_err_t cs_spi_register_write(uint8_t reg_start, uint8_t reg_end, uint8_t reg_type)
{
	esp_err_t ret = ESP_OK;

	for(uint8_t i = reg_start; i < reg_end; i++)
	{
#if defined(CTC_CS48L32_WMFW_20190826)
		if((reg_type == CS48L32_REG_TYPE_DSP_START) && (i == 107 || i == 110 || i == 113 || i == 116 || i == 119 || i == 122 || i == 125 || i == 128)) // Block write
		{
			cs_spi_register_block_write(i);
		}
		else
#elif defined(CTC_CS48L32_WMFW_12062019)
		if((reg_type == CS48L32_REG_TYPE_DSP_START) && (i == 153 || i == 156 || i == 159 || i == 162 || i == 165 || i == 168 || i == 171 || i == 174)) // Block write
		{
			cs_spi_register_block_write(i);
		}
		else
#endif
		{
			spi_transaction_t t;
			memset(&t, 0, sizeof(t));

			uint8_t* dataOut = (uint8_t*)malloc(12);
			assert(0 != dataOut);

			t.length = 96;

			switch(reg_type)
			{
				case CS48L32_REG_TYPE_CONFIG:
					swap_endianness(&dataOut[0], (uint8_t*)&cs48l32_config[i][0], 4);
					swap_endianness(&dataOut[4], (uint8_t*)&cs48l32_spi_padding, 4);
					swap_endianness(&dataOut[8], (uint8_t*)&cs48l32_config[i][1], 4);
					break;

				case CS48L32_REG_TYPE_DSP_PROGRAM:
					swap_endianness(&dataOut[0], (uint8_t*)&cs48l32_dsp_program[i][0], 4);
					swap_endianness(&dataOut[4], (uint8_t*)&cs48l32_spi_padding, 4);
					swap_endianness(&dataOut[8], (uint8_t*)&cs48l32_dsp_program[i][1], 4);
					break;

#if defined(CTC_CS48L32_TUNE_1ST)
				case CS48L32_REG_TYPE_TUNE_1ST:
					swap_endianness(&dataOut[0], (uint8_t*)&cs48l32_tune_1st[i][0], 4);
					swap_endianness(&dataOut[4], (uint8_t*)&cs48l32_spi_padding, 4);
					swap_endianness(&dataOut[8], (uint8_t*)&cs48l32_tune_1st[i][1], 4);
					break;
#endif

				case CS48L32_REG_TYPE_DSP_START:
					swap_endianness(&dataOut[0], (uint8_t*)&cs48l32_dsp_start[i][0], 4);
					swap_endianness(&dataOut[4], (uint8_t*)&cs48l32_spi_padding, 4);
					swap_endianness(&dataOut[8], (uint8_t*)&cs48l32_dsp_start[i][1], 4);
					break;

#if defined(CTC_CS48L32_FLL_ASP1_BCLK)
				case CS48L32_REG_TYPE_FLL_CHANGE:
					swap_endianness(&dataOut[0], (uint8_t*)&cs48l32_fll_change[i][0], 4);
					swap_endianness(&dataOut[4], (uint8_t*)&cs48l32_spi_padding, 4);
					swap_endianness(&dataOut[8], (uint8_t*)&cs48l32_fll_change[i][1], 4);
					break;
#endif

#if defined(CTC_CS48L32_CHECK_REG)
				case CS48L32_REG_TYPE_CHECK_REG:
					swap_endianness(&dataOut[0], (uint8_t*)&cs48l32_check_reg[i][0], 4);
					swap_endianness(&dataOut[4], (uint8_t*)&cs48l32_spi_padding, 4);
					swap_endianness(&dataOut[8], (uint8_t*)&cs48l32_check_reg[i][1], 4);
					break;
#endif

#if defined(CTC_CS48L32_MUTE_CONTROL)
				case CS48L32_REG_TYPE_MUTE_CONTROL:
					swap_endianness(&dataOut[0], (uint8_t*)&cs48l32_mute_control[i][0], 4);
					swap_endianness(&dataOut[4], (uint8_t*)&cs48l32_spi_padding, 4);
					swap_endianness(&dataOut[8], (uint8_t*)&cs48l32_mute_control[i][1], 4);
					break;
#endif

				default:
					ret = ESP_FAIL;
					break;
			}

			t.tx_buffer = dataOut;

			ret = spi_device_transmit(g_spi, &t);

			if (dataOut)
			{
				free(dataOut);
				dataOut = NULL;
			}

			if(reg_type == CS48L32_REG_TYPE_CONFIG && i == 0)
			{
				ESP_LOGE(TAG, "INSERT DELAY 1");
				vTaskDelay(1000 / portTICK_PERIOD_MS);
			}
		}
	}

	return ret;
}
#endif

#define GPIO_ESP_SW3		0
#define GPIO_ESP_CS_IRQ		21
#define GPIO_IRQ_PIN_SEL	((1ULL<<GPIO_ESP_SW3) | (1ULL<<GPIO_ESP_CS_IRQ))

#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
	uint32_t io_num;
	uint8_t toggle = 1;
#if defined(CTC_TRIGGER_TEST)
	uint32_t trigger_count = 0;
#endif

	for(;;) {
		if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
#if defined(CTC_CS48L32_SWITCH_TYPE)
			if((io_num == 0) && (gpio_get_level(io_num) == 0))
			{
				ESP_LOGE(TAG, "[AK4384VT] AMP mute.");
				gpio_set_level(GPIO_AK_PDN, 0);
#if defined(CTC_CS48L32_SENSORY)
				cs_spi_register_write((CS48L32_MUTE_CONTROL - 4), CS48L32_MUTE_CONTROL, CS48L32_REG_TYPE_MUTE_CONTROL);
#endif
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}
			else if((io_num == 0) && (gpio_get_level(io_num) == 1))
			{
				ESP_LOGE(TAG, "[AK4384VT] AMP unmute.");
				gpio_set_level(GPIO_AK_PDN, 1);
#if defined(CTC_CS48L32_SENSORY)
				cs_spi_register_write(0, (CS48L32_MUTE_CONTROL - 4), CS48L32_REG_TYPE_MUTE_CONTROL);
#endif
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}
#else
			if((io_num == 0) && (gpio_get_level(io_num) == 0))
			{
				toggle ^= (1 << 0);
				ESP_LOGE(TAG, "[AK4384VT] AMP PDN toggle[%d].", toggle);
				gpio_set_level(GPIO_AK_PDN, toggle);
#if defined(CTC_CS48L32_MUTE_CONTROL)
				if(toggle)
					cs_spi_register_write(0, (CS48L32_MUTE_CONTROL - 4), CS48L32_REG_TYPE_MUTE_CONTROL);
				else
					cs_spi_register_write((CS48L32_MUTE_CONTROL - 4), CS48L32_MUTE_CONTROL, CS48L32_REG_TYPE_MUTE_CONTROL);
#endif
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}
#endif
			else if((io_num == 21) && (gpio_get_level(io_num) == 0))
			{
#if defined(CTC_CS48L32_SENSORY_TRIGGER)
#if defined(CTC_TRIGGER_TEST)
				trigger_count++;
				ESP_LOGE(TAG, "[CS48L32] Sensory detection triggered. Count[%d]", trigger_count);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				cs_spi_sensory_ready();
#else
				ESP_LOGE(TAG, "[CS48L32] Sensory detection triggered.");
#if defined (EMO_ROBOT)
				gpio_set_level(TRI_LED, 0);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				cs_spi_sensory_ready();
				gpio_set_level(TRI_LED, 1);
#else
				va_dsp_tap_to_talk_start();
#endif
#endif
#endif
			}
		}
	}
}

static void esp_cs_irq_intr_init(void)
{
	gpio_config_t io_conf;
	//interrupt of rising edge
#if defined(CTC_CS48L32_SWITCH_TYPE)
	io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
#else
	io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
#endif
	//bit mask of the pins, use GPIO4/5 here
	io_conf.pin_bit_mask = GPIO_IRQ_PIN_SEL;
	//set as input mode    
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

	//create a queue to handle gpio event from isr
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	//start gpio task
	xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, (CONFIG_ESP32_PTHREAD_TASK_PRIO_DEFAULT - 1), NULL);

	//install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_ESP_SW3, gpio_isr_handler, (void*) GPIO_ESP_SW3);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_ESP_CS_IRQ, gpio_isr_handler, (void*) GPIO_ESP_CS_IRQ);
}

#if defined(BLYNK_APPS)
#define POWEROFF	0x0
#define POWERON		0x1
#define STEP1		0x2
#define STEP2		0x3
#define STEP3		0x4

#define BLY_VENT_POWER 10
#define BLY_VENT_STEP1 11
#define BLY_VENT_STEP2 12
#define BLY_VENT_STEP3 13

extern bool vent_power_on;
extern uint8_t vent_step;

extern uint8_t	_binary_00_bin_start, _binary_00_bin_end, _binary_01_bin_start, _binary_01_bin_end, _binary_02_bin_start, _binary_02_bin_end, _binary_03_bin_start, _binary_03_bin_end,
				_binary_04_bin_start, _binary_04_bin_end;

static esp_err_t blynk_tone_play(uint8_t cmd)
{
	int res = 0;

	media_hal_audio_info_t bin_info = {0};

	bin_info.sample_rate = 16000;
	bin_info.channels = 1;
	bin_info.bits_per_sample = 16;

	switch(cmd)
	{
		case POWEROFF:
			res = tone_play_custom(&_binary_00_bin_start, &_binary_00_bin_end, &bin_info);
			break;
		case POWERON:
			res = tone_play_custom(&_binary_01_bin_start, &_binary_01_bin_end, &bin_info);
			break;
		case STEP1:
			res = tone_play_custom(&_binary_02_bin_start, &_binary_02_bin_end, &bin_info);
			break;
		case STEP2:
			res = tone_play_custom(&_binary_03_bin_start, &_binary_03_bin_end, &bin_info);
			break;
		case STEP3:
			res = tone_play_custom(&_binary_04_bin_start, &_binary_04_bin_end, &bin_info);
			break;
		
	}

	if(res != ESP_OK)
	{
		ESP_LOGE(TAG, "Error tone play.");
	}
	return res;
}

/* Blynk client state handler */
static void state_handler(blynk_client_t *c, const blynk_state_evt_t *ev, void *data) {
	ESP_LOGI(TAG, "state: %d\n", ev->state);
}

/* Virtual write handler */
static void vw_handler(blynk_client_t *c, uint16_t id, const char *cmd, int argc, char **argv, void *data)
{
	if (argc > 1)
	{
		switch(atoi(argv[0]))
		{
			case BLY_VENT_POWER:
				if(!atoi(argv[1]))
				{
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 0);
					vent_step = 0;
				}
				vent_power_on = atoi(argv[1]);
				blynk_tone_play(atoi(argv[1]));
				break;
			case BLY_VENT_STEP1:
				if(atoi(argv[1]))
				{
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 0);
					blynk_tone_play(STEP1);
					vent_step = BLY_VENT_STEP1;
				}
				break;
			case BLY_VENT_STEP2:
				if(atoi(argv[1]))
				{
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 0);
					blynk_tone_play(STEP2);
					vent_step = BLY_VENT_STEP2;
				}
				break;
			case BLY_VENT_STEP3:
				if(atoi(argv[1]))
				{
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 0);
					blynk_tone_play(STEP3);
					vent_step = BLY_VENT_STEP3;
				}
				break;
		}
	}
}

uint8_t prev_pw_value = 0;
uint8_t prev_step_value = 0;

/* Virtual read handler */
static void vr_handler(blynk_client_t *c, uint16_t id, const char *cmd, int argc, char **argv, void *data)
{
	if (!argc) {
		return;
	}

	int pin = atoi(argv[0]);

	if(pin == BLY_VENT_POWER)
	{
		uint8_t pw_value = (uint8_t)vent_power_on;
		uint8_t step_value = vent_step;

		if(prev_pw_value != pw_value)
		{
			/* Respond with `virtual write' command */
			blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_POWER, pw_value);
			if(!pw_value)
			{
				blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 0);
				blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 0);
				blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 0);
			}
		}
		prev_pw_value = pw_value;

		if(prev_step_value != step_value)
		{
			switch(step_value)
			{
				case BLY_VENT_STEP1:
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 1);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 0);
					break;
				case BLY_VENT_STEP2:
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 1);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 0);
					break;
				case BLY_VENT_STEP3:
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 1);
					break;
				default:
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 0);
					
			}
		}
		prev_step_value = step_value;
		
	}
}

static void esp_blynk_apps(void)
{
	blynk_err_t ret;

	blynk_client_t *client = malloc(sizeof(blynk_client_t));
	blynk_init(client);

	blynk_options_t opt = {
		.token = CONFIG_BLYNK_TOKEN,
		.server = CONFIG_BLYNK_SERVER,
		/* Use default timeouts */
	};

	blynk_set_options(client, &opt);

	/* Subscribe to state changes and errors */
	blynk_set_state_handler(client, state_handler, NULL);

	/* blynk_set_handler sets hardware (BLYNK_CMD_HARDWARE) command handler */
	blynk_set_handler(client, "vw", vw_handler, NULL);
	blynk_set_handler(client, "vr", vr_handler, NULL);

	/* Start Blynk client task */
	ret = blynk_start(client);
	ESP_LOGI(TAG, "blynk_start ret[%d]", ret);
}
#endif


static EventGroupHandle_t cm_event_group;
const int CONNECTED_BIT = BIT0;
const int PROV_DONE_BIT = BIT1;
#if defined(FACTORY_RESET)
uint8_t reset_counter = 0;
#endif

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    conn_mgr_prov_event_handler(ctx, event);

    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        esp_wifi_set_storage(WIFI_STORAGE_FLASH);
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        // We already have print in SYSTEM_EVENT_STA_GOT_IP
#if defined(FACTORY_RESET)
		reset_counter = 0;
#endif
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        app_wifi_stop_timeout_timer();
        printf("%s: Connected with IP Address: %s\n", TAG, ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        xEventGroupSetBits(cm_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
    case SYSTEM_EVENT_STA_LOST_IP:
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
        app_wifi_stop_timeout_timer();
#if defined(FACTORY_RESET)
		printf("%s: Disconnected. Event: %d. Connecting to the AP again Try %d\n", TAG, event->event_id, reset_counter++);
		if(reset_counter < 20)
			esp_wifi_connect();
		else
		{
			reset_counter = 0;

			va_led_set(LED_OFF);
			va_nvs_flash_erase();
			va_reset();
			esp_restart();
		}
#else
        printf("%s: Disconnected. Event: %d. Connecting to the AP again\n", TAG, event->event_id);
        esp_wifi_connect();
#endif
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void wifi_init_sta()
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_start() );
#ifdef CONFIG_PM_ENABLE
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
#else
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
#endif
}

#define MEDIA_HAL_DEFAULT()     \
    {   \
        .op_mode    = MEDIA_HAL_MODE_SLAVE,              \
        .adc_input  = MEDIA_HAL_ADC_INPUT_LINE1,         \
        .dac_output = MEDIA_HAL_DAC_OUTPUT_ALL,          \
        .codec_mode = MEDIA_HAL_CODEC_MODE_BOTH,         \
        .bit_length = MEDIA_HAL_BIT_LENGTH_16BITS,       \
        .format     = MEDIA_HAL_I2S_NORMAL,              \
        .port_num = 0,                          \
    };

void app_prov_done_cb()
{
    xEventGroupSetBits(cm_event_group, PROV_DONE_BIT);
}

void app_main()
{
    ESP_LOGI(TAG, "==== Voice Assistant SDK version: %s ====", va_get_sdk_version());

    /* This will never be freed */
    alexa_config_t *va_cfg = va_mem_alloc(sizeof(alexa_config_t), VA_MEM_EXTERNAL);

    if (!va_cfg) {
        ESP_LOGE(TAG, "Failed to alloc voice assistant config");
        abort();
    }
    va_cfg->product_id = CONFIG_ALEXA_PRODUCT_ID;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    va_board_init();
    static media_hal_config_t media_hal_conf = MEDIA_HAL_DEFAULT();
    media_hal_init(&media_hal_conf);

#if defined(CTC_REV01)
	ctc_led_init();
#else
	va_board_button_init();
#endif
    va_board_led_init();

#if defined(CTC_CS48L32)
	cs_reset();

	cs_spi_init();

	cs_spi_register_write(0, CS48L32_CONFIG_REG, CS48L32_REG_TYPE_CONFIG);
	vTaskDelay(100 / portTICK_PERIOD_MS);

	cs_spi_register_write(0, (CS48L32_DSP_PROGRAM_REG - 2), CS48L32_REG_TYPE_DSP_PROGRAM);
	vTaskDelay(500 / portTICK_PERIOD_MS);

	cs_spi_firmware_write();
	vTaskDelay(100 / portTICK_PERIOD_MS);

	cs_spi_register_write((CS48L32_DSP_PROGRAM_REG - 2), CS48L32_DSP_PROGRAM_REG, CS48L32_REG_TYPE_DSP_PROGRAM);
	vTaskDelay(100 / portTICK_PERIOD_MS);

#if defined(CTC_CS48L32_TUNE_1ST)
	cs_spi_register_write(0, CS48L32_TUNE_1ST_REG, CS48L32_REG_TYPE_TUNE_1ST);
	vTaskDelay(500 / portTICK_PERIOD_MS);
#endif

	cs_spi_register_write(0, CS48L32_DSP_START_REG, CS48L32_REG_TYPE_DSP_START);
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	esp_cs_irq_intr_init();

	ak_reset();
	
	ESP_LOGE(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
#endif

    scli_init();
    va_diag_register_cli();
    wifi_register_cli();
    app_wifi_reset_to_prov_init();
    app_auth_register_cli();
    cm_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    printf("\r");       // To remove a garbage print ">>"
    alexa_auth_delegate_init(NULL, NULL);
    bool provisioned = false;
    if (conn_mgr_prov_is_provisioned(&provisioned) != ESP_OK) {
        ESP_LOGE(TAG, "Error getting device provisioning state");
        abort();
    }
    if (app_wifi_get_reset_to_prov() > 0) {
        app_wifi_start_timeout_timer();
        provisioned = false;
        app_wifi_unset_reset_to_prov();
        esp_wifi_set_storage(WIFI_STORAGE_RAM);
    }

    char service_name[20];
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(service_name, sizeof(service_name), "%s%02X%02X", SOFTAP_SSID_PREFIX, mac[4], mac[5]);

    if (!provisioned) {
        va_led_set(LED_RESET);

#if defined (EMO_ROBOT)
		/* This is a blocking call */
		va_dsp_init(speech_recognizer_recognize, speech_recognizer_record);

#if defined(EMO_ROBOT_16khz)
		i2s_set_sample_rates(I2S_NUM_0, 16000);
		vTaskDelay(100 / portTICK_PERIOD_MS);
#endif
		
#if defined(CTC_CS48L32_FLL_ASP1_BCLK)
		ESP_LOGE(TAG, "BCLK changed.");
		cs_spi_register_write(0, CS48L32_FLL_CHANGE_REG, CS48L32_REG_TYPE_FLL_CHANGE);
		vTaskDelay(100 / portTICK_PERIOD_MS);
#endif
		gpio_set_level(TRI_LED, 1);
		gpio_set_level(RES_LED, 1);
#endif

        printf("%s: Starting provisioning\n", TAG);
        conn_mgr_prov_t prov_type = conn_mgr_prov_mode_ble;
        prov_type.event_cb = alexa_conn_mgr_prov_cb;
        prov_type.cb_user_data = (void *)va_cfg;
        int security = 1;
        const char *pop = "abcd1234";
        const char *service_key = "";
        conn_mgr_prov_start_provisioning(prov_type, security, pop, service_name, service_key);
        printf("\tproof of possession (pop): %s\n", pop);
    } else {
        va_led_set(VA_CAN_START);
        ESP_LOGI(TAG, "Already provisioned, starting station");
        conn_mgr_prov_mem_release();        // This is useful in case of BLE provisioning
        app_prov_done_cb();
        wifi_init_sta();
    }

#if !defined (EMO_ROBOT)

    xEventGroupWaitBits(cm_event_group, CONNECTED_BIT | PROV_DONE_BIT, false, true, portMAX_DELAY);

    if (!provisioned) {
        va_led_set(VA_CAN_START);
    }

#ifdef CONFIG_ALEXA_ENABLE_EQUALIZER
    alexa_equalizer_init();
#endif

    ret = alexa_local_config_start(va_cfg, service_name);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start local SSDP instance. Some features might not work.");
    }

#ifdef ALEXA_BT
    alexa_bt_a2dp_sink_init();
#endif
    ret = alexa_init(va_cfg);

    if (ret != ESP_OK) {
        while(1) vTaskDelay(2);
    }
    /* This is a blocking call */
    va_dsp_init(speech_recognizer_recognize, speech_recognizer_record);

#if defined(CTC_CS48L32_FLL_ASP1_BCLK)
	ESP_LOGE(TAG, "BCLK changed.");
	cs_spi_register_write(0, CS48L32_FLL_CHANGE_REG, CS48L32_REG_TYPE_FLL_CHANGE);
	vTaskDelay(100 / portTICK_PERIOD_MS);
#endif

#if defined(CTC_CS48L32_CHECK_REG)
	cs_spi_register_write(0, CS48L32_CHECK_REG, CS48L32_REG_TYPE_CHECK_REG);
	vTaskDelay(100 / portTICK_PERIOD_MS);
#endif

#if !defined(CTC_CS48L32_SENSORY)
	cs_spi_deinit();
#endif

#if defined(BLYNK_APPS)
	esp_blynk_apps();
#endif

#ifdef CONFIG_ALEXA_ENABLE_OTA
    /* Doing OTA init after full alexa boot-up. */
    app_ota_init();
#endif
    /* This is only supported with minimum flash size of 8MB. */
    alexa_tone_enable_larger_tones();

#endif

#ifdef CONFIG_PM_ENABLE
    rtc_cpu_freq_t max_freq;
    rtc_clk_cpu_freq_from_mhz(CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ, &max_freq);
    esp_pm_config_esp32_t pm_config = {
            .max_cpu_freq = max_freq,
            .min_cpu_freq = RTC_CPU_FREQ_XTAL,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
            .light_sleep_enable = true
#endif
    };
    ESP_ERROR_CHECK( esp_pm_configure(&pm_config));
    gpio_wakeup_enable(GPIO_NUM_36, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_pm_dump_locks(stdout);
#endif
    return;
}
