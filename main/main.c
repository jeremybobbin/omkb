#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_event.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#include "esp_hidd_prf_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hid_dev.h"
#include "nvs_flash.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c_master.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "sdkconfig.h"

#define LENGTH(x)  ((int)(sizeof (x) / sizeof *(x)))
#define MIN(a, b)  ((a) > (b) ? (b) : (a))
#define MAX(a, b)  ((a) < (b) ? (b) : (a))

#define LCD_PIXEL_CLOCK_HZ (400 * 1000)
#define PIN_NUM_SDA                  8
#define PIN_NUM_SCL                  9
#define PIN_NUM_RST                 -1
#define LCD_H_RES                  128
#define LCD_V_RES                   32
#define LCD_CMD_BITS                 8
#define LCD_PARAM_BITS               8

static const char *TAG = "lask5";
static int disconnects = 0;
static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static char *str = "----\0";
static lv_obj_t *label = NULL;

static adc_channel_t channels[4] = {
	ADC_CHANNEL_0,
	ADC_CHANNEL_1,
	ADC_CHANNEL_2,
	ADC_CHANNEL_3
};

static uint8_t hidd_service_uuid128[] = {
	0xfb, 0x34, 0x9b, 0x5f,
	0x80, 0x00, 0x00, 0x80,
	0x00, 0x10, 0x00, 0x00,
	0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
	.set_scan_rsp = false,
	.include_name = true,
	.include_txpower = true,
	.min_interval = 0x0006,	//slave connection min interval, Time = min_interval * 1.25 msec
	.max_interval = 0x0010,	//slave connection max interval, Time = max_interval * 1.25 msec
	.appearance = 0x03c0,	//HID Generic,
	.manufacturer_len = 0,
	.p_manufacturer_data = NULL,
	.service_data_len = 0,
	.p_service_data = NULL,
	.service_uuid_len = sizeof(hidd_service_uuid128),
	.p_service_uuid = hidd_service_uuid128,
	.flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
	.adv_type             =  ADV_TYPE_IND,
	.own_addr_type        =  BLE_ADDR_TYPE_PUBLIC,
	.channel_map          =  ADV_CHNL_ALL,
	.adv_filter_policy    =  ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
	//.peer_addr_type       =  0,
	.adv_int_min          =  0x20,
	.adv_int_max          =  0x30,
	//.peer_addr            =  { 0x80, 0x86, 0xF2, 0xD9, 0x8C, 0xAC },
};

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param) {
	switch (event) {
	case ESP_HIDD_EVENT_REG_FINISH:
		if (param->init_finish.state == ESP_HIDD_INIT_OK) {
			esp_ble_gap_set_device_name("LASK-5 Keyboard");
			esp_ble_gap_config_adv_data(&hidd_adv_data);

		}
		break;
	case ESP_HIDD_EVENT_BLE_CONNECT:
		ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
		hid_conn_id = param->connect.conn_id;
		break;
	case ESP_HIDD_EVENT_BLE_DISCONNECT:
		sec_conn = false;
		ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT %d", disconnects++);
		esp_ble_gap_start_advertising(&hidd_adv_params);
		break;
	case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT:
		ESP_LOGI(TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
		ESP_LOG_BUFFER_HEX(TAG, param->vendor_write.data, param->vendor_write.length);
		break;
	case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT:
		ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
		ESP_LOG_BUFFER_HEX(TAG, param->led_write.data, param->led_write.length);
		break;
	//case ESP_BAT_EVENT_REG:
	//case ESP_HIDD_EVENT_DEINIT_FINISH:
	default:
		break;
	}
	return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
	switch (event) {
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		esp_ble_gap_start_advertising(&hidd_adv_params);
		sec_conn = false;
		break;
	case ESP_GAP_BLE_SEC_REQ_EVT:
		for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
			ESP_LOGD(TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
		}
		esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
		sec_conn = false;
		break;
	case ESP_GAP_BLE_AUTH_CMPL_EVT:
		esp_bd_addr_t bd_addr;
		memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
		ESP_LOGI(TAG, "remote BD_ADDR: %08x%04x",
			(bd_addr[0] << 24) + (bd_addr[1] << 16) +
			(bd_addr[2] << 8) + bd_addr[3],
			(bd_addr[4] << 8) + bd_addr[5]
		);
		ESP_LOGI(TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
		ESP_LOGI(TAG, "pair status = %s", param->ble_security.auth_cmpl. success ? "success" : "fail");
		if (!param->ble_security.auth_cmpl.success) {
			ESP_LOGE(TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
		}
		vTaskDelay(pdMS_TO_TICKS(50));
		sec_conn = true;
		break;
	default:
		break;
	}
}

adc_oneshot_unit_handle_t adc;

typedef struct {
	int ch;
	int hid;
} keymap;

const keymap qwerty[] = {
	{ '0',  HID_KEY_0,                },
	{ 'P',  HID_KEY_P,                },
	{ ';',  HID_KEY_SEMI_COLON,       },
	{ '/',  HID_KEY_FWD_SLASH,        },
	{ '9',  HID_KEY_9,                },
	{ 'O',  HID_KEY_O,                },
	{ 'L',  HID_KEY_L,                },
	{ '.',  HID_KEY_DOT,              },
	{ '8',  HID_KEY_8,                },
	{ 'I',  HID_KEY_I,                },
	{ 'K',  HID_KEY_K,                },
	{ ',',  HID_KEY_COMMA,            },
	{ '7',  HID_KEY_7,                },
	{ 'U',  HID_KEY_U,                },
	{ 'J',  HID_KEY_J,                },
	{ 'M',  HID_KEY_M,                },
};

void hid_task(void *pvParameters)
{
	int i, n;
	uint8_t c;
	for (;; vTaskDelay(pdMS_TO_TICKS(100))) {
		if (!sec_conn) {
			continue;
		}
		for (i = 0; i < LENGTH(channels); i++) {
			ESP_ERROR_CHECK(adc_oneshot_read(adc, channels[i], &n));
			ESP_LOGI(TAG, "Channel[%d] Raw Data: %d", i, n);

			if (n < 2150 || n > 3000) {
				continue;
			}
			n = (n-2150)/75;
			n = MIN(n, 4);
			c = qwerty[(i*4)+n].hid;
			str[i] = qwerty[(i*4)+n].ch;
			ESP_LOGI(TAG, "sending key: %d\n", c);
			if (esp_hidd_send_keyboard_value(hid_conn_id, 0, &c, 1)) {
				sec_conn = false;
				ESP_LOGI(TAG, "failed sending up");
				continue;
			}
			if (esp_hidd_send_keyboard_value(hid_conn_id, 0, &c, 0)) {
				sec_conn = false;
				ESP_LOGI(TAG, "failed down");
				continue;
			}
		}
		lv_label_set_text(label, str);
	}
	ESP_ERROR_CHECK(adc_oneshot_del_unit(adc));

}

void app_main(void)
{
	esp_err_t ret;
	int i;
	uint8_t key_size, init_key, rsp_key;

	// Initialize NVS.
	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	ESP_ERROR_CHECK(ret);
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));


	const adc_oneshot_chan_cfg_t chan = {
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};

	const adc_oneshot_unit_init_cfg_t unit = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &adc));
	for (i = 0; i < LENGTH(channels); i++) {
		ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, channels[i], &chan));
	}

	ESP_LOGI(TAG, "Initialize I2C bus");
	i2c_master_bus_handle_t i2c_bus = NULL;
	i2c_master_bus_config_t bus_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.i2c_port = I2C_NUM_0,
		.sda_io_num = PIN_NUM_SDA,
		.scl_io_num = PIN_NUM_SCL,
		.flags.enable_internal_pullup = true,
	};
	ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

	ESP_LOGI(TAG, "Install panel IO");
	esp_lcd_panel_io_handle_t io_handle = NULL;
	esp_lcd_panel_io_i2c_config_t io_config = {
		.dev_addr = 0x3C,
		.scl_speed_hz = LCD_PIXEL_CLOCK_HZ,
		.control_phase_bytes = 1,
		.lcd_cmd_bits = LCD_CMD_BITS,
		.lcd_param_bits = LCD_PARAM_BITS,
		.dc_bit_offset = 6,
	};
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

	ESP_LOGI(TAG, "Install SSD1306 panel driver");
	esp_lcd_panel_handle_t panel_handle = NULL;
	esp_lcd_panel_ssd1306_config_t ssd1306_config = {
		.height = LCD_V_RES,
	};
	esp_lcd_panel_dev_config_t panel_config = {
		.bits_per_pixel = 1,
		.reset_gpio_num = PIN_NUM_RST,
		.vendor_config = &ssd1306_config,
	};

	ESP_LOGI(TAG, "new panel");
	ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

	ESP_LOGI(TAG, "reset panel");
	ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));

	ESP_LOGI(TAG, "init panel");
	ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

	ESP_LOGI(TAG, "cycle panel");
	ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

	ESP_LOGI(TAG, "Initialize LVGL");
	const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
	lvgl_port_init(&lvgl_cfg);

	const lvgl_port_display_cfg_t disp_cfg = {
		.io_handle = io_handle,
		.panel_handle = panel_handle,
		.buffer_size = LCD_H_RES * LCD_V_RES,
		.double_buffer = true,
		.hres = LCD_H_RES,
		.vres = LCD_V_RES,
		.monochrome = true,
		.rotation = {
			.swap_xy = false,
			.mirror_x = false,
			.mirror_y = false,
		}
	};
	lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);

	/* Rotation of the screen */
	lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

	ESP_LOGI(TAG, "Display LVGL Scroll Text");
	// Lock the mutex due to the LVGL APIs are not thread-safe
	if (!lvgl_port_lock(0)) {
		ESP_LOGI(TAG, "failed to lock LVGL port");
		return;
	}

	lv_obj_t *scr = lv_disp_get_scr_act(disp);
	label = lv_label_create(scr);
	lv_obj_set_width(label, disp->driver->hor_res);
	lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
	lvgl_port_unlock();

	esp_bt_controller_config_t bt = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	if ((ret = esp_bt_controller_init(&bt))) {
		ESP_LOGE(TAG, "%s initialize controller failed",
			 __func__);
		return;
	}

	if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE))) {
		ESP_LOGE(TAG, "enable controller failed");
		return;
	}

	if ((ret = esp_bluedroid_init())) {
		ESP_LOGE(TAG, "init bluedroid failed");
		return;
	}

	if ((ret = esp_bluedroid_enable())) {
		ESP_LOGE(TAG, "init bluedroid failed");
		return;
	}

	if ((ret = esp_hidd_profile_init()) != ESP_OK) {
		ESP_LOGE(TAG, "init bluedroid failed");
	}

	esp_ble_gap_register_callback(gap_event_handler);
	esp_hidd_register_callbacks(hidd_event_callback);

	esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
	esp_ble_io_cap_t   iocap    = ESP_IO_CAP_NONE;
	init_key                    = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	rsp_key                     = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	key_size                    = 16;
	esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
	esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE,      &iocap,    sizeof(iocap));
	esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE,    &key_size, sizeof(key_size));
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY,    &init_key, sizeof(init_key));
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY,     &rsp_key,  sizeof(rsp_key));

	xTaskCreate(&hid_task, "hid_task", 2048<<3, NULL, 5, NULL);

}
