#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "driver/i2c_master.h"
#include "esp_adc/adc_continuous.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_event.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"

#include "hid.h"

#define LENGTH(x)  ((int)(sizeof (x) / sizeof *(x)))
#define MIN(a, b)  ((a) > (b) ? (b) : (a))
#define MAX(a, b)  ((a) < (b) ? (b) : (a))

#define W                          128
#define H                           32

typedef struct {
	int ch;
	uint8_t hid;
} Key;

// lask
static const char *TAG = "lask5";
static bool left = 1;

// interprocess-communication
static QueueHandle_t KeyboardQueue, MouseQueue, DisplayQueue, NetworkQueue;

// display
static esp_lcd_panel_handle_t panel = NULL;

// bluetooth
static volatile int disconnects = 0;
static volatile uint16_t gatts_interface = ESP_GATT_IF_NONE;
static volatile uint16_t hid_conn_id = 0;
static volatile bool sec_conn = false;

static uint8_t service_id[] = {
	0xfb, 0x34, 0x9b, 0x5f,
	0x80, 0x00, 0x00, 0x80,
	0x00, 0x10, 0x00, 0x00,
	0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t advert = {
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
	.service_uuid_len = sizeof(service_id),
	.p_service_uuid = service_id,
	.flag = 0x6,
};

static esp_ble_adv_params_t advert_config = {
	.adv_type             =  ADV_TYPE_IND,
	.own_addr_type        =  BLE_ADDR_TYPE_PUBLIC,
	.channel_map          =  ADV_CHNL_ALL,
	.adv_filter_policy    =  ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
	.adv_int_min          =  0x20,
	.adv_int_max          =  0x30,
};

// adc
adc_continuous_handle_t adc = NULL;

static adc_channel_t channels[] = {
	ADC_CHANNEL_0,
	ADC_CHANNEL_1,
	ADC_CHANNEL_2,
	ADC_CHANNEL_3,
	ADC_CHANNEL_4,
	ADC_CHANNEL_5,
};

const Key qwerty[] = {
	{ '0',  HID_KEY_0,                },
	{ 'p',  HID_KEY_P,                },
	{ ';',  HID_KEY_SEMI_COLON,       },
	{ '/',  HID_KEY_FWD_SLASH,        },
	{ '9',  HID_KEY_9,                },
	{ 'o',  HID_KEY_O,                },
	{ 'l',  HID_KEY_L,                },
	{ '.',  HID_KEY_DOT,              },
	{ '8',  HID_KEY_8,                },
	{ 'i',  HID_KEY_I,                },
	{ 'k',  HID_KEY_K,                },
	{ ',',  HID_KEY_COMMA,            },
	{ '7',  HID_KEY_7,                },
	{ 'u',  HID_KEY_U,                },
	{ 'j',  HID_KEY_J,                },
	{ 'm',  HID_KEY_M,                },
	{ '6',  HID_KEY_6,                },
	{ 'y',  HID_KEY_Y,                },
	{ 'h',  HID_KEY_H,                },
	{ 'n',  HID_KEY_N,                },
	{ '5',  HID_KEY_5,                },
	{ 't',  HID_KEY_T,                },
	{ 'g',  HID_KEY_G,                },
	{ 'b',  HID_KEY_B,                },
	{ '4',  HID_KEY_4,                },
	{ 'r',  HID_KEY_R,                },
	{ 'f',  HID_KEY_F,                },
	{ 'v',  HID_KEY_V,                },
	{ '3',  HID_KEY_3,                },
	{ 'e',  HID_KEY_E,                },
	{ 'd',  HID_KEY_D,                },
	{ 'c',  HID_KEY_C,                },
	{ '2',  HID_KEY_2,                },
	{ 'w',  HID_KEY_W,                },
	{ 's',  HID_KEY_S,                },
	{ 'x',  HID_KEY_X,                },
	{ '1',  HID_KEY_1,                },
	{ 'q',  HID_KEY_Q,                },
	{ 'a',  HID_KEY_A,                },
	{ 'z',  HID_KEY_Z,                },
	{ '`',  HID_KEY_GRV_ACCENT,       },
	{ '\t', HID_KEY_TAB,              },
};

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	switch (event) {
	case ESP_GATTS_REG_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_REG_EVT %d", param->reg.app_id);
		if (param->reg.status != ESP_GATT_OK) {
			ESP_LOGI(TAG, "app registration failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
			break;
		}
		gatts_interface = gatts_if;
		esp_ble_gap_config_local_icon(ESP_BLE_APPEARANCE_GENERIC_HID);

		switch (param->reg.app_id) {
		case HIDD_APP_ID:
			hidd_le_env.gatt_if = gatts_if;
			if (param->reg.status == 0) {
				esp_ble_gap_set_device_name("LASK-5 Keyboard");
				esp_ble_gap_config_adv_data(&advert);

			}
			/* Here should added the battery service first, because the hid service should include the battery service.
			   After finish to added the battery service then can added the hid service. */
			esp_ble_gatts_create_attr_tab(bas_att_db, gatts_if, BAS_IDX_NB, 0);
			break;
		case BATTRAY_APP_ID:
		}
		break;
	case ESP_GATTS_CONF_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT");
		break;
	case ESP_GATTS_CREATE_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_CREATE_EVT");
		break;
	case ESP_GATTS_CONNECT_EVT:
		hidd_clcb_alloc(param->connect.conn_id, param->connect.remote_bda);
		esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_NO_MITM);
		hid_conn_id = param->connect.conn_id;
		ESP_LOGI(TAG, "HID connection establish, conn_id = %x", param->connect.conn_id);
		break;
	case ESP_GATTS_DISCONNECT_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT %d", disconnects++);
		sec_conn = false;
		esp_ble_gap_start_advertising(&advert_config);
		hidd_clcb_dealloc(param->disconnect.conn_id);
		break;
	case ESP_GATTS_CLOSE_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_CLOSE_EVT");
		break;
	case ESP_GATTS_WRITE_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_WRITE_EVT %d", param->write.len);
		if (param->write.handle == hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_LED_OUT_VAL]) {
			ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);
		} else if (param->write.handle == hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_VENDOR_OUT_VAL]) {
			ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);
		}
		break;
	case ESP_GATTS_CREAT_ATTR_TAB_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_CREAT_ATTR_TAB_EVT");
		if (param->add_attr_tab.status != ESP_GATT_OK) {
			ESP_LOGE(TAG, "ESP_GATTS_CREAT_ATTR_TAB_EVT returned %d", param->add_attr_tab.status);
		}
		switch (param->add_attr_tab.num_handle) {
		case BAS_IDX_NB:
			if (param->add_attr_tab.status != ESP_GATT_OK) {
				break;
			}
			incl_svc.start_hdl = param->add_attr_tab.handles[BAS_IDX_SVC];
			incl_svc.end_hdl = incl_svc.start_hdl + BAS_IDX_NB - 1;
			ESP_LOGI(TAG, "start added the hid service to the stack database. incl_handle = %d", incl_svc.start_hdl);
			esp_ble_gatts_create_attr_tab(hidd_le_gatt_db, gatts_if, HIDD_LE_IDX_NB, 0);
		default:
			//ESP_LOGE(TAG, "ESP_GATTS_CREAT_ATTR_TAB_EVT unknown handle %d", param->add_attr_tab.num_handle);
		}
		if (param->add_attr_tab.num_handle == HIDD_LE_IDX_NB && param->add_attr_tab.status == ESP_GATT_OK) {
			memcpy(hidd_le_env.hidd_inst.att_tbl, param->add_attr_tab.handles, HIDD_LE_IDX_NB * sizeof(uint16_t));
			ESP_LOGI(TAG, "hid svc handle = %x", hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_SVC]);
			hid_add_id_tbl();
			esp_ble_gatts_start_service(hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_SVC]);
		} else {
			esp_ble_gatts_start_service(param->add_attr_tab.handles[0]);
		}
		break;
	case ESP_GATTS_READ_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_READ_EVT");
		break;
	case ESP_GATTS_EXEC_WRITE_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
		break;
	case ESP_GATTS_MTU_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT");
		break;
	case ESP_GATTS_UNREG_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_UNREG_EVT");
		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_ADD_INCL_SRVC_EVT");
		break;
	case ESP_GATTS_ADD_CHAR_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_ADD_CHAR_EVT");
		break;
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_ADD_CHAR_DESCR_EVT");
		break;
	case ESP_GATTS_DELETE_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_DELETE_EVT");
		break;
	case ESP_GATTS_START_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_START_EVT");
		break;
	case ESP_GATTS_STOP_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_STOP_EVT");
		break;
	case ESP_GATTS_OPEN_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_OPEN_EVT");
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_CANCEL_OPEN_EVT");
		break;
	case ESP_GATTS_LISTEN_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_LISTEN_EVT");
		break;
	case ESP_GATTS_CONGEST_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_CONGEST_EVT");
		break;
	case ESP_GATTS_RESPONSE_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_RESPONSE_EVT");
		break;
	case ESP_GATTS_SET_ATTR_VAL_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_SET_ATTR_VAL_EVT");
		break;
	case ESP_GATTS_SEND_SERVICE_CHANGE_EVT:
		ESP_LOGI(TAG, "ESP_GATTS_SEND_SERVICE_CHANGE_EVT");
		break;
	}
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
	switch (event) {
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		ESP_LOGI(TAG, "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT");
		esp_ble_gap_start_advertising(&advert_config);
		//sec_conn = false;
		break;
	case ESP_GAP_BLE_SEC_REQ_EVT:
		ESP_LOGI(TAG, "ESP_GAP_BLE_SEC_REQ_EVT");
		for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
			ESP_LOGD(TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
		}
		esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
		//sec_conn = false;
		break;
	case ESP_GAP_BLE_AUTH_CMPL_EVT:
		ESP_LOGI(TAG, "ESP_GAP_BLE_AUTH_CMPL_EVT");
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
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(50));
		sec_conn = true;
		break;
	default:
		ESP_LOGI(TAG, "ESP_GAP_UNKNOWN_EVT");
		break;
	}
}

void listen_adc(void *pvParameters) {
	int i, j;
	int n;
	uint8_t buf[LENGTH(channels)*SOC_ADC_DIGI_RESULT_BYTES*32];
	uint16_t min[8], max[8];
	const Key *key;
	adc_digi_output_data_t *bp;
	esp_err_t ret;

	uint16_t items[8], prev[8][8], sent[8];
	for (i = 0; i < 8; i++) {
		for (j = 0; j < LENGTH(prev); j++) {
			prev[j][i] = ~0;
		}
		items[i] = 0;
		max[i] = 0;
		min[i] = ~0;
		sent[i] = ~0;
	}

	adc_continuous_handle_cfg_t adc_config = {
		.max_store_buf_size = sizeof(buf)*4,
		.conv_frame_size = sizeof(buf)*1,
	};

	adc_digi_pattern_config_t adc_pattern[LENGTH(channels)];
	for (i = 0; i < LENGTH(channels); i++) {
		adc_pattern[i].atten = ADC_ATTEN_DB_12;
		adc_pattern[i].channel = channels[i] & 0x7;
		adc_pattern[i].unit = ADC_UNIT_1;
		adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
	}

	adc_continuous_config_t dig_cfg = {
		.sample_freq_hz = 20 * 1000,
		.conv_mode = ADC_CONV_SINGLE_UNIT_1,
		.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
		.pattern_num = LENGTH(channels),
		.adc_pattern = adc_pattern,
	};

	if (adc_continuous_new_handle(&adc_config, &adc) != ESP_OK) {
		ESP_LOGI(TAG, "failed to config ADC");
		return;
	}

	if (adc_continuous_config(adc, &dig_cfg) != ESP_OK) {
		ESP_LOGI(TAG, "failed to init ADC");
		return;
	}

	uint64_t conv[LENGTH(channels)];
	for (j = 0;; j++) {
		if (adc_continuous_start(adc) != ESP_OK) {
			ESP_LOGI(TAG, "failed to start ADC");
			return;
		}

		if ((ret = adc_continuous_read(adc, buf, sizeof(buf), (uint32_t*)(&n), 50)) != ESP_OK) {
			adc_continuous_stop(adc);
			continue;
		}

		adc_continuous_stop(adc);

		memset(conv, 0, sizeof(conv));
		memset(items, 0, sizeof(items));
		for (i = 0; i < n; i += SOC_ADC_DIGI_RESULT_BYTES) {
			bp = (void*) &buf[i];
			conv[bp->type2.channel] += bp->type2.data;
			items[bp->type2.channel]++;
		}

		for (i = 0; i < LENGTH(channels); i++) {
			items[i] = conv[i]/items[i];
		}

		xQueueSend(DisplayQueue, &items[0], ~0);

		for (i = 0; i < 6; i++) {
			if (items[i] < min[i]) {
				min[i] = items[i];
			}
			if (items[i] > max[i]) {
				max[i] = items[i];
			}
			if (min[i] >= max[i]) {
				continue;
			}

			n = items[i]-min[i];
			switch (i) {
			case 0: case 1: case 2: case 3:
				items[i] = MIN(6-1, (sqrt(n)*sqrt(max[i]-min[i])*(6))/(max[i]-min[i]));
				break;
			case 4: case 5:
				items[i] = MIN((H)-1, (n*(H))/(max[i]-min[i]));
				break;
			}
		}

		for (i = 0; i < 4; i++) {
			n = items[i];
			/*
			ESP_LOGI(TAG, "maybe sending event: %d\n", n);

			*/
			if (n < 2) {
				sent[i] = ~0;
				continue;
			}
			n = MIN(n-2, 4);
			int match = 1, k = 0;
			for (k = 0; k < LENGTH(prev); k++) {
				if (prev[k][i] != n) {
					match = 0;
				}
			}
			prev[j%LENGTH(prev)][i] = n;
			if (!match)  {
				continue;
			}
			if (sent[i] == n) {
				sent[i] = ~0;
				continue;
			}
			memset(prev, ~0, sizeof(prev));
			key = &qwerty[(i*4)+n+(left?24:0)];
			ESP_LOGI(TAG, "sending event: %c %d\n", key->ch, n);

			xQueueSend(KeyboardQueue, &key, ~0);
		}
	}
	adc_continuous_stop(adc);
	adc_continuous_deinit(adc);

}

void draw(void *pvParameters)
{
	int i, j;
	uint8_t buf[4][128];
	uint32_t n, m;
	uint16_t items[8], min[8], max[8];
	for (i = 0; i < 8; i++) {
		items[i] = 0;
		max[i] = 0;
		min[i] = ~0;
	}
	esp_lcd_panel_swap_xy(panel, false);
	esp_lcd_panel_set_gap(panel, 0, 0);
	memset(buf, 0, sizeof(buf));
	esp_lcd_panel_draw_bitmap(panel, 0, 0, W, H, &buf);
	for (;;) {
		xQueueReceive(DisplayQueue, items, ~0);

		m = 0;
		for (i = 0; i < 6; i++) {
			if (items[i] < min[i]) {
				min[i] = items[i];
			}
			if (items[i] > max[i]) {
				max[i] = items[i];
			}
			if (min[i] >= max[i]) {
				continue;
			}

			n = items[i]-min[i];
			switch (i) {
			case 0: case 1: case 2: case 3:
				items[i] = MIN((W/8)-1, (sqrt(n)*sqrt(max[i]-min[i])*(W/8))/(max[i]-min[i]));
				break;
			case 4: case 5:
				items[i] = MIN((H)-1, (n*(H))/(max[i]-min[i]));
				break;
			}
		}

		for (i = 0; i < 4; i++) {
			n = items[i];
			buf[0][n + (i*(W/8)) + (W/4)] |= 1;
		}

		for (i = 3; i >= 0; i--) {
			memset(buf[i], 0, W/4);
		}

		n = items[4];
		m = items[5];

		buf[m/8][n] = 1<<(m%8);

		if (esp_lcd_panel_draw_bitmap(panel, 0, 0, W, H, buf)) {
			break;
		}
		for (i = 3; i >= 0; i--) {
			for (j = W/4; j < (W/4)*3; j++) {
				buf[i][j] <<= 1;
				if (i-1 >= 0) {
					buf[i][j] |= buf[i-1][j]>>7;
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(20));
	}

	esp_lcd_panel_disp_on_off(panel, false);
}

void bluetooth_send(void *pvParameters)
{
	Key *key;
	esp_err_t ret;

	for (;; vTaskDelay(pdMS_TO_TICKS(10))) {
		xQueueReceive(KeyboardQueue, &key, ~0);
		if (!sec_conn) {
			continue;
		}

		if ((ret = esp_hidd_send_keyboard_value(hid_conn_id, 0, &key->hid, 1))) {
			sec_conn = false;
			ESP_LOGI(TAG, "failed sending key %c - %d", key->ch, ret);
			continue;
		}

		if (esp_hidd_send_keyboard_value(hid_conn_id, 0, &key->hid, 0)) {
			sec_conn = false;
			ESP_LOGI(TAG, "failed down");
			continue;
		}
	}
}

void app_main(void)
{
	esp_err_t ret;
	uint8_t key_size, init_key, rsp_key;

	DisplayQueue = xQueueCreate(1, sizeof(uint16_t)*6);
	if (DisplayQueue == NULL) {
		ESP_LOGI(TAG, "failed create input queue");
		return;
	}

	KeyboardQueue = xQueueCreate(1, sizeof(Key*));
	if (KeyboardQueue == NULL) {
		ESP_LOGI(TAG, "failed create input queue");
		return;
	}

	MouseQueue = xQueueCreate(64, sizeof(uint16_t)*6);
	if (MouseQueue == NULL) {
		ESP_LOGI(TAG, "failed create input queue");
		return;
	}

	NetworkQueue = xQueueCreate(64, sizeof(uint16_t)*6);
	if (NetworkQueue == NULL) {
		ESP_LOGI(TAG, "failed create input queue");
		return;
	}

	ESP_LOGI(TAG, "Initialize I2C bus");
	i2c_master_bus_handle_t i2c_bus = NULL;
	i2c_master_bus_config_t bus_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.i2c_port = I2C_NUM_0,
		.sda_io_num = 8,
		.scl_io_num = 9,
		.flags.enable_internal_pullup = true,
	};
	ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

	ESP_LOGI(TAG, "Install panel IO");
	esp_lcd_panel_io_handle_t io_handle = NULL;
	esp_lcd_panel_io_i2c_config_t io_config = {
		.dev_addr = 0x3C,
		.scl_speed_hz = 400 * 1000,
		.control_phase_bytes = 1,
		.lcd_cmd_bits = 32,
		.lcd_param_bits = 8,
		.dc_bit_offset = 6,
	};
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

	ESP_LOGI(TAG, "Install SSD1306 panel driver");
	esp_lcd_panel_ssd1306_config_t ssd1306_config = {
		.height = H,
	};
	esp_lcd_panel_dev_config_t panel_config = {
		.bits_per_pixel = 1,
		.reset_gpio_num = -1,
		.vendor_config = &ssd1306_config,
	};

	ESP_LOGI(TAG, "new panel");
	ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel));

	ESP_LOGI(TAG, "reset panel");
	//ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));

	ESP_LOGI(TAG, "init panel");
	ESP_ERROR_CHECK(esp_lcd_panel_init(panel));

	ESP_LOGI(TAG, "cycle panel");
	ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

	ESP_LOGI(TAG, "Initialize LVGL");

	ESP_LOGI(TAG, "Display LVGL Scroll Text");

	// Initialize NVS.
	ESP_LOGI(TAG, "initalizing NVS");
	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	ESP_ERROR_CHECK(ret);

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));


	ESP_LOGI(TAG, "initalizing bluetooth");
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
		return;
	}

	esp_ble_gap_register_callback(gap_event_handler);

	if ((ret = esp_ble_gatts_register_callback(gatts_event_handler)) != ESP_OK) {
		ESP_LOGE(TAG, "init bluedroid failed");
		return;
	}

	if ((ret = esp_ble_gatts_app_register(BATTRAY_APP_ID)) != ESP_OK) {
		ESP_LOGE(TAG, "init battray app failed");
		return;
	}

	if ((ret = esp_ble_gatts_app_register(HIDD_APP_ID)) != ESP_OK) {
		ESP_LOGE(TAG, "init hidd app failed");
		return;
	}

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

	xTaskCreate(&listen_adc, "listen_adc", 2048<<1, NULL, 5, NULL);
	xTaskCreate(&draw, "draw", 2048<<1, NULL, 5, NULL);
	xTaskCreate(&bluetooth_send, "bluetooth_send", 2048<<1, NULL, 5, NULL);
}
