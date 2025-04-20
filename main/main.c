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

#include "sdkconfig.h"

#define LENGTH(x)  ((int)(sizeof (x) / sizeof *(x)))
#define MIN(a, b)  ((a) > (b) ? (b) : (a))
#define MAX(a, b)  ((a) < (b) ? (b) : (a))

static const char *TAG = "lask5";
static int disconnects = 0;
static uint16_t hid_conn_id = 0;
static bool sec_conn = false;

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
	.peer_addr_type       =  0,
	.adv_int_min          =  0x20,
	.adv_int_max          =  0x30,
	.peer_addr            =  { 0x80, 0x86, 0xF2, 0xD9, 0x8C, 0xAC },
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

const uint8_t qwerty[] = {
	HID_KEY_0,
	HID_KEY_P,
	HID_KEY_SEMI_COLON,
	HID_KEY_FWD_SLASH,
	HID_KEY_9,
	HID_KEY_O,
	HID_KEY_L,
	HID_KEY_DOT,
	HID_KEY_8,
	HID_KEY_I,
	HID_KEY_K,
	HID_KEY_COMMA,
	HID_KEY_7,
	HID_KEY_U,
	HID_KEY_J,
	HID_KEY_M,
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
			c = qwerty[(i*4)+n];
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

	xTaskCreate(&hid_task, "hid_task", 4096<<2, NULL, 5, NULL);

}
