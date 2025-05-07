#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_log.h"
#include "hid.h"

// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN     8

// HID LED output report length
#define HID_LED_OUT_RPT_LEN         1

// HID mouse input report length
#define HID_MOUSE_IN_RPT_LEN        5

// HID consumer control input report length
#define HID_CC_IN_RPT_LEN           2

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

struct prf_char_pres_fmt {
	/// Unit (The Unit is a UUID)
	uint16_t unit;
	/// Description
	uint16_t description;
	/// Format
	uint8_t format;
	/// Exponent
	uint8_t exponent;
	/// Name space
	uint8_t name_space;
};

// HID report mapping table
static hid_report_map_t hid_rpt_map[HID_NUM_REPORTS];
static hid_report_map_t *hid_dev_rpt_tbl;
static uint8_t hid_dev_rpt_tbl_Len;

// HID Report Map characteristic value
// Keyboard report descriptor (using format for Boot interface descriptor)
static const uint8_t hidReportMap[] = {
	0x05, 0x01,		// Usage Page (Generic Desktop)
	0x09, 0x02,		// Usage (Mouse)
	0xA1, 0x01,		// Collection (Application)
	0x85, 0x01,		// Report Id (1)
	0x09, 0x01,		//   Usage (Pointer)
	0xA1, 0x00,		//   Collection (Physical)
	0x05, 0x09,		//     Usage Page (Buttons)
	0x19, 0x01,		//     Usage Minimum (01) - Button 1
	0x29, 0x03,		//     Usage Maximum (03) - Button 3
	0x15, 0x00,		//     Logical Minimum (0)
	0x25, 0x01,		//     Logical Maximum (1)
	0x75, 0x01,		//     Report Size (1)
	0x95, 0x03,		//     Report Count (3)
	0x81, 0x02,		//     Input (Data, Variable, Absolute) - Button states
	0x75, 0x05,		//     Report Size (5)
	0x95, 0x01,		//     Report Count (1)
	0x81, 0x01,		//     Input (Constant) - Padding or Reserved bits
	0x05, 0x01,		//     Usage Page (Generic Desktop)
	0x09, 0x30,		//     Usage (X)
	0x09, 0x31,		//     Usage (Y)
	0x09, 0x38,		//     Usage (Wheel)
	0x15, 0x81,		//     Logical Minimum (-127)
	0x25, 0x7F,		//     Logical Maximum (127)
	0x75, 0x08,		//     Report Size (8)
	0x95, 0x03,		//     Report Count (3)
	0x81, 0x06,		//     Input (Data, Variable, Relative) - X & Y coordinate
	0xC0,			//   End Collection
	0xC0,			// End Collection

	0x05, 0x01,		// Usage Pg (Generic Desktop)
	0x09, 0x06,		// Usage (Keyboard)
	0xA1, 0x01,		// Collection: (Application)
	0x85, 0x02,		// Report Id (2)
	//
	0x05, 0x07,		//   Usage Pg (Key Codes)
	0x19, 0xE0,		//   Usage Min (224)
	0x29, 0xE7,		//   Usage Max (231)
	0x15, 0x00,		//   Log Min (0)
	0x25, 0x01,		//   Log Max (1)
	//
	//   Modifier byte
	0x75, 0x01,		//   Report Size (1)
	0x95, 0x08,		//   Report Count (8)
	0x81, 0x02,		//   Input: (Data, Variable, Absolute)
	//
	//   Reserved byte
	0x95, 0x01,		//   Report Count (1)
	0x75, 0x08,		//   Report Size (8)
	0x81, 0x01,		//   Input: (Constant)
	//
	//   LED report
	0x05, 0x08,		//   Usage Pg (LEDs)
	0x19, 0x01,		//   Usage Min (1)
	0x29, 0x05,		//   Usage Max (5)
	0x95, 0x05,		//   Report Count (5)
	0x75, 0x01,		//   Report Size (1)
	0x91, 0x02,		//   Output: (Data, Variable, Absolute)
	//
	//   LED report padding
	0x95, 0x01,		//   Report Count (1)
	0x75, 0x03,		//   Report Size (3)
	0x91, 0x01,		//   Output: (Constant)
	//
	//   Key arrays (6 bytes)
	0x95, 0x06,		//   Report Count (6)
	0x75, 0x08,		//   Report Size (8)
	0x15, 0x00,		//   Log Min (0)
	0x25, 0x65,		//   Log Max (101)
	0x05, 0x07,		//   Usage Pg (Key Codes)
	0x19, 0x00,		//   Usage Min (0)
	0x29, 0x65,		//   Usage Max (101)
	0x81, 0x00,		//   Input: (Data, Array)
	//
	0xC0,			// End Collection
	//
	0x05, 0x0C,		// Usage Pg (Consumer Devices)
	0x09, 0x01,		// Usage (Consumer Control)
	0xA1, 0x01,		// Collection (Application)
	0x85, 0x03,		// Report Id (3)
	0x09, 0x02,		//   Usage (Numeric Key Pad)
	0xA1, 0x02,		//   Collection (Logical)
	0x05, 0x09,		//     Usage Pg (Button)
	0x19, 0x01,		//     Usage Min (Button 1)
	0x29, 0x0A,		//     Usage Max (Button 10)
	0x15, 0x01,		//     Logical Min (1)
	0x25, 0x0A,		//     Logical Max (10)
	0x75, 0x04,		//     Report Size (4)
	0x95, 0x01,		//     Report Count (1)
	0x81, 0x00,		//     Input (Data, Ary, Abs)
	0xC0,			//   End Collection
	0x05, 0x0C,		//   Usage Pg (Consumer Devices)
	0x09, 0x86,		//   Usage (Channel)
	0x15, 0xFF,		//   Logical Min (-1)
	0x25, 0x01,		//   Logical Max (1)
	0x75, 0x02,		//   Report Size (2)
	0x95, 0x01,		//   Report Count (1)
	0x81, 0x46,		//   Input (Data, Var, Rel, Null)
	0x09, 0xE9,		//   Usage (Volume Up)
	0x09, 0xEA,		//   Usage (Volume Down)
	0x15, 0x00,		//   Logical Min (0)
	0x75, 0x01,		//   Report Size (1)
	0x95, 0x02,		//   Report Count (2)
	0x81, 0x02,		//   Input (Data, Var, Abs)
	0x09, 0xE2,		//   Usage (Mute)
	0x09, 0x30,		//   Usage (Power)
	0x09, 0x83,		//   Usage (Recall Last)
	0x09, 0x81,		//   Usage (Assign Selection)
	0x09, 0xB0,		//   Usage (Play)
	0x09, 0xB1,		//   Usage (Pause)
	0x09, 0xB2,		//   Usage (Record)
	0x09, 0xB3,		//   Usage (Fast Forward)
	0x09, 0xB4,		//   Usage (Rewind)
	0x09, 0xB5,		//   Usage (Scan Next)
	0x09, 0xB6,		//   Usage (Scan Prev)
	0x09, 0xB7,		//   Usage (Stop)
	0x15, 0x01,		//   Logical Min (1)
	0x25, 0x0C,		//   Logical Max (12)
	0x75, 0x04,		//   Report Size (4)
	0x95, 0x01,		//   Report Count (1)
	0x81, 0x00,		//   Input (Data, Ary, Abs)
	0x09, 0x80,		//   Usage (Selection)
	0xA1, 0x02,		//   Collection (Logical)
	0x05, 0x09,		//     Usage Pg (Button)
	0x19, 0x01,		//     Usage Min (Button 1)
	0x29, 0x03,		//     Usage Max (Button 3)
	0x15, 0x01,		//     Logical Min (1)
	0x25, 0x03,		//     Logical Max (3)
	0x75, 0x02,		//     Report Size (2)
	0x81, 0x00,		//     Input (Data, Ary, Abs)
	0xC0,			//   End Collection
	0x81, 0x03,		//   Input (Const, Var, Abs)
	0xC0,			// End Collectionq

	0x06, 0xFF, 0xFF,	// Usage Page(Vendor defined)
	0x09, 0xA5,		// Usage(Vendor Defined)
	0xA1, 0x01,		// Collection(Application)
	0x85, 0x04,		// Report Id (4)
	0x09, 0xA6,		// Usage(Vendor defined)
	0x09, 0xA9,		// Usage(Vendor defined)
	0x75, 0x08,		// Report Size
	0x95, 0x7F,		// Report Count = 127 Btyes
	0x91, 0x02,		// Output(Data, Variable, Absolute)
	0xC0,			// End Collection
};

hidd_le_env_t hidd_le_env;

// HID report map length
uint8_t hidReportMapLen = sizeof(hidReportMap);
uint8_t hidProtocolMode = HID_PROTOCOL_MODE_REPORT;

// HID report mapping table
//static hidRptMap_t  hidRptMap[HID_NUM_REPORTS];

// HID Information characteristic value
static const uint8_t hidInfo[HID_INFORMATION_LEN] = {
	LO_UINT16(0x0111), HI_UINT16(0x0111),	// bcdHID (USB HID version)
	0x00,			// bCountryCode
	HID_KBD_FLAGS		// Flags
};

static uint16_t hidExtReportRefDesc = ESP_GATT_UUID_BATTERY_LEVEL;
static uint8_t hidReportRefMouseIn[HID_REPORT_REF_LEN] = { HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT };
static uint8_t hidReportRefKeyIn[HID_REPORT_REF_LEN] = { HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT };
static uint8_t hidReportRefLedOut[HID_REPORT_REF_LEN] = { HID_RPT_ID_LED_OUT, HID_REPORT_TYPE_OUTPUT };
static uint8_t hidReportRefVendorOut[HID_REPORT_REF_LEN] = { HID_RPT_ID_VENDOR_OUT, HID_REPORT_TYPE_OUTPUT };
static uint8_t hidReportRefFeature[HID_REPORT_REF_LEN] = { HID_RPT_ID_FEATURE, HID_REPORT_TYPE_FEATURE };
static uint8_t hidReportRefCCIn[HID_REPORT_REF_LEN] = { HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT };

static uint16_t hid_le_svc = ATT_SVC_HID;
uint16_t hid_count = 0;
esp_gatts_incl_svc_desc_t incl_svc = { 0 };

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t include_service_uuid = ESP_GATT_UUID_INCLUDE_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t hid_info_char_uuid = ESP_GATT_UUID_HID_INFORMATION;
static const uint16_t hid_report_map_uuid = ESP_GATT_UUID_HID_REPORT_MAP;
static const uint16_t hid_control_point_uuid = ESP_GATT_UUID_HID_CONTROL_POINT;
static const uint16_t hid_report_uuid = ESP_GATT_UUID_HID_REPORT;
static const uint16_t hid_proto_mode_uuid = ESP_GATT_UUID_HID_PROTO_MODE;
static const uint16_t hid_kb_input_uuid = ESP_GATT_UUID_HID_BT_KB_INPUT;
static const uint16_t hid_kb_output_uuid = ESP_GATT_UUID_HID_BT_KB_OUTPUT;
static const uint16_t hid_mouse_input_uuid = ESP_GATT_UUID_HID_BT_MOUSE_INPUT;
static const uint16_t hid_repot_map_ext_desc_uuid = ESP_GATT_UUID_EXT_RPT_REF_DESCR;
static const uint16_t hid_report_ref_descr_uuid = ESP_GATT_UUID_RPT_REF_DESCR;
///the propoty definition
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write_nr = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_write_nr = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

/// battary Service
static const uint16_t battary_svc = ESP_GATT_UUID_BATTERY_SERVICE_SVC;

static const uint16_t bat_lev_uuid = ESP_GATT_UUID_BATTERY_LEVEL;
static const uint8_t bat_lev_ccc[2] = { 0x00, 0x00 };

static const uint16_t char_format_uuid = ESP_GATT_UUID_CHAR_PRESENT_FORMAT;

static uint8_t battary_lev = 50;
/// Full HRS Database Description - Used to add attributes into the database
const esp_gatts_attr_db_t bas_att_db[BAS_IDX_NB] = {
	// Battary Service Declaration
	[BAS_IDX_SVC] = {{ESP_GATT_AUTO_RSP},
			 {ESP_UUID_LEN_16, (uint8_t *) & primary_service_uuid,
			  ESP_GATT_PERM_READ,
			  sizeof(uint16_t), sizeof(battary_svc), (uint8_t *) & battary_svc}
			 },

	// Battary level Characteristic Declaration
	[BAS_IDX_BATT_LVL_CHAR] = {{ESP_GATT_AUTO_RSP},
				   {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid,
				    ESP_GATT_PERM_READ,
				    sizeof(uint8_t), sizeof(uint8_t),
				    (uint8_t *) & char_prop_read_notify}
				   },

	// Battary level Characteristic Value
	[BAS_IDX_BATT_LVL_VAL] = {{ESP_GATT_AUTO_RSP},
				  {ESP_UUID_LEN_16, (uint8_t *) & bat_lev_uuid, ESP_GATT_PERM_READ,
				   sizeof(uint8_t), sizeof(uint8_t), &battary_lev}
				  },

	// Battary level Characteristic - Client Characteristic Configuration Descriptor
	[BAS_IDX_BATT_LVL_NTF_CFG] = {{ESP_GATT_AUTO_RSP},
				      {ESP_UUID_LEN_16, (uint8_t *) & character_client_config_uuid,
				       ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				       sizeof(uint16_t), sizeof(bat_lev_ccc), (uint8_t *) bat_lev_ccc}
				      },

	// Battary level report Characteristic Declaration
	[BAS_IDX_BATT_LVL_PRES_FMT] = {{ESP_GATT_AUTO_RSP},
				       {ESP_UUID_LEN_16, (uint8_t *) & char_format_uuid,
					ESP_GATT_PERM_READ,
					sizeof(struct prf_char_pres_fmt), 0, NULL}},
};

/// Full Hid device Database Description - Used to add attributes into the database
esp_gatts_attr_db_t hidd_le_gatt_db[HIDD_LE_IDX_NB] = {
	// HID Service Declaration
	[HIDD_LE_IDX_SVC] = {{ESP_GATT_AUTO_RSP},
			     {ESP_UUID_LEN_16, (uint8_t *) & primary_service_uuid,
			      ESP_GATT_PERM_READ_ENCRYPTED, sizeof(uint16_t),
			      sizeof(hid_le_svc),
			      (uint8_t *) & hid_le_svc}
			     },

	// HID Service Declaration
	[HIDD_LE_IDX_INCL_SVC] = {{ESP_GATT_AUTO_RSP},
				  {ESP_UUID_LEN_16, (uint8_t *) & include_service_uuid,
				   ESP_GATT_PERM_READ,
				   sizeof(esp_gatts_incl_svc_desc_t),
				   sizeof(esp_gatts_incl_svc_desc_t),
				   (uint8_t *) & incl_svc}
				  },

	// HID Information Characteristic Declaration
	[HIDD_LE_IDX_HID_INFO_CHAR] = {{ESP_GATT_AUTO_RSP},
				       {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid,
					ESP_GATT_PERM_READ,
					sizeof(uint8_t), sizeof(uint8_t),
					(uint8_t *) & char_prop_read}
				       },
	// HID Information Characteristic Value
	[HIDD_LE_IDX_HID_INFO_VAL] = {{ESP_GATT_AUTO_RSP},
				      {ESP_UUID_LEN_16, (uint8_t *) & hid_info_char_uuid,
				       ESP_GATT_PERM_READ,
				       sizeof(hids_hid_info_t), sizeof(hidInfo),
				       (uint8_t *) & hidInfo}
				      },

	// HID Control Point Characteristic Declaration
	[HIDD_LE_IDX_HID_CTNL_PT_CHAR] = {{ESP_GATT_AUTO_RSP},
					  {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid,
					   ESP_GATT_PERM_READ,
					   sizeof(uint8_t), sizeof(uint8_t),
					   (uint8_t *) & char_prop_write_nr}
					  },
	// HID Control Point Characteristic Value
	[HIDD_LE_IDX_HID_CTNL_PT_VAL] = {{ESP_GATT_AUTO_RSP},
					 {ESP_UUID_LEN_16, (uint8_t *) & hid_control_point_uuid,
					  ESP_GATT_PERM_WRITE,
					  sizeof(uint8_t), 0,
					  NULL}
					 },

	// Report Map Characteristic Declaration
	[HIDD_LE_IDX_REPORT_MAP_CHAR] = {{ESP_GATT_AUTO_RSP},
					 {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid,
					  ESP_GATT_PERM_READ,
					  sizeof(uint8_t), sizeof(uint8_t),
					  (uint8_t *) & char_prop_read}
					 },
	// Report Map Characteristic Value
	[HIDD_LE_IDX_REPORT_MAP_VAL] = {{ESP_GATT_AUTO_RSP},
					{ESP_UUID_LEN_16, (uint8_t *) & hid_report_map_uuid,
					 ESP_GATT_PERM_READ,
					 HIDD_LE_REPORT_MAP_MAX_LEN, sizeof(hidReportMap),
					 (uint8_t *) & hidReportMap}
					},

	// Report Map Characteristic - External Report Reference Descriptor
	[HIDD_LE_IDX_REPORT_MAP_EXT_REP_REF] = {{ESP_GATT_AUTO_RSP},
						{ESP_UUID_LEN_16, (uint8_t *) & hid_repot_map_ext_desc_uuid,
						 ESP_GATT_PERM_READ,
						 sizeof(uint16_t), sizeof(uint16_t),
						 (uint8_t *) & hidExtReportRefDesc}
						},

	// Protocol Mode Characteristic Declaration
	[HIDD_LE_IDX_PROTO_MODE_CHAR] = {{ESP_GATT_AUTO_RSP},
					 {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid,
					  ESP_GATT_PERM_READ,
					  sizeof(uint8_t), sizeof(uint8_t),
					  (uint8_t *) & char_prop_read_write}
					 },
	// Protocol Mode Characteristic Value
	[HIDD_LE_IDX_PROTO_MODE_VAL] = {{ESP_GATT_AUTO_RSP},
					{ESP_UUID_LEN_16, (uint8_t *) & hid_proto_mode_uuid,
					 (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
					 sizeof(uint8_t), sizeof(hidProtocolMode),
					 (uint8_t *) & hidProtocolMode}
					},

	[HIDD_LE_IDX_REPORT_MOUSE_IN_CHAR] = {{ESP_GATT_AUTO_RSP},
					      {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid,
					       ESP_GATT_PERM_READ,
					       sizeof(uint8_t), sizeof(uint8_t),
					       (uint8_t *) & char_prop_read_notify}
					      },

	[HIDD_LE_IDX_REPORT_MOUSE_IN_VAL] = {{ESP_GATT_AUTO_RSP},
					     {ESP_UUID_LEN_16, (uint8_t *) & hid_report_uuid,
					      ESP_GATT_PERM_READ,
					      HIDD_LE_REPORT_MAX_LEN, 0,
					      NULL}
					     },

	[HIDD_LE_IDX_REPORT_MOUSE_IN_CCC] = {{ESP_GATT_AUTO_RSP},
					     {ESP_UUID_LEN_16, (uint8_t *) & character_client_config_uuid,
					      (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
					      sizeof(uint16_t), 0,
					      NULL}
					     },

	[HIDD_LE_IDX_REPORT_MOUSE_REP_REF] = {{ESP_GATT_AUTO_RSP},
					      {ESP_UUID_LEN_16, (uint8_t *) & hid_report_ref_descr_uuid,
					       ESP_GATT_PERM_READ,
					       sizeof(hidReportRefMouseIn), sizeof(hidReportRefMouseIn),
					       hidReportRefMouseIn}
					      },
	// Report Characteristic Declaration
	[HIDD_LE_IDX_REPORT_KEY_IN_CHAR] = {{ESP_GATT_AUTO_RSP},
					    {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid,
					     ESP_GATT_PERM_READ,
					     sizeof(uint8_t), sizeof(uint8_t),
					     (uint8_t *) & char_prop_read_notify}
					    },
	// Report Characteristic Value
	[HIDD_LE_IDX_REPORT_KEY_IN_VAL] = {{ESP_GATT_AUTO_RSP},
					   {ESP_UUID_LEN_16, (uint8_t *) & hid_report_uuid,
					    ESP_GATT_PERM_READ,
					    HIDD_LE_REPORT_MAX_LEN, 0,
					    NULL}
					   },
	// Report KEY INPUT Characteristic - Client Characteristic Configuration Descriptor
	[HIDD_LE_IDX_REPORT_KEY_IN_CCC] = {{ESP_GATT_AUTO_RSP},
					   {ESP_UUID_LEN_16, (uint8_t *) & character_client_config_uuid,
					    (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
					    sizeof(uint16_t), 0,
					    NULL}
					   },
	// Report Characteristic - Report Reference Descriptor
	[HIDD_LE_IDX_REPORT_KEY_IN_REP_REF] = {{ESP_GATT_AUTO_RSP},
					       {ESP_UUID_LEN_16, (uint8_t *) & hid_report_ref_descr_uuid,
						ESP_GATT_PERM_READ,
						sizeof(hidReportRefKeyIn), sizeof(hidReportRefKeyIn),
						hidReportRefKeyIn}
					       },

	// Report Characteristic Declaration
	[HIDD_LE_IDX_REPORT_LED_OUT_CHAR] = {{ESP_GATT_AUTO_RSP},
					     {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid,
					      ESP_GATT_PERM_READ,
					      sizeof(uint8_t), sizeof(uint8_t),
					      (uint8_t *) & char_prop_read_write_write_nr}
					     },

	[HIDD_LE_IDX_REPORT_LED_OUT_VAL] = {{ESP_GATT_AUTO_RSP},
					    {ESP_UUID_LEN_16, (uint8_t *) & hid_report_uuid,
					     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
					     HIDD_LE_REPORT_MAX_LEN, 0,
					     NULL}
					    },
	[HIDD_LE_IDX_REPORT_LED_OUT_REP_REF] = {{ESP_GATT_AUTO_RSP},
						{ESP_UUID_LEN_16, (uint8_t *) & hid_report_ref_descr_uuid,
						 ESP_GATT_PERM_READ,
						 sizeof(hidReportRefLedOut), sizeof(hidReportRefLedOut),
						 hidReportRefLedOut}
						},
	// Report Characteristic Declaration
	[HIDD_LE_IDX_REPORT_VENDOR_OUT_CHAR] = {{ESP_GATT_AUTO_RSP},
						{ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid,
						 ESP_GATT_PERM_READ,
						 sizeof(uint8_t), sizeof(uint8_t),
						 (uint8_t *) & char_prop_read_write_notify}
						},
	[HIDD_LE_IDX_REPORT_VENDOR_OUT_VAL] = {{ESP_GATT_AUTO_RSP},
					       {ESP_UUID_LEN_16, (uint8_t *) & hid_report_uuid,
						ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
						HIDD_LE_REPORT_MAX_LEN, 0,
						NULL}
					       },
	[HIDD_LE_IDX_REPORT_VENDOR_OUT_REP_REF] = {{ESP_GATT_AUTO_RSP},
						   {ESP_UUID_LEN_16, (uint8_t *) & hid_report_ref_descr_uuid,
						    ESP_GATT_PERM_READ,
						    sizeof(hidReportRefVendorOut), sizeof(hidReportRefVendorOut),
						    hidReportRefVendorOut}
						   },
	// Report Characteristic Declaration
	[HIDD_LE_IDX_REPORT_CC_IN_CHAR] = {{ESP_GATT_AUTO_RSP},
					   {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid,
					    ESP_GATT_PERM_READ,
					    sizeof(uint8_t), sizeof(uint8_t),
					    (uint8_t *) & char_prop_read_notify}
					   },
	// Report Characteristic Value
	[HIDD_LE_IDX_REPORT_CC_IN_VAL] = {{ESP_GATT_AUTO_RSP},
					  {ESP_UUID_LEN_16, (uint8_t *) & hid_report_uuid,
					   ESP_GATT_PERM_READ,
					   HIDD_LE_REPORT_MAX_LEN, 0,
					   NULL}
					  },
	// Report KEY INPUT Characteristic - Client Characteristic Configuration Descriptor
	[HIDD_LE_IDX_REPORT_CC_IN_CCC] = {{ESP_GATT_AUTO_RSP},
					  {ESP_UUID_LEN_16, (uint8_t *) & character_client_config_uuid,
					   (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED),
					   sizeof(uint16_t), 0,
					   NULL}
					  },
	// Report Characteristic - Report Reference Descriptor
	[HIDD_LE_IDX_REPORT_CC_IN_REP_REF] = {{ESP_GATT_AUTO_RSP},
					      {ESP_UUID_LEN_16, (uint8_t *) & hid_report_ref_descr_uuid,
					       ESP_GATT_PERM_READ,
					       sizeof(hidReportRefCCIn), sizeof(hidReportRefCCIn),
					       hidReportRefCCIn}
					      },

	// Boot Keyboard Input Report Characteristic Declaration
	[HIDD_LE_IDX_BOOT_KB_IN_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP},
						{ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid,
						 ESP_GATT_PERM_READ,
						 sizeof(uint8_t), sizeof(uint8_t),
						 (uint8_t *) & char_prop_read_notify}
						},
	// Boot Keyboard Input Report Characteristic Value
	[HIDD_LE_IDX_BOOT_KB_IN_REPORT_VAL] = {{ESP_GATT_AUTO_RSP},
					       {ESP_UUID_LEN_16, (uint8_t *) & hid_kb_input_uuid,
						ESP_GATT_PERM_READ,
						HIDD_LE_BOOT_REPORT_MAX_LEN, 0,
						NULL}
					       },
	// Boot Keyboard Input Report Characteristic - Client Characteristic Configuration Descriptor
	[HIDD_LE_IDX_BOOT_KB_IN_REPORT_NTF_CFG] = {{ESP_GATT_AUTO_RSP},
						   {ESP_UUID_LEN_16, (uint8_t *) & character_client_config_uuid,
						    (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
						    sizeof(uint16_t), 0,
						    NULL}
						   },

	// Boot Keyboard Output Report Characteristic Declaration
	[HIDD_LE_IDX_BOOT_KB_OUT_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP},
						 {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid,
						  ESP_GATT_PERM_READ,
						  sizeof(uint8_t), sizeof(uint8_t),
						  (uint8_t *) & char_prop_read_write}
						 },
	// Boot Keyboard Output Report Characteristic Value
	[HIDD_LE_IDX_BOOT_KB_OUT_REPORT_VAL] = {{ESP_GATT_AUTO_RSP},
						{ESP_UUID_LEN_16, (uint8_t *) & hid_kb_output_uuid,
						 (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
						 HIDD_LE_BOOT_REPORT_MAX_LEN, 0,
						 NULL}
						},

	// Boot Mouse Input Report Characteristic Declaration
	[HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP},
						   {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid,
						    ESP_GATT_PERM_READ,
						    sizeof(uint8_t), sizeof(uint8_t),
						    (uint8_t *) & char_prop_read_notify}
						   },
	// Boot Mouse Input Report Characteristic Value
	[HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_VAL] = {{ESP_GATT_AUTO_RSP},
						  {ESP_UUID_LEN_16, (uint8_t *) & hid_mouse_input_uuid,
						   ESP_GATT_PERM_READ,
						   HIDD_LE_BOOT_REPORT_MAX_LEN, 0,
						   NULL}
						  },
	// Boot Mouse Input Report Characteristic - Client Characteristic Configuration Descriptor
	[HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG] = {{ESP_GATT_AUTO_RSP},
						      {ESP_UUID_LEN_16, (uint8_t *) & character_client_config_uuid,
						       (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
						       sizeof(uint16_t), 0,
						       NULL}
						      },

	// Report Characteristic Declaration
	[HIDD_LE_IDX_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP},
				     {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid,
				      ESP_GATT_PERM_READ,
				      sizeof(uint8_t), sizeof(uint8_t),
				      (uint8_t *) & char_prop_read_write}
				     },
	// Report Characteristic Value
	[HIDD_LE_IDX_REPORT_VAL] = {{ESP_GATT_AUTO_RSP},
				    {ESP_UUID_LEN_16, (uint8_t *) & hid_report_uuid,
				     ESP_GATT_PERM_READ,
				     HIDD_LE_REPORT_MAX_LEN, 0,
				     NULL}
				    },
	// Report Characteristic - Report Reference Descriptor
	[HIDD_LE_IDX_REPORT_REP_REF] = {{ESP_GATT_AUTO_RSP},
					{ESP_UUID_LEN_16, (uint8_t *) & hid_report_ref_descr_uuid,
					 ESP_GATT_PERM_READ,
					 sizeof(hidReportRefFeature), sizeof(hidReportRefFeature),
					 hidReportRefFeature}
					},
};

void hid_add_id_tbl(void)
{
	// Mouse input report
	hid_rpt_map[0].id = hidReportRefMouseIn[0];
	hid_rpt_map[0].type = hidReportRefMouseIn[1];
	hid_rpt_map[0].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_MOUSE_IN_VAL];
	hid_rpt_map[0].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_MOUSE_IN_VAL];
	hid_rpt_map[0].mode = HID_PROTOCOL_MODE_REPORT;

	// Key input report
	hid_rpt_map[1].id = hidReportRefKeyIn[0];
	hid_rpt_map[1].type = hidReportRefKeyIn[1];
	hid_rpt_map[1].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_KEY_IN_VAL];
	hid_rpt_map[1].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_KEY_IN_CCC];
	hid_rpt_map[1].mode = HID_PROTOCOL_MODE_REPORT;

	// Consumer Control input report
	hid_rpt_map[2].id = hidReportRefCCIn[0];
	hid_rpt_map[2].type = hidReportRefCCIn[1];
	hid_rpt_map[2].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_CC_IN_VAL];
	hid_rpt_map[2].cccdHandle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_CC_IN_CCC];
	hid_rpt_map[2].mode = HID_PROTOCOL_MODE_REPORT;

	// LED output report
	hid_rpt_map[3].id = hidReportRefLedOut[0];
	hid_rpt_map[3].type = hidReportRefLedOut[1];
	hid_rpt_map[3].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_LED_OUT_VAL];
	hid_rpt_map[3].cccdHandle = 0;
	hid_rpt_map[3].mode = HID_PROTOCOL_MODE_REPORT;

	// Boot keyboard input report
	// Use same ID and type as key input report
	hid_rpt_map[4].id = hidReportRefKeyIn[0];
	hid_rpt_map[4].type = hidReportRefKeyIn[1];
	hid_rpt_map[4].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_BOOT_KB_IN_REPORT_VAL];
	hid_rpt_map[4].cccdHandle = 0;
	hid_rpt_map[4].mode = HID_PROTOCOL_MODE_BOOT;

	// Boot keyboard output report
	// Use same ID and type as LED output report
	hid_rpt_map[5].id = hidReportRefLedOut[0];
	hid_rpt_map[5].type = hidReportRefLedOut[1];
	hid_rpt_map[5].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_BOOT_KB_OUT_REPORT_VAL];
	hid_rpt_map[5].cccdHandle = 0;
	hid_rpt_map[5].mode = HID_PROTOCOL_MODE_BOOT;

	// Boot mouse input report
	// Use same ID and type as mouse input report
	hid_rpt_map[6].id = hidReportRefMouseIn[0];
	hid_rpt_map[6].type = hidReportRefMouseIn[1];
	hid_rpt_map[6].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_VAL];
	hid_rpt_map[6].cccdHandle = 0;
	hid_rpt_map[6].mode = HID_PROTOCOL_MODE_BOOT;

	// Feature report
	hid_rpt_map[7].id = hidReportRefFeature[0];
	hid_rpt_map[7].type = hidReportRefFeature[1];
	hid_rpt_map[7].handle = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_VAL];
	hid_rpt_map[7].cccdHandle = 0;
	hid_rpt_map[7].mode = HID_PROTOCOL_MODE_REPORT;

	// Setup report ID map
	hid_dev_register_reports(HID_NUM_REPORTS, hid_rpt_map);
}

esp_err_t esp_hidd_profile_init(void)
{
	if (hidd_le_env.enabled) {
		ESP_LOGE(HID_LE_PRF_TAG, "HID device profile already initialized");
		return ESP_FAIL;
	}
	// Reset the hid device target environment
	memset(&hidd_le_env, 0, sizeof(hidd_le_env_t));
	hidd_le_env.enabled = true;
	return ESP_OK;
}

esp_err_t esp_hidd_profile_deinit(void)
{
	uint16_t hidd_svc_hdl = hidd_le_env.hidd_inst.att_tbl[HIDD_LE_IDX_SVC];
	if (!hidd_le_env.enabled) {
		ESP_LOGE(HID_LE_PRF_TAG, "HID device profile already initialized");
		return ESP_OK;
	}

	if (hidd_svc_hdl != 0) {
		esp_ble_gatts_stop_service(hidd_svc_hdl);
		esp_ble_gatts_delete_service(hidd_svc_hdl);
	} else {
		return ESP_FAIL;
	}

	/* register the HID device profile to the BTA_GATTS module */
	esp_ble_gatts_app_unregister(hidd_le_env.gatt_if);

	return ESP_OK;
}

void esp_hidd_send_consumer_value(uint16_t conn_id, uint8_t key_cmd, bool key_pressed)
{
	uint8_t buffer[HID_CC_IN_RPT_LEN] = { 0, 0 };
	if (key_pressed) {
		ESP_LOGD(HID_LE_PRF_TAG, "hid_consumer_build_report");
		hid_consumer_build_report(buffer, key_cmd);
	}
	ESP_LOGD(HID_LE_PRF_TAG, "buffer[0] = %x, buffer[1] = %x", buffer[0], buffer[1]);
	hid_dev_send_report(hidd_le_env.gatt_if, conn_id, HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT, HID_CC_IN_RPT_LEN, buffer);
	return;
}

int esp_hidd_send_keyboard_value(uint16_t conn_id, key_mask_t special_key_mask, uint8_t *keyboard_cmd, uint8_t num_key)
{
	if (num_key > HID_KEYBOARD_IN_RPT_LEN - 2) {
		ESP_LOGE(HID_LE_PRF_TAG, "%s(), the number key should not be more than %d", __func__, HID_KEYBOARD_IN_RPT_LEN);
		return 1;
	}

	uint8_t buffer[HID_KEYBOARD_IN_RPT_LEN] = { 0 };

	buffer[0] = special_key_mask;

	for (int i = 0; i < num_key; i++) {
		buffer[i + 2] = keyboard_cmd[i];
	}

	ESP_LOGD(HID_LE_PRF_TAG, "the key vaule = %d,%d,%d, %d, %d, %d,%d, %d",
		 buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
	return hid_dev_send_report(hidd_le_env.gatt_if, conn_id, HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT, HID_KEYBOARD_IN_RPT_LEN, buffer);
}

int esp_hidd_send_mouse_value(uint16_t conn_id, uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y)
{
	uint8_t buffer[HID_MOUSE_IN_RPT_LEN];

	buffer[0] = mouse_button;	// Buttons
	buffer[1] = mickeys_x;	// X
	buffer[2] = mickeys_y;	// Y
	buffer[3] = 0;		// Wheel
	buffer[4] = 0;		// AC Pan

	return hid_dev_send_report(hidd_le_env.gatt_if, conn_id, HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT, HID_MOUSE_IN_RPT_LEN, buffer);
}

void hid_dev_register_reports(uint8_t num_reports, hid_report_map_t *p_report)
{
	hid_dev_rpt_tbl = p_report;
	hid_dev_rpt_tbl_Len = num_reports;
	return;
}

void hidd_le_init(void)
{

	// Reset the hid device target environment
	memset(&hidd_le_env, 0, sizeof(hidd_le_env_t));
}

void hidd_clcb_alloc(uint16_t conn_id, esp_bd_addr_t bda)
{
	uint8_t i_clcb = 0;
	hidd_clcb_t *p_clcb = NULL;

	for (i_clcb = 0, p_clcb = hidd_le_env.hidd_clcb; i_clcb < HID_MAX_APPS; i_clcb++, p_clcb++) {
		if (!p_clcb->in_use) {
			p_clcb->in_use = true;
			p_clcb->conn_id = conn_id;
			p_clcb->connected = true;
			memcpy(p_clcb->remote_bda, bda, ESP_BD_ADDR_LEN);
			break;
		}
	}
	return;
}

bool hidd_clcb_dealloc(uint16_t conn_id)
{
	uint8_t i_clcb = 0;
	hidd_clcb_t *p_clcb = NULL;

	for (i_clcb = 0, p_clcb = hidd_le_env.hidd_clcb; i_clcb < HID_MAX_APPS; i_clcb++, p_clcb++) {
		memset(p_clcb, 0, sizeof(hidd_clcb_t));
		return true;
	}

	return false;
}

void hidd_set_attr_value(uint16_t handle, uint16_t val_len, const uint8_t *value)
{
	hidd_inst_t *hidd_inst = &hidd_le_env.hidd_inst;
	if (hidd_inst->att_tbl[HIDD_LE_IDX_HID_INFO_VAL] <= handle && hidd_inst->att_tbl[HIDD_LE_IDX_REPORT_REP_REF] >= handle) {
		esp_ble_gatts_set_attr_value(handle, val_len, value);
	} else {
		ESP_LOGE(HID_LE_PRF_TAG, "%s error:Invalid handle value.", __func__);
	}
	return;
}

void hidd_get_attr_value(uint16_t handle, uint16_t *length, uint8_t **value)
{
	hidd_inst_t *hidd_inst = &hidd_le_env.hidd_inst;
	if (hidd_inst->att_tbl[HIDD_LE_IDX_HID_INFO_VAL] <= handle && hidd_inst->att_tbl[HIDD_LE_IDX_REPORT_REP_REF] >= handle) {
		esp_ble_gatts_get_attr_value(handle, length, (const uint8_t **)value);
	} else {
		ESP_LOGE(HID_LE_PRF_TAG, "%s error:Invalid handle value.", __func__);
	}

	return;
}

static hid_report_map_t *hid_dev_rpt_by_id(uint8_t id, uint8_t type)
{
	hid_report_map_t *rpt = hid_dev_rpt_tbl;

	for (uint8_t i = hid_dev_rpt_tbl_Len; i > 0; i--, rpt++) {
		if (rpt->id == id && rpt->type == type && rpt->mode == hidProtocolMode) {
			return rpt;
		}
	}

	return NULL;
}

int hid_dev_send_report(esp_gatt_if_t gatts_if, uint16_t conn_id, uint8_t id, uint8_t type, uint8_t length, uint8_t *data)
{
	hid_report_map_t *p_rpt;

	// get att handle for report
	if ((p_rpt = hid_dev_rpt_by_id(id, type)) != NULL) {
		// if notifications are enabled
		ESP_LOGD(HID_LE_PRF_TAG, "%s(), send the report, handle = %d", __func__, p_rpt->handle);
		return esp_ble_gatts_send_indicate(gatts_if, conn_id, p_rpt->handle, length, data, false);
	}
	return 1;

}

void hid_consumer_build_report(uint8_t *buffer, consumer_cmd_t cmd)
{
	if (!buffer) {
		ESP_LOGE(HID_LE_PRF_TAG, "%s(), the buffer is NULL, hid build report failed.", __func__);
		return;
	}

	switch (cmd) {
	case HID_CONSUMER_CHANNEL_UP:
		HID_CC_RPT_SET_CHANNEL(buffer, HID_CC_RPT_CHANNEL_UP);
		break;

	case HID_CONSUMER_CHANNEL_DOWN:
		HID_CC_RPT_SET_CHANNEL(buffer, HID_CC_RPT_CHANNEL_DOWN);
		break;

	case HID_CONSUMER_VOLUME_UP:
		HID_CC_RPT_SET_VOLUME_UP(buffer);
		break;

	case HID_CONSUMER_VOLUME_DOWN:
		HID_CC_RPT_SET_VOLUME_DOWN(buffer);
		break;

	case HID_CONSUMER_MUTE:
		HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_MUTE);
		break;

	case HID_CONSUMER_POWER:
		HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_POWER);
		break;

	case HID_CONSUMER_RECALL_LAST:
		HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_LAST);
		break;

	case HID_CONSUMER_ASSIGN_SEL:
		HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_ASSIGN_SEL);
		break;

	case HID_CONSUMER_PLAY:
		HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_PLAY);
		break;

	case HID_CONSUMER_PAUSE:
		HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_PAUSE);
		break;

	case HID_CONSUMER_RECORD:
		HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_RECORD);
		break;

	case HID_CONSUMER_FAST_FORWARD:
		HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_FAST_FWD);
		break;

	case HID_CONSUMER_REWIND:
		HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_REWIND);
		break;

	case HID_CONSUMER_SCAN_NEXT_TRK:
		HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_SCAN_NEXT_TRK);
		break;

	case HID_CONSUMER_SCAN_PREV_TRK:
		HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_SCAN_PREV_TRK);
		break;

	case HID_CONSUMER_STOP:
		HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_STOP);
		break;

	default:
		break;
	}

	return;
}
