#include "esp_err.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"

/* HID information flags */
#define HID_FLAGS_REMOTE_WAKE           0x01	// RemoteWake
#define HID_FLAGS_NORMALLY_CONNECTABLE  0x02	// NormallyConnectable

/* Control point commands */
#define HID_CMD_SUSPEND                 0x00	// Suspend
#define HID_CMD_EXIT_SUSPEND            0x01	// Exit Suspend

/* HID protocol mode values */
#define HID_PROTOCOL_MODE_BOOT          0x00	// Boot Protocol Mode
#define HID_PROTOCOL_MODE_REPORT        0x01	// Report Protocol Mode

/* Attribute value lengths */
#define HID_PROTOCOL_MODE_LEN           1	// HID Protocol Mode
#define HID_INFORMATION_LEN             4	// HID Information
#define HID_REPORT_REF_LEN              2	// HID Report Reference Descriptor
#define HID_EXT_REPORT_REF_LEN          2	// External Report Reference Descriptor

// HID feature flags
#define HID_KBD_FLAGS             HID_FLAGS_REMOTE_WAKE

/* HID Report type */
#define HID_REPORT_TYPE_INPUT       1
#define HID_REPORT_TYPE_OUTPUT      2
#define HID_REPORT_TYPE_FEATURE     3

//HID BLE profile log tag
#define HID_LE_PRF_TAG                        "HID_LE_PRF"

// Maximal number of HIDS that can be added in the DB
#ifndef USE_ONE_HIDS_INSTANCE
#define HIDD_LE_NB_HIDS_INST_MAX              (2)
#else
#define HIDD_LE_NB_HIDS_INST_MAX              (1)
#endif

#define HIDD_GREAT_VER   0x01	//Version + Subversion
#define HIDD_SUB_VER     0x00	//Version + Subversion
#define HIDD_VERSION     ((HIDD_GREAT_VER<<8)|HIDD_SUB_VER)	//Version + Subversion

#define HID_MAX_APPS                 1

// Number of HID reports defined in the service
#define HID_NUM_REPORTS          9

// HID Report IDs for the service
#define HID_RPT_ID_MOUSE_IN      1	// Mouse input report ID
#define HID_RPT_ID_KEY_IN        2	// Keyboard input report ID
#define HID_RPT_ID_CC_IN         3	//Consumer Control input report ID
#define HID_RPT_ID_VENDOR_OUT    4	// Vendor output report ID
#define HID_RPT_ID_LED_OUT       2	// LED output report ID
#define HID_RPT_ID_FEATURE       0	// Feature report ID

#define HIDD_APP_ID			0x1812	//ATT_SVC_HID

#define BATTRAY_APP_ID       0x180f

#define ATT_SVC_HID          0x1812

// Maximal number of Report Char. that can be added in the DB for one HIDS - Up to 11
#define HIDD_LE_NB_REPORT_INST_MAX            (5)

// Maximal length of Report Char. Value
#define HIDD_LE_REPORT_MAX_LEN                (255)
// Maximal length of Report Map Char. Value
#define HIDD_LE_REPORT_MAP_MAX_LEN            (512)

// Length of Boot Report Char. Value Maximal Length
#define HIDD_LE_BOOT_REPORT_MAX_LEN           (8)

// Boot KB Input Report Notification Configuration Bit Mask
#define HIDD_LE_BOOT_KB_IN_NTF_CFG_MASK       (0x40)
// Boot KB Input Report Notification Configuration Bit Mask
#define HIDD_LE_BOOT_MOUSE_IN_NTF_CFG_MASK    (0x80)
// Boot Report Notification Configuration Bit Mask
#define HIDD_LE_REPORT_NTF_CFG_MASK           (0x20)

#define HIDD_LE_CLEANUP_FNCT        (NULL)

#define LEFT_CONTROL_KEY_MASK        (1 << 0)
#define LEFT_SHIFT_KEY_MASK          (1 << 1)
#define LEFT_ALT_KEY_MASK            (1 << 2)
#define LEFT_GUI_KEY_MASK            (1 << 3)
#define RIGHT_CONTROL_KEY_MASK       (1 << 4)
#define RIGHT_SHIFT_KEY_MASK         (1 << 5)
#define RIGHT_ALT_KEY_MASK           (1 << 6)
#define RIGHT_GUI_KEY_MASK           (1 << 7)

/* HID Report type */
#define HID_TYPE_INPUT       1
#define HID_TYPE_OUTPUT      2
#define HID_TYPE_FEATURE     3

// HID Keyboard/Keypad Usage IDs (subset of the codes available in the USB HID Usage Tables spec)
#define HID_KEY_RESERVED       0	// No event inidicated
#define HID_KEY_A              4	// Keyboard a and A
#define HID_KEY_B              5	// Keyboard b and B
#define HID_KEY_C              6	// Keyboard c and C
#define HID_KEY_D              7	// Keyboard d and D
#define HID_KEY_E              8	// Keyboard e and E
#define HID_KEY_F              9	// Keyboard f and F
#define HID_KEY_G              10	// Keyboard g and G
#define HID_KEY_H              11	// Keyboard h and H
#define HID_KEY_I              12	// Keyboard i and I
#define HID_KEY_J              13	// Keyboard j and J
#define HID_KEY_K              14	// Keyboard k and K
#define HID_KEY_L              15	// Keyboard l and L
#define HID_KEY_M              16	// Keyboard m and M
#define HID_KEY_N              17	// Keyboard n and N
#define HID_KEY_O              18	// Keyboard o and O
#define HID_KEY_P              19	// Keyboard p and p
#define HID_KEY_Q              20	// Keyboard q and Q
#define HID_KEY_R              21	// Keyboard r and R
#define HID_KEY_S              22	// Keyboard s and S
#define HID_KEY_T              23	// Keyboard t and T
#define HID_KEY_U              24	// Keyboard u and U
#define HID_KEY_V              25	// Keyboard v and V
#define HID_KEY_W              26	// Keyboard w and W
#define HID_KEY_X              27	// Keyboard x and X
#define HID_KEY_Y              28	// Keyboard y and Y
#define HID_KEY_Z              29	// Keyboard z and Z
#define HID_KEY_1              30	// Keyboard 1 and !
#define HID_KEY_2              31	// Keyboard 2 and @
#define HID_KEY_3              32	// Keyboard 3 and #
#define HID_KEY_4              33	// Keyboard 4 and %
#define HID_KEY_5              34	// Keyboard 5 and %
#define HID_KEY_6              35	// Keyboard 6 and ^
#define HID_KEY_7              36	// Keyboard 7 and &
#define HID_KEY_8              37	// Keyboard 8 and *
#define HID_KEY_9              38	// Keyboard 9 and (
#define HID_KEY_0              39	// Keyboard 0 and )
#define HID_KEY_RETURN         40	// Keyboard Return (ENTER)
#define HID_KEY_ESCAPE         41	// Keyboard ESCAPE
#define HID_KEY_DELETE         42	// Keyboard DELETE (Backspace)
#define HID_KEY_TAB            43	// Keyboard Tab
#define HID_KEY_SPACEBAR       44	// Keyboard Spacebar
#define HID_KEY_MINUS          45	// Keyboard - and (underscore)
#define HID_KEY_EQUAL          46	// Keyboard = and +
#define HID_KEY_LEFT_BRKT      47	// Keyboard [ and {
#define HID_KEY_RIGHT_BRKT     48	// Keyboard ] and }
#define HID_KEY_BACK_SLASH     49	// Keyboard \ and |
#define HID_KEY_SEMI_COLON     51	// Keyboard ; and :
#define HID_KEY_SGL_QUOTE      52	// Keyboard ' and "
#define HID_KEY_GRV_ACCENT     53	// Keyboard Grave Accent and Tilde
#define HID_KEY_COMMA          54	// Keyboard , and <
#define HID_KEY_DOT            55	// Keyboard . and >
#define HID_KEY_FWD_SLASH      56	// Keyboard / and ?
#define HID_KEY_CAPS_LOCK      57	// Keyboard Caps Lock
#define HID_KEY_F1             58	// Keyboard F1
#define HID_KEY_F2             59	// Keyboard F2
#define HID_KEY_F3             60	// Keyboard F3
#define HID_KEY_F4             61	// Keyboard F4
#define HID_KEY_F5             62	// Keyboard F5
#define HID_KEY_F6             63	// Keyboard F6
#define HID_KEY_F7             64	// Keyboard F7
#define HID_KEY_F8             65	// Keyboard F8
#define HID_KEY_F9             66	// Keyboard F9
#define HID_KEY_F10            67	// Keyboard F10
#define HID_KEY_F11            68	// Keyboard F11
#define HID_KEY_F12            69	// Keyboard F12
#define HID_KEY_PRNT_SCREEN    70	// Keyboard Print Screen
#define HID_KEY_SCROLL_LOCK    71	// Keyboard Scroll Lock
#define HID_KEY_PAUSE          72	// Keyboard Pause
#define HID_KEY_INSERT         73	// Keyboard Insert
#define HID_KEY_HOME           74	// Keyboard Home
#define HID_KEY_PAGE_UP        75	// Keyboard PageUp
#define HID_KEY_DELETE_FWD     76	// Keyboard Delete Forward
#define HID_KEY_END            77	// Keyboard End
#define HID_KEY_PAGE_DOWN      78	// Keyboard PageDown
#define HID_KEY_RIGHT_ARROW    79	// Keyboard RightArrow
#define HID_KEY_LEFT_ARROW     80	// Keyboard LeftArrow
#define HID_KEY_DOWN_ARROW     81	// Keyboard DownArrow
#define HID_KEY_UP_ARROW       82	// Keyboard UpArrow
#define HID_KEY_NUM_LOCK       83	// Keypad Num Lock and Clear
#define HID_KEY_DIVIDE         84	// Keypad /
#define HID_KEY_MULTIPLY       85	// Keypad *
#define HID_KEY_SUBTRACT       86	// Keypad -
#define HID_KEY_ADD            87	// Keypad +
#define HID_KEY_ENTER          88	// Keypad ENTER
#define HID_KEYPAD_1           89	// Keypad 1 and End
#define HID_KEYPAD_2           90	// Keypad 2 and Down Arrow
#define HID_KEYPAD_3           91	// Keypad 3 and PageDn
#define HID_KEYPAD_4           92	// Keypad 4 and Lfet Arrow
#define HID_KEYPAD_5           93	// Keypad 5
#define HID_KEYPAD_6           94	// Keypad 6 and Right Arrow
#define HID_KEYPAD_7           95	// Keypad 7 and Home
#define HID_KEYPAD_8           96	// Keypad 8 and Up Arrow
#define HID_KEYPAD_9           97	// Keypad 9 and PageUp
#define HID_KEYPAD_0           98	// Keypad 0 and Insert
#define HID_KEYPAD_DOT         99	// Keypad . and Delete
#define HID_KEY_MUTE           127	// Keyboard Mute
#define HID_KEY_VOLUME_UP      128	// Keyboard Volume up
#define HID_KEY_VOLUME_DOWN    129	// Keyboard Volume down
#define HID_KEY_LEFT_CTRL      224	// Keyboard LeftContorl
#define HID_KEY_LEFT_SHIFT     225	// Keyboard LeftShift
#define HID_KEY_LEFT_ALT       226	// Keyboard LeftAlt
#define HID_KEY_LEFT_GUI       227	// Keyboard LeftGUI
#define HID_KEY_RIGHT_CTRL     228	// Keyboard RightContorl
#define HID_KEY_RIGHT_SHIFT    229	// Keyboard RightShift
#define HID_KEY_RIGHT_ALT      230	// Keyboard RightAlt
#define HID_KEY_RIGHT_GUI      231	// Keyboard RightGUI

#define HID_MOUSE_LEFT       253
#define HID_MOUSE_MIDDLE     254
#define HID_MOUSE_RIGHT      255

// HID Consumer Usage IDs (subset of the codes available in the USB HID Usage Tables spec)
#define HID_CONSUMER_POWER          48	// Power
#define HID_CONSUMER_RESET          49	// Reset
#define HID_CONSUMER_SLEEP          50	// Sleep

#define HID_CONSUMER_MENU           64	// Menu
#define HID_CONSUMER_SELECTION      128	// Selection
#define HID_CONSUMER_ASSIGN_SEL     129	// Assign Selection
#define HID_CONSUMER_MODE_STEP      130	// Mode Step
#define HID_CONSUMER_RECALL_LAST    131	// Recall Last
#define HID_CONSUMER_QUIT           148	// Quit
#define HID_CONSUMER_HELP           149	// Help
#define HID_CONSUMER_CHANNEL_UP     156	// Channel Increment
#define HID_CONSUMER_CHANNEL_DOWN   157	// Channel Decrement

#define HID_CONSUMER_PLAY           176	// Play
#define HID_CONSUMER_PAUSE          177	// Pause
#define HID_CONSUMER_RECORD         178	// Record
#define HID_CONSUMER_FAST_FORWARD   179	// Fast Forward
#define HID_CONSUMER_REWIND         180	// Rewind
#define HID_CONSUMER_SCAN_NEXT_TRK  181	// Scan Next Track
#define HID_CONSUMER_SCAN_PREV_TRK  182	// Scan Previous Track
#define HID_CONSUMER_STOP           183	// Stop
#define HID_CONSUMER_EJECT          184	// Eject
#define HID_CONSUMER_RANDOM_PLAY    185	// Random Play
#define HID_CONSUMER_SELECT_DISC    186	// Select Disk
#define HID_CONSUMER_ENTER_DISC     187	// Enter Disc
#define HID_CONSUMER_REPEAT         188	// Repeat
#define HID_CONSUMER_STOP_EJECT     204	// Stop/Eject
#define HID_CONSUMER_PLAY_PAUSE     205	// Play/Pause
#define HID_CONSUMER_PLAY_SKIP      206	// Play/Skip

#define HID_CONSUMER_VOLUME         224	// Volume
#define HID_CONSUMER_BALANCE        225	// Balance
#define HID_CONSUMER_MUTE           226	// Mute
#define HID_CONSUMER_BASS           227	// Bass
#define HID_CONSUMER_VOLUME_UP      233	// Volume Increment
#define HID_CONSUMER_VOLUME_DOWN    234	// Volume Decrement

#define HID_CC_RPT_MUTE                 1
#define HID_CC_RPT_POWER                2
#define HID_CC_RPT_LAST                 3
#define HID_CC_RPT_ASSIGN_SEL           4
#define HID_CC_RPT_PLAY                 5
#define HID_CC_RPT_PAUSE                6
#define HID_CC_RPT_RECORD               7
#define HID_CC_RPT_FAST_FWD             8
#define HID_CC_RPT_REWIND               9
#define HID_CC_RPT_SCAN_NEXT_TRK        10
#define HID_CC_RPT_SCAN_PREV_TRK        11
#define HID_CC_RPT_STOP                 12

#define HID_CC_RPT_CHANNEL_UP           0x01
#define HID_CC_RPT_CHANNEL_DOWN         0x03
#define HID_CC_RPT_VOLUME_UP            0x40
#define HID_CC_RPT_VOLUME_DOWN          0x80

// HID Consumer Control report bitmasks
#define HID_CC_RPT_NUMERIC_BITS         0xF0
#define HID_CC_RPT_CHANNEL_BITS         0xCF
#define HID_CC_RPT_VOLUME_BITS          0x3F
#define HID_CC_RPT_BUTTON_BITS          0xF0
#define HID_CC_RPT_SELECTION_BITS       0xCF

// Macros for the HID Consumer Control 2-byte report
#define HID_CC_RPT_SET_NUMERIC(s, x)    (s)[0] &= HID_CC_RPT_NUMERIC_BITS;   \
                                        (s)[0] = (x)
#define HID_CC_RPT_SET_CHANNEL(s, x)    (s)[0] &= HID_CC_RPT_CHANNEL_BITS;   \
                                        (s)[0] |= ((x) & 0x03) << 4
#define HID_CC_RPT_SET_VOLUME_UP(s)     (s)[0] &= HID_CC_RPT_VOLUME_BITS;    \
                                        (s)[0] |= 0x40
#define HID_CC_RPT_SET_VOLUME_DOWN(s)   (s)[0] &= HID_CC_RPT_VOLUME_BITS;    \
                                        (s)[0] |= 0x80
#define HID_CC_RPT_SET_BUTTON(s, x)     (s)[1] &= HID_CC_RPT_BUTTON_BITS;    \
                                        (s)[1] |= (x)
#define HID_CC_RPT_SET_SELECTION(s, x)  (s)[1] &= HID_CC_RPT_SELECTION_BITS; \
                                        (s)[1] |= ((x) & 0x03) << 4

/// Battery Service Attributes Indexes
enum {
	BAS_IDX_SVC,

	BAS_IDX_BATT_LVL_CHAR,
	BAS_IDX_BATT_LVL_VAL,
	BAS_IDX_BATT_LVL_NTF_CFG,
	BAS_IDX_BATT_LVL_PRES_FMT,

	BAS_IDX_NB,
};


typedef enum {
	ESP_HIDD_EVENT_REG_FINISH = 0,
	ESP_BAT_EVENT_REG,
	ESP_HIDD_EVENT_DEINIT_FINISH,
	ESP_HIDD_EVENT_BLE_CONNECT,
	ESP_HIDD_EVENT_BLE_DISCONNECT,
	ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT,
	ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT,
} esp_hidd_cb_event_t;

// HID config status
typedef enum {
	ESP_HIDD_STA_CONN_SUCCESS = 0x00,
	ESP_HIDD_STA_CONN_FAIL = 0x01,
} esp_hidd_sta_conn_state_t;

// HID init status
typedef enum {
	ESP_HIDD_INIT_OK = 0,
	ESP_HIDD_INIT_FAILED = 1,
} esp_hidd_init_state_t;

// HID deinit status
typedef enum {
	ESP_HIDD_DEINIT_OK = 0,
	ESP_HIDD_DEINIT_FAILED = 0,
} esp_hidd_deinit_state_t;

// HID Service Attributes Indexes
enum {
	HIDD_LE_IDX_SVC,
	// Included Service
	HIDD_LE_IDX_INCL_SVC,
	// HID Information
	HIDD_LE_IDX_HID_INFO_CHAR,
	HIDD_LE_IDX_HID_INFO_VAL,
	// HID Control Point
	HIDD_LE_IDX_HID_CTNL_PT_CHAR,
	HIDD_LE_IDX_HID_CTNL_PT_VAL,
	// Report Map
	HIDD_LE_IDX_REPORT_MAP_CHAR,
	HIDD_LE_IDX_REPORT_MAP_VAL,
	HIDD_LE_IDX_REPORT_MAP_EXT_REP_REF,
	// Protocol Mode
	HIDD_LE_IDX_PROTO_MODE_CHAR,
	HIDD_LE_IDX_PROTO_MODE_VAL,
	// Report mouse input
	HIDD_LE_IDX_REPORT_MOUSE_IN_CHAR,
	HIDD_LE_IDX_REPORT_MOUSE_IN_VAL,
	HIDD_LE_IDX_REPORT_MOUSE_IN_CCC,
	HIDD_LE_IDX_REPORT_MOUSE_REP_REF,
	//Report Key input
	HIDD_LE_IDX_REPORT_KEY_IN_CHAR,
	HIDD_LE_IDX_REPORT_KEY_IN_VAL,
	HIDD_LE_IDX_REPORT_KEY_IN_CCC,
	HIDD_LE_IDX_REPORT_KEY_IN_REP_REF,
	// Report Led output
	HIDD_LE_IDX_REPORT_LED_OUT_CHAR,
	HIDD_LE_IDX_REPORT_LED_OUT_VAL,
	HIDD_LE_IDX_REPORT_LED_OUT_REP_REF,
	// Report Vendor
	HIDD_LE_IDX_REPORT_VENDOR_OUT_CHAR,
	HIDD_LE_IDX_REPORT_VENDOR_OUT_VAL,
	HIDD_LE_IDX_REPORT_VENDOR_OUT_REP_REF,
	HIDD_LE_IDX_REPORT_CC_IN_CHAR,
	HIDD_LE_IDX_REPORT_CC_IN_VAL,
	HIDD_LE_IDX_REPORT_CC_IN_CCC,
	HIDD_LE_IDX_REPORT_CC_IN_REP_REF,
	// Boot Keyboard Input Report
	HIDD_LE_IDX_BOOT_KB_IN_REPORT_CHAR,
	HIDD_LE_IDX_BOOT_KB_IN_REPORT_VAL,
	HIDD_LE_IDX_BOOT_KB_IN_REPORT_NTF_CFG,
	// Boot Keyboard Output Report
	HIDD_LE_IDX_BOOT_KB_OUT_REPORT_CHAR,
	HIDD_LE_IDX_BOOT_KB_OUT_REPORT_VAL,
	// Boot Mouse Input Report
	HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_CHAR,
	HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_VAL,
	HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG,
	// Report
	HIDD_LE_IDX_REPORT_CHAR,
	HIDD_LE_IDX_REPORT_VAL,
	HIDD_LE_IDX_REPORT_REP_REF,
	//HIDD_LE_IDX_REPORT_NTF_CFG,
	HIDD_LE_IDX_NB,
};

// Attribute Table Indexes
enum {
	HIDD_LE_INFO_CHAR,
	HIDD_LE_CTNL_PT_CHAR,
	HIDD_LE_REPORT_MAP_CHAR,
	HIDD_LE_REPORT_CHAR,
	HIDD_LE_PROTO_MODE_CHAR,
	HIDD_LE_BOOT_KB_IN_REPORT_CHAR,
	HIDD_LE_BOOT_KB_OUT_REPORT_CHAR,
	HIDD_LE_BOOT_MOUSE_IN_REPORT_CHAR,
	HIDD_LE_CHAR_MAX	//= HIDD_LE_REPORT_CHAR + HIDD_LE_NB_REPORT_INST_MAX,
};

//att read event table Indexs
enum {
	HIDD_LE_READ_INFO_EVT,
	HIDD_LE_READ_CTNL_PT_EVT,
	HIDD_LE_READ_REPORT_MAP_EVT,
	HIDD_LE_READ_REPORT_EVT,
	HIDD_LE_READ_PROTO_MODE_EVT,
	HIDD_LE_BOOT_KB_IN_REPORT_EVT,
	HIDD_LE_BOOT_KB_OUT_REPORT_EVT,
	HIDD_LE_BOOT_MOUSE_IN_REPORT_EVT,

	HID_LE_EVT_MAX
};

// Client Characteristic Configuration Codes
enum {
	HIDD_LE_DESC_MASK = 0x10,

	HIDD_LE_BOOT_KB_IN_REPORT_CFG = HIDD_LE_BOOT_KB_IN_REPORT_CHAR | HIDD_LE_DESC_MASK,
	HIDD_LE_BOOT_MOUSE_IN_REPORT_CFG = HIDD_LE_BOOT_MOUSE_IN_REPORT_CHAR | HIDD_LE_DESC_MASK,
	HIDD_LE_REPORT_CFG = HIDD_LE_REPORT_CHAR | HIDD_LE_DESC_MASK,
};

// Features Flag Values
enum {
	HIDD_LE_CFG_KEYBOARD = 0x01,
	HIDD_LE_CFG_MOUSE = 0x02,
	HIDD_LE_CFG_PROTO_MODE = 0x04,
	HIDD_LE_CFG_MAP_EXT_REF = 0x08,
	HIDD_LE_CFG_BOOT_KB_WR = 0x10,
	HIDD_LE_CFG_BOOT_MOUSE_WR = 0x20,
};

// Report Char. Configuration Flag Values
enum {
	HIDD_LE_CFG_REPORT_IN = 0x01,
	HIDD_LE_CFG_REPORT_OUT = 0x02,
	//HOGPD_CFG_REPORT_FEAT can be used as a mask to check Report type
	HIDD_LE_CFG_REPORT_FEAT = 0x03,
	HIDD_LE_CFG_REPORT_WR = 0x10,
};

/**
 * @brief HIDD callback parameters union
 */
typedef union {
	struct hidd_init_finish_evt_param {
		esp_hidd_init_state_t state;
		esp_gatt_if_t gatts_if;
	} init_finish;
	struct hidd_deinit_finish_evt_param {
		esp_hidd_deinit_state_t state;
	} deinit_finish;
	struct hidd_connect_evt_param {
		uint16_t conn_id;
		esp_bd_addr_t remote_bda;
	} connect;
	struct hidd_disconnect_evt_param {
		esp_bd_addr_t remote_bda;
	} disconnect;
	struct hidd_vendor_write_evt_param {
		uint16_t conn_id;
		uint16_t report_id;
		uint16_t length;
		uint8_t *data;
	} vendor_write;
	struct hidd_led_write_evt_param {
		uint16_t conn_id;
		uint8_t report_id;
		uint8_t length;
		uint8_t *data;
	} led_write;
} esp_hidd_cb_param_t;

typedef uint8_t keyboard_cmd_t;
typedef uint8_t mouse_cmd_t;
typedef uint8_t consumer_cmd_t;
typedef uint8_t key_mask_t;
typedef void (*esp_hidd_event_cb_t)(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

// HIDD Features structure
typedef struct {
	// Service Features
	uint8_t svc_features;
	// Number of Report Char. instances to add in the database
	uint8_t report_nb;
	// Report Char. Configuration
	uint8_t report_char_cfg[HIDD_LE_NB_REPORT_INST_MAX];
} hidd_feature_t;

typedef struct {
	bool in_use;
	bool congest;
	uint16_t conn_id;
	bool connected;
	esp_bd_addr_t remote_bda;
	uint32_t trans_id;
	uint8_t cur_srvc_id;
} hidd_clcb_t;

// HID report mapping table
typedef struct {
	uint16_t handle;	// Handle of report characteristic
	uint16_t cccdHandle;	// Handle of CCCD for report characteristic
	uint8_t id;		// Report ID
	uint8_t type;		// Report type
	uint8_t mode;		// Protocol mode (report or boot)
} hidRptMap_t;

typedef struct {
	// hidd profile id
	uint8_t app_id;
	// Notified handle
	uint16_t ntf_handle;
	//Attribute handle Table
	uint16_t att_tbl[HIDD_LE_IDX_NB];
	// Supported Features
	hidd_feature_t hidd_feature[HIDD_LE_NB_HIDS_INST_MAX];
	// Current Protocol Mode
	uint8_t proto_mode[HIDD_LE_NB_HIDS_INST_MAX];
	// Number of HIDS added in the database
	uint8_t hids_nb;
	uint8_t pending_evt;
	uint16_t pending_hal;
} hidd_inst_t;

// Report Reference structure
typedef struct {
	//Report ID
	uint8_t report_id;
	//Report Type
	uint8_t report_type;
} hids_report_ref_t;

// HID Information structure
typedef struct {
	// bcdHID
	uint16_t bcdHID;
	// bCountryCode
	uint8_t bCountryCode;
	// Flags
	uint8_t flags;
} hids_hid_info_t;

/* service engine control block */
typedef struct {
	hidd_clcb_t hidd_clcb[HID_MAX_APPS];	/* connection link */
	esp_gatt_if_t gatt_if;
	bool enabled;
	bool is_take;
	bool is_primery;
	hidd_inst_t hidd_inst;
	esp_hidd_event_cb_t hidd_cb;
	uint8_t inst_id;
} hidd_le_env_t;

// HID report mapping table
typedef struct {
	uint16_t handle;	// Handle of report characteristic
	uint16_t cccdHandle;	// Handle of CCCD for report characteristic
	uint8_t id;		// Report ID
	uint8_t type;		// Report type
	uint8_t mode;		// Protocol mode (report or boot)
} hid_report_map_t;

// HID dev configuration structure
typedef struct {
	uint32_t idleTimeout;	// Idle timeout in milliseconds
	uint8_t hidFlags;	// HID feature flags

} hid_dev_cfg_t;

extern hidd_le_env_t hidd_le_env;
extern uint8_t hidProtocolMode;

extern esp_gatts_attr_db_t hidd_le_gatt_db[HIDD_LE_IDX_NB];
extern const esp_gatts_attr_db_t bas_att_db[BAS_IDX_NB];
extern esp_gatts_incl_svc_desc_t incl_svc;

void hid_add_id_tbl(void);




void hidd_clcb_alloc(uint16_t conn_id, esp_bd_addr_t bda);
bool hidd_clcb_dealloc(uint16_t conn_id);
void hidd_set_attr_value(uint16_t handle, uint16_t val_len, const uint8_t *value);
void hidd_get_attr_value(uint16_t handle, uint16_t *length, uint8_t **value);
esp_err_t hidd_register_cb(void);

esp_err_t esp_hidd_register_callbacks(esp_hidd_event_cb_t callbacks);
esp_err_t esp_hidd_profile_init(void);
esp_err_t esp_hidd_profile_deinit(void);
void esp_hidd_send_consumer_value(uint16_t conn_id, uint8_t key_cmd, bool key_pressed);
int esp_hidd_send_keyboard_value(uint16_t conn_id, key_mask_t special_key_mask, uint8_t *keyboard_cmd, uint8_t num_key);
int esp_hidd_send_mouse_value(uint16_t conn_id, uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y);

void hid_dev_register_reports(uint8_t num_reports, hid_report_map_t *p_report);
int hid_dev_send_report(esp_gatt_if_t gatts_if, uint16_t conn_id, uint8_t id, uint8_t type, uint8_t length, uint8_t *data);
void hid_consumer_build_report(uint8_t *buffer, consumer_cmd_t cmd);
void hid_keyboard_build_report(uint8_t *buffer, keyboard_cmd_t cmd);
void hid_mouse_build_report(uint8_t *buffer, mouse_cmd_t cmd);
