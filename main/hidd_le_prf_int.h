// Copyright 2017-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __HID_DEVICE_LE_PRF__
#define __HID_DEVICE_LE_PRF__

#include <stdbool.h>

#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#include "uuid_definition.h"

#define HID_LE_PRF_TAG "HID_LE_PRF"

/// Maximal number of HIDS that can be added in the DB
#ifndef USE_ONE_HIDS_INSTANCE
#define HIDD_LE_NB_HIDS_INST_MAX (2)
#else
#define HIDD_LE_NB_HIDS_INST_MAX (1)
#endif

#define HIDD_GREAT_VER 0x01                                  // Version + Subversion
#define HIDD_SUB_VER 0x00                                    // Version + Subversion
#define HIDD_VERSION ((HIDD_GREAT_VER << 8) | HIDD_SUB_VER)  // Version + Subversion
#define HIDD_APP_ID 0x1812
#define BATTRAY_APP_ID 0x180f
#define ATT_SVC_HID 0x1812

#define HID_MAX_APPS 1
#define HID_NUM_REPORTS 9        // Number of HID reports defined in the service
#define HID_RPT_ID_MOUSE_IN 1    // Mouse input report ID
#define HID_RPT_ID_KEY_IN 2      // Keyboard input report ID
#define HID_RPT_ID_CC_IN 3       // Consumer Control input report ID
#define HID_RPT_ID_VENDOR_OUT 4  // Vendor output report ID
#define HID_RPT_ID_LED_OUT 0     // LED output report ID
#define HID_RPT_ID_FEATURE 0     // Feature report ID

#define HIDD_LE_NB_REPORT_INST_MAX (5)             // Max number of Report Char. added in the DB for one HID - Up to 11
#define HIDD_LE_REPORT_MAX_LEN (255)               // Maximal length of Report Char. Value
#define HIDD_LE_REPORT_MAP_MAX_LEN (512)           // Maximal length of Report Map Char. Value
#define HIDD_LE_BOOT_REPORT_MAX_LEN (8)            // Length of Boot Report Char. Value Maximal Length
#define HIDD_LE_BOOT_KB_IN_NTF_CFG_MASK (0x40)     // Boot KB Input Report Notification Configuration Bit Mask
#define HIDD_LE_BOOT_MOUSE_IN_NTF_CFG_MASK (0x80)  // Boot KB Input Report Notification Configuration Bit Mask
#define HIDD_LE_REPORT_NTF_CFG_MASK (0x20)         // Boot Report Notification Configuration Bit Mask

/* HID information flags */
#define HID_FLAGS_REMOTE_WAKE 0x01           // RemoteWake
#define HID_FLAGS_NORMALLY_CONNECTABLE 0x02  // NormallyConnectable

/* Control point commands */
#define HID_CMD_SUSPEND 0x00       // Suspend
#define HID_CMD_EXIT_SUSPEND 0x01  // Exit Suspend

/* HID protocol mode values */
#define HID_PROTOCOL_MODE_BOOT 0x00    // Boot Protocol Mode
#define HID_PROTOCOL_MODE_REPORT 0x01  // Report Protocol Mode

/* Attribute value lengths */
#define HID_PROTOCOL_MODE_LEN 1   // HID Protocol Mode
#define HID_INFORMATION_LEN 4     // HID Information
#define HID_REPORT_REF_LEN 2      // HID Report Reference Descriptor
#define HID_EXT_REPORT_REF_LEN 2  // External Report Reference Descriptor

// HID feature flags
#define HID_KBD_FLAGS HID_FLAGS_REMOTE_WAKE

/* HID Report type */
#define HID_REPORT_TYPE_INPUT 1
#define HID_REPORT_TYPE_OUTPUT 2
#define HID_REPORT_TYPE_FEATURE 3

/* HID Report type */
#define HID_TYPE_INPUT 1
#define HID_TYPE_OUTPUT 2
#define HID_TYPE_FEATURE 3

#define HID_KEYBOARD_IN_RPT_LEN 8  // HID keyboard input report length
#define HID_LED_OUT_RPT_LEN 1      // HID LED output report length
#define HID_MOUSE_IN_RPT_LEN 5     // HID mouse input report length
#define HID_CC_IN_RPT_LEN 2        // HID consumer control input report length

#define LEFT_CONTROL_KEY_MASK (1 << 0)
#define LEFT_SHIFT_KEY_MASK (1 << 1)
#define LEFT_ALT_KEY_MASK (1 << 2)
#define LEFT_GUI_KEY_MASK (1 << 3)
#define RIGHT_CONTROL_KEY_MASK (1 << 4)
#define RIGHT_SHIFT_KEY_MASK (1 << 5)
#define RIGHT_ALT_KEY_MASK (1 << 6)
#define RIGHT_GUI_KEY_MASK (1 << 7)

// Macros for the HID Consumer Control 2-byte report
#define HID_CC_RPT_SET_SELECTION(s, x) \
  (s)[1] &= HID_CC_RPT_SELECTION_BITS; \
  (s)[1] |= ((x)&0x03) << 4
#define HID_CC_RPT_SET_BUTTON(s, x) \
  (s)[1] &= HID_CC_RPT_BUTTON_BITS; \
  (s)[1] |= (x)
#define HID_CC_RPT_SET_NUMERIC(s, x) \
  (s)[0] &= HID_CC_RPT_NUMERIC_BITS; \
  (s)[0] = (x)
#define HID_CC_RPT_SET_CHANNEL(s, x) \
  (s)[0] &= HID_CC_RPT_CHANNEL_BITS; \
  (s)[0] |= ((x)&0x03) << 4
#define HID_CC_RPT_SET_VOLUME_UP(s) \
  (s)[0] &= HID_CC_RPT_VOLUME_BITS; \
  (s)[0] |= 0x40
#define HID_CC_RPT_SET_VOLUME_DOWN(s) \
  (s)[0] &= HID_CC_RPT_VOLUME_BITS;   \
  (s)[0] |= 0x80

/// Pointer to the connection clean-up function
#define HIDD_LE_CLEANUP_FNCT (NULL)

enum HIDServiceAttrIndex {
  HIDD_LE_IDX_SVC,
  HIDD_LE_IDX_INCL_SVC,                      // Included Service
  HIDD_LE_IDX_HID_INFO_CHAR,                 // HID Information
  HIDD_LE_IDX_HID_INFO_VAL,                  // HID Information
  HIDD_LE_IDX_HID_CTNL_PT_CHAR,              // HID Control Point
  HIDD_LE_IDX_HID_CTNL_PT_VAL,               // HID Control Point
  HIDD_LE_IDX_REPORT_MAP_CHAR,               // Report Map
  HIDD_LE_IDX_REPORT_MAP_VAL,                // Report Map
  HIDD_LE_IDX_REPORT_MAP_EXT_REP_REF,        // Report Map
  HIDD_LE_IDX_PROTO_MODE_CHAR,               // Protocol Mode
  HIDD_LE_IDX_PROTO_MODE_VAL,                // Protocol Mode
  HIDD_LE_IDX_REPORT_MOUSE_IN_CHAR,          // Report mouse input
  HIDD_LE_IDX_REPORT_MOUSE_IN_VAL,           // Report mouse input
  HIDD_LE_IDX_REPORT_MOUSE_IN_CCC,           // Report mouse input
  HIDD_LE_IDX_REPORT_MOUSE_REP_REF,          // Report mouse input
  HIDD_LE_IDX_REPORT_KEY_IN_CHAR,            // Report Key input
  HIDD_LE_IDX_REPORT_KEY_IN_VAL,             // Report Key input
  HIDD_LE_IDX_REPORT_KEY_IN_CCC,             // Report Key input
  HIDD_LE_IDX_REPORT_KEY_IN_REP_REF,         // Report Key input
  HIDD_LE_IDX_REPORT_LED_OUT_CHAR,           /// Report Led output
  HIDD_LE_IDX_REPORT_LED_OUT_VAL,            /// Report Led output
  HIDD_LE_IDX_REPORT_LED_OUT_REP_REF,        /// Report Led output
  HIDD_LE_IDX_REPORT_CC_IN_CHAR,             // Consumer device input
  HIDD_LE_IDX_REPORT_CC_IN_VAL,              // Consumer device input
  HIDD_LE_IDX_REPORT_CC_IN_CCC,              // Consumer device input
  HIDD_LE_IDX_REPORT_CC_IN_REP_REF,          // Consumer device input
  HIDD_LE_IDX_BOOT_KB_IN_REPORT_CHAR,        // Boot Keyboard Input Report
  HIDD_LE_IDX_BOOT_KB_IN_REPORT_VAL,         // Boot Keyboard Input Report
  HIDD_LE_IDX_BOOT_KB_IN_REPORT_NTF_CFG,     // Boot Keyboard Input Report
  HIDD_LE_IDX_BOOT_KB_OUT_REPORT_CHAR,       // Boot Keyboard Output Report
  HIDD_LE_IDX_BOOT_KB_OUT_REPORT_VAL,        // Boot Keyboard Output Report
  HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_CHAR,     // Boot Mouse Input Report
  HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_VAL,      // Boot Mouse Input Report
  HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG,  // Boot Mouse Input Report
  HIDD_LE_IDX_REPORT_CHAR,                   // Report
  HIDD_LE_IDX_REPORT_VAL,                    // Report
  HIDD_LE_IDX_REPORT_REP_REF,                // Report
  HIDD_LE_IDX_NB,                            // Number of IDs
};

enum AttributeTableIndex {
  HIDD_LE_INFO_CHAR,
  HIDD_LE_CTNL_PT_CHAR,
  HIDD_LE_REPORT_MAP_CHAR,
  HIDD_LE_REPORT_CHAR,
  HIDD_LE_PROTO_MODE_CHAR,
  HIDD_LE_BOOT_KB_IN_REPORT_CHAR,
  HIDD_LE_BOOT_KB_OUT_REPORT_CHAR,
  HIDD_LE_BOOT_MOUSE_IN_REPORT_CHAR,
  HIDD_LE_CHAR_MAX  //= HIDD_LE_REPORT_CHAR + HIDD_LE_NB_REPORT_INST_MAX,
};

enum AttributeReadEventIndex {
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

enum ClientCharacteristicConfigCodes {
  HIDD_LE_DESC_MASK = 0x10,
  HIDD_LE_BOOT_KB_IN_REPORT_CFG = HIDD_LE_BOOT_KB_IN_REPORT_CHAR | HIDD_LE_DESC_MASK,
  HIDD_LE_BOOT_MOUSE_IN_REPORT_CFG = HIDD_LE_BOOT_MOUSE_IN_REPORT_CHAR | HIDD_LE_DESC_MASK,
  HIDD_LE_REPORT_CFG = HIDD_LE_REPORT_CHAR | HIDD_LE_DESC_MASK,
};

enum FeaturesFlag {
  HIDD_LE_CFG_KEYBOARD = 0x01,
  HIDD_LE_CFG_MOUSE = 0x02,
  HIDD_LE_CFG_PROTO_MODE = 0x04,
  HIDD_LE_CFG_MAP_EXT_REF = 0x08,
  HIDD_LE_CFG_BOOT_KB_WR = 0x10,
  HIDD_LE_CFG_BOOT_MOUSE_WR = 0x20,
};

enum ReportCharacteristicConfigFlag {
  HIDD_LE_CFG_REPORT_IN = 0x01,
  HIDD_LE_CFG_REPORT_OUT = 0x02,
  // HOGPD_CFG_REPORT_FEAT can be used as a mask to check Report type
  HIDD_LE_CFG_REPORT_FEAT = 0x03,
  HIDD_LE_CFG_REPORT_WR = 0x10,
};

typedef uint8_t key_mask;

typedef uint8_t keyboard_cmd;

typedef uint8_t consumer_cmd;

typedef enum HIDEvent {
  ESP_HIDD_EVENT_REG_FINISH = 0,
  ESP_BAT_EVENT_REG,
  ESP_HIDD_EVENT_DEINIT_FINISH,
  ESP_HIDD_EVENT_BLE_CONNECT,
  ESP_HIDD_EVENT_BLE_DISCONNECT,
  ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT,
} HIDEvent;

typedef enum HIDConnectionStatus {
  ESP_HIDD_STA_CONN_SUCCESS = 0x00,
  ESP_HIDD_STA_CONN_FAIL = 0x01,
} HIDConnectionStatus;

typedef enum HIDInitStatus {
  ESP_HIDD_INIT_OK = 0,
  ESP_HIDD_INIT_FAILED = 1,
} HIDInitStatus;

typedef enum HIDDeInitStatus {
  ESP_HIDD_DEINIT_OK = 0,
  ESP_HIDD_DEINIT_FAILED = 0,
} HIDDeInitStatus;

typedef union HIDCallbackParameters {
  struct HIDInitFinishEvent {
    // ESP_HIDD_EVENT_INIT_FINISH
    HIDInitStatus state;
    esp_gatt_if_t gatts_if;
  } init_finish;

  struct HIDDeInitFinishEvent {
    // ESP_HIDD_EVENT_DEINIT_FINISH
    HIDDeInitStatus state;
  } deinit_finish;

  struct HIDConnectEvent {
    // ESP_HIDD_EVENT_CONNECT
    uint16_t conn_id;
    esp_bd_addr_t remote_bda;
  } connect;

  struct HIDDisconnectEvent {
    // ESP_HIDD_EVENT_DISCONNECT
    esp_bd_addr_t remote_bda;
  } disconnect;

  struct HIDVendorWriteEvent {
    // ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT
    uint16_t conn_id;
    uint16_t report_id;
    uint16_t length;
    uint8_t* data;
  } vendor_write;

} HIDCallbackParameters;

typedef void (*HIDEventCallback)(HIDEvent event, HIDCallbackParameters* param);

typedef struct HIDReportMapping {
  uint16_t handle;      // Handle of report characteristic
  uint16_t cccdHandle;  // Handle of CCCD for report characteristic
  uint8_t id;           // Report ID
  uint8_t type;         // Report type
  uint8_t mode;         // Protocol mode (report or boot)
} HIDReportMapping;

typedef struct HIDDeviceConfiguration {
  uint32_t idleTimeout;  // Idle timeout in milliseconds
  uint8_t hidFlags;      // HID feature flags

} HIDDeviceConfiguration;

typedef struct HIDFeatures {
  /// Service Features
  uint8_t svc_features;
  /// Number of Report Char. instances to add in the database
  uint8_t report_nb;
  /// Report Char. Configuration
  uint8_t report_char_cfg[HIDD_LE_NB_REPORT_INST_MAX];
} HIDFeatures;

typedef struct HIDConnectionLink {
  bool in_use;
  bool congest;
  uint16_t conn_id;
  bool connected;
  esp_bd_addr_t remote_bda;  // Bluetooth Device Address
  uint32_t trans_id;
  uint8_t cur_srvc_id;

} HIDConnectionLink;

typedef struct HIDInstance {
  /// hidd profile id
  uint8_t app_id;
  /// Notified handle
  uint16_t ntf_handle;
  /// Attribute handle Table
  uint16_t att_tbl[HIDD_LE_IDX_NB];
  /// Supported Features
  HIDFeatures hidd_feature[HIDD_LE_NB_HIDS_INST_MAX];
  /// Current Protocol Mode
  uint8_t proto_mode[HIDD_LE_NB_HIDS_INST_MAX];
  /// Number of HIDS added in the database
  uint8_t hids_nb;
  uint8_t pending_evt;
  uint16_t pending_hal;
} HIDInstance;

typedef struct HIDInformation {
  uint16_t bcdHID;
  uint8_t bCountryCode;
  uint8_t flags;
} HIDInformation;

typedef struct HIDServiceEngine {
  HIDConnectionLink hidd_clcb[HID_MAX_APPS]; /* connection link*/
  esp_gatt_if_t gatt_if;
  bool enabled;
  bool is_take;
  bool is_primery;
  HIDInstance hidd_inst;
  HIDEventCallback hidd_cb;
  uint8_t inst_id;
} HIDServiceEngine;

extern HIDServiceEngine hid_engine;
extern uint8_t hidProtocolMode;

void hidd_clcb_alloc(uint16_t conn_id, esp_bd_addr_t bda);

bool hidd_clcb_dealloc(uint16_t conn_id);

void hidd_set_attr_value(uint16_t handle, uint16_t val_len, const uint8_t* value);

void hidd_get_attr_value(uint16_t handle, uint16_t* length, uint8_t** value);

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

void hid_dev_register_reports(uint8_t num_reports, HIDReportMapping* p_report);

#endif
