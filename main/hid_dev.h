#ifndef HID_DEV_H__
#define HID_DEV_H__

#include "esp_bt_defs.h"
#include "esp_err.h"
#include "esp_gatt_defs.h"
#include "hid_keydefinition.h"
#include "hidd_le_prf_int.h"

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
#define HID_CC_RPT_SET_BUTTON(s, x) \
  (s)[1] &= HID_CC_RPT_BUTTON_BITS; \
  (s)[1] |= (x)
#define HID_CC_RPT_SET_SELECTION(s, x) \
  (s)[1] &= HID_CC_RPT_SELECTION_BITS; \
  (s)[1] |= ((x)&0x03) << 4

typedef uint8_t key_mask;

typedef uint8_t keyboard_cmd;

typedef uint8_t consumer_cmd;

typedef enum HIDCallbackEvent {
  ESP_HIDD_EVENT_REG_FINISH = 0,
  ESP_BAT_EVENT_REG,
  ESP_HIDD_EVENT_DEINIT_FINISH,
  ESP_HIDD_EVENT_BLE_CONNECT,
  ESP_HIDD_EVENT_BLE_DISCONNECT,
  ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT,
} esp_hidd_cb_event_t;

typedef enum HIDConfigStatus {
  ESP_HIDD_STA_CONN_SUCCESS = 0x00,
  ESP_HIDD_STA_CONN_FAIL = 0x01,
} esp_hidd_sta_conn_state_t;

typedef enum HIDInitStatus {
  ESP_HIDD_INIT_OK = 0,
  ESP_HIDD_INIT_FAILED = 1,
} esp_hidd_init_state_t;

typedef enum HIDDeInitStatus {
  ESP_HIDD_DEINIT_OK = 0,
  ESP_HIDD_DEINIT_FAILED = 0,
} esp_hidd_deinit_state_t;

typedef union HIDCallbackParameters {
  struct HIDInitFinishEvent {
    // ESP_HIDD_EVENT_INIT_FINISH
    esp_hidd_init_state_t state;
    esp_gatt_if_t gatts_if;
  } init_finish;

  struct HIDDeInitFinishEvent {
    // ESP_HIDD_EVENT_DEINIT_FINISH
    esp_hidd_deinit_state_t state;
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

} esp_hidd_cb_param_t;

typedef void (*esp_hidd_event_cb_t)(esp_hidd_cb_event_t event, esp_hidd_cb_param_t* param);

typedef struct HIDReportMapping {
  uint16_t handle;      // Handle of report characteristic
  uint16_t cccdHandle;  // Handle of CCCD for report characteristic
  uint8_t id;           // Report ID
  uint8_t type;         // Report type
  uint8_t mode;         // Protocol mode (report or boot)
} hid_report_map_t;

typedef struct HIDDeviceConfiguration {
  uint32_t idleTimeout;  // Idle timeout in milliseconds
  uint8_t hidFlags;      // HID feature flags

} hid_dev_cfg_t;

static hid_report_map_t* hid_get_report_by_id(uint8_t id, uint8_t type);

void hid_dev_register_reports(uint8_t num_reports, hid_report_map_t* p_report);

void hid_dev_send_report(esp_gatt_if_t gatts_if, uint16_t conn_id, uint8_t id, uint8_t type, uint8_t length,
                         uint8_t* data);

void hid_consumer_build_report(uint8_t* buffer, consumer_cmd cmd);

void hid_keyboard_build_report(uint8_t* buffer, keyboard_cmd cmd);

void hid_send_consumer_value(uint16_t conn_id, uint8_t key_cmd, bool key_pressed);

void hid_send_keyboard_value(uint16_t conn_id, key_mask special_key_mask, keyboard_cmd* keyboard_cmd, uint8_t num_key);

void hid_device_register_callbacks(esp_hidd_event_cb_t callbacks);

void hid_device_profile_init(void);

void hid_device_profile_deinit(void);

// TODO: possible deprecate
void hid_send_mouse_value(uint16_t conn_id, uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y);

#endif /* HID_DEV_H__ */
