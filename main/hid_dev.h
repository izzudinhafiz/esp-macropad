#ifndef HID_DEV_H__
#define HID_DEV_H__

#include "esp_bt_defs.h"
#include "esp_err.h"
#include "esp_gatt_defs.h"
#include "hid_keydefinition.h"
#include "hidd_le_prf_int.h"

void hid_dev_register_reports(uint8_t num_reports, HIDReportMapping* p_report);

void hid_dev_send_report(esp_gatt_if_t gatts_if, uint16_t conn_id, uint8_t id, uint8_t type, uint8_t length,
                         uint8_t* data);

void hid_consumer_build_report(uint8_t* buffer, consumer_cmd cmd);

void hid_keyboard_build_report(uint8_t* buffer, keyboard_cmd cmd);

void hid_send_consumer_value(uint16_t conn_id, uint8_t key_cmd, bool key_pressed);

void hid_send_keyboard_value(uint16_t conn_id, key_mask special_key_mask, keyboard_cmd* keyboard_cmd, uint8_t num_key);

void hid_device_register_callbacks(HIDEventCallback callbacks);

void hid_device_profile_init(void);

void hid_device_profile_deinit(void);

// TODO: possible deprecate
void hid_send_mouse_value(uint16_t conn_id, uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y);

#endif /* HID_DEV_H__ */
