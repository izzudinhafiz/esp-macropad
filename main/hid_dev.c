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

// Section is about logic of preparing input reports

#include "hid_dev.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#define HIDD_TAG "HID_DEVICE"

static HIDReportMapping* hid_dev_rpt_tbl;
static uint8_t hid_dev_rpt_tbl_Len;

static HIDReportMapping* hid_get_report_by_id(uint8_t id, uint8_t type) {
  HIDReportMapping* rpt = hid_dev_rpt_tbl;

  for (uint8_t i = hid_dev_rpt_tbl_Len; i > 0; i--, rpt++) {
    if (rpt->id == id && rpt->type == type && rpt->mode == hidProtocolMode) {
      return rpt;
    }
  }

  return NULL;
}

void hid_dev_register_reports(uint8_t num_reports, HIDReportMapping* p_report) {
  hid_dev_rpt_tbl = p_report;
  hid_dev_rpt_tbl_Len = num_reports;
  return;
}

void hid_dev_send_report(esp_gatt_if_t gatts_if, uint16_t conn_id, uint8_t id, uint8_t type, uint8_t length,
                         uint8_t* data) {
  HIDReportMapping* report;
  report = hid_get_report_by_id(id, type);
  // get att handle for report
  if (report != NULL) {
    // if notifications are enabled
    ESP_LOGD(HIDD_TAG, "%s(), send the report, handle = %d", __func__, report->handle);
    esp_ble_gatts_send_indicate(gatts_if, conn_id, report->handle, length, data, false);
  }

  return;
}

void hid_consumer_build_report(uint8_t* buffer, consumer_cmd cmd) {
  if (buffer) {
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
  }
}

void hid_send_consumer_value(uint16_t conn_id, uint8_t key_cmd, bool key_pressed) {
  uint8_t buffer[HID_CC_IN_RPT_LEN] = {0, 0};
  if (key_pressed) {
    hid_consumer_build_report(buffer, key_cmd);
  }
  hid_dev_send_report(hid_engine.gatt_if, conn_id, HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT, HID_CC_IN_RPT_LEN, buffer);
  return;
}

void hid_send_keyboard_value(uint16_t conn_id, key_mask special_key_mask, keyboard_cmd* keyboard_cmd, uint8_t num_key) {
  if (num_key > HID_KEYBOARD_IN_RPT_LEN - 2) {
    ESP_LOGE(HIDD_TAG, "%s(), the number key should not be more than %d", __func__, HID_KEYBOARD_IN_RPT_LEN);
    return;
  }

  uint8_t buffer[HID_KEYBOARD_IN_RPT_LEN] = {0};

  buffer[0] = special_key_mask;

  for (int i = 0; i < num_key; i++) {
    buffer[i + 2] = keyboard_cmd[i];
  }

  ESP_LOGD(HIDD_TAG, "the key vaule = %d,%d,%d, %d, %d, %d,%d, %d", buffer[0], buffer[1], buffer[2], buffer[3],
           buffer[4], buffer[5], buffer[6], buffer[7]);
  hid_dev_send_report(hid_engine.gatt_if, conn_id, HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT, HID_KEYBOARD_IN_RPT_LEN,
                      buffer);
  return;
}

void hid_device_profile_init(void) {
  if (hid_engine.enabled) return;

  // Reset the hid device target environment
  memset(&hid_engine, 0, sizeof(HIDServiceEngine));
  hid_engine.enabled = true;
}

void hid_device_profile_deinit(void) {
  uint16_t hidd_svc_hdl = hid_engine.hidd_inst.att_tbl[HIDD_LE_IDX_SVC];
  if (!hid_engine.enabled) return;

  if (hidd_svc_hdl != 0) {
    esp_ble_gatts_stop_service(hidd_svc_hdl);
    esp_ble_gatts_delete_service(hidd_svc_hdl);
  } else
    return;

  /* unregister the HID device profile to the BTA_GATTS module*/
  esp_ble_gatts_app_unregister(hid_engine.gatt_if);
}

// TODO: possible deprecate
void hid_send_mouse_value(uint16_t conn_id, uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y) {
  uint8_t buffer[HID_MOUSE_IN_RPT_LEN];

  buffer[0] = mouse_button;  // Buttons
  buffer[1] = mickeys_x;     // X
  buffer[2] = mickeys_y;     // Y
  buffer[3] = 0;             // Wheel
  buffer[4] = 0;             // AC Pan

  hid_dev_send_report(hid_engine.gatt_if, conn_id, HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT, HID_MOUSE_IN_RPT_LEN,
                      buffer);
  return;
}