#include "driver/gpio.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "hid_dev.h"

#define BTCONFIG_TAG "BT_CONFIG"
#define GAP_TAG "GAP_HANDLER"
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
#define HIDD_DEVICE_NAME "BT HID Macropad"

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static uint16_t buttonToggleMask = 0;
static uint16_t last_counter = 0;
static void hidd_event_callback(HIDCallbackEvent event, HIDEventParameters* param);

static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,  // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010,  // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,    // HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void hidd_event_callback(HIDCallbackEvent event, HIDEventParameters* param) {
  ESP_LOGI(BTCONFIG_TAG, "HID Device Event");
  switch (event) {
    case ESP_HIDD_EVENT_REG_FINISH: {
      ESP_LOGI(BTCONFIG_TAG, "HID Device Event Register Finish");
      if (param->init_finish.state == ESP_HIDD_INIT_OK) {
        // esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
        esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
        esp_ble_gap_config_adv_data(&hidd_adv_data);
      }
      break;
    }
    case ESP_BAT_EVENT_REG: {
      ESP_LOGI(BTCONFIG_TAG, "Battery Registration Event");
      break;
    }
    case ESP_HIDD_EVENT_DEINIT_FINISH:
      ESP_LOGI(BTCONFIG_TAG, "Battery HID Device Deinit Finish Event");
      break;
    case ESP_HIDD_EVENT_BLE_CONNECT: {
      ESP_LOGI(BTCONFIG_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
      hid_conn_id = param->connect.conn_id;
      break;
    }
    case ESP_HIDD_EVENT_BLE_DISCONNECT: {
      sec_conn = false;
      ESP_LOGI(BTCONFIG_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
      esp_ble_gap_start_advertising(&hidd_adv_params);
      break;
    }
    case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
      ESP_LOGI(BTCONFIG_TAG, "ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT");
    }
    default:
      // ESP_LOGI(BTCONFIG_TAG, "HID Device Event Unmanaged x%02X", event);
      break;
  }
  return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  // ESP_LOGI(GAP_TAG, "GAP Event");
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
      ESP_LOGI(GAP_TAG, "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT");
      esp_ble_gap_start_advertising(&hidd_adv_params);
      break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
      ESP_LOGI(GAP_TAG, "ESP_GAP_BLE_SEC_REQ_EVT");
      for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
        ESP_LOGD(GAP_TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
      }
      esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
      ESP_LOGI(GAP_TAG, "ESP_GAP_BLE_AUTH_CMPL_EVT");
      sec_conn = true;
      esp_bd_addr_t bd_addr;
      memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
      ESP_LOGI(GAP_TAG, "remote BD_ADDR: %08x%04x",
               (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
               (bd_addr[4] << 8) + bd_addr[5]);
      ESP_LOGI(GAP_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
      ESP_LOGI(GAP_TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
      if (!param->ble_security.auth_cmpl.success) {
        ESP_LOGE(GAP_TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
      }
      break;
    default:
      ESP_LOGI(GAP_TAG, "GAP Event Unmanaged x%02X", event);
      break;
  }
}
