#include "ble_profile.h"

#include <string.h>

#include "esp_log.h"
#include "hid_keydefinition.h"

#define BLEPRF_TAG "BLE_PROFILE"
struct CharacteristicPresentationInfo {
  uint16_t unit;  // Unit (The Unit is a UUID)
  uint16_t description;
  uint8_t format;
  uint8_t exponent;
  uint8_t name_space;
};

struct GATTSProfileInstance {
  esp_gatts_cb_t gatts_cb;
  uint16_t gatts_if;
  uint16_t app_id;
  uint16_t conn_id;
};

// HID report mapping table
static HIDReportMapping hid_rpt_map[HID_NUM_REPORTS];
HIDServiceEngine hid_engine;

// HID Information characteristic value
static const uint8_t hid_info_characteristic_value[HID_INFORMATION_LEN] = {
    LO_UINT16(0x0111), HI_UINT16(0x0111),  // bcdHID (USB HID version)
    0x00,                                  // bCountryCode
    HID_KBD_FLAGS                          // Flags
};

static uint16_t hidExtReportRefDesc = ESP_GATT_UUID_BATTERY_LEVEL;
static uint8_t hidReportRefMouseIn[HID_REPORT_REF_LEN] = {HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT};
static uint8_t hidReportRefKeyIn[HID_REPORT_REF_LEN] = {HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT};
static uint8_t hidReportRefLedOut[HID_REPORT_REF_LEN] = {HID_RPT_ID_LED_OUT, HID_REPORT_TYPE_OUTPUT};
static uint8_t hidReportRefFeature[HID_REPORT_REF_LEN] = {HID_RPT_ID_FEATURE, HID_REPORT_TYPE_FEATURE};
static uint8_t hidReportRefCCIn[HID_REPORT_REF_LEN] = {HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT};

static uint16_t hid_service_uuid = ATT_SVC_HID;
uint16_t hid_count = 0;
esp_gatts_incl_svc_desc_t incl_svc = {0};

static const uint16_t battery_service_uuid = ESP_GATT_UUID_BATTERY_SERVICE_SVC;
static const uint8_t batery_level_ccc[2] = {0x00, 0x00};
static uint8_t battery_level = 50;

static const esp_gatts_attr_db_t battery_attribute_table[BAS_IDX_NB] = {
    // Battary Service Declaration
    [BAS_IDX_SVC] = {{ESP_GATT_AUTO_RSP},
                     {ESP_UUID_LEN_16, (uint8_t*)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t),
                      sizeof(battery_service_uuid), (uint8_t*)&battery_service_uuid}},

    // Battary level Characteristic Declaration
    [BAS_IDX_BATT_LVL_CHAR] = {{ESP_GATT_AUTO_RSP},
                               {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
                                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read_notify}},

    // Battary level Characteristic Value
    [BAS_IDX_BATT_LVL_VAL] = {{ESP_GATT_AUTO_RSP},
                              {ESP_UUID_LEN_16, (uint8_t*)&battery_level_uuid, ESP_GATT_PERM_READ, sizeof(uint8_t),
                               sizeof(uint8_t), &battery_level}},

    // Battary level Characteristic - Client Characteristic Configuration Descriptor
    [BAS_IDX_BATT_LVL_NTF_CFG] = {{ESP_GATT_AUTO_RSP},
                                  {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(batery_level_ccc),
                                   (uint8_t*)batery_level_ccc}},

    // Battary level report Characteristic Declaration
    [BAS_IDX_BATT_LVL_PRES_FMT] = {{ESP_GATT_AUTO_RSP},
                                   {ESP_UUID_LEN_16, (uint8_t*)&char_format_uuid, ESP_GATT_PERM_READ,
                                    sizeof(struct CharacteristicPresentationInfo), 0, NULL}},
};

static esp_gatts_attr_db_t hidd_attribute_table[HIDD_LE_IDX_NB] = {
    // HID Service Declaration
    [HIDD_LE_IDX_SVC] = {{ESP_GATT_AUTO_RSP},
                         {ESP_UUID_LEN_16, (uint8_t*)&primary_service_uuid, ESP_GATT_PERM_READ_ENCRYPTED,
                          sizeof(uint16_t), sizeof(hid_service_uuid), (uint8_t*)&hid_service_uuid}},

    // HID Service Declaration
    [HIDD_LE_IDX_INCL_SVC] = {{ESP_GATT_AUTO_RSP},
                              {ESP_UUID_LEN_16, (uint8_t*)&include_service_uuid, ESP_GATT_PERM_READ,
                               sizeof(esp_gatts_incl_svc_desc_t), sizeof(esp_gatts_incl_svc_desc_t),
                               (uint8_t*)&incl_svc}},

    // HID Information Characteristic Declaration
    [HIDD_LE_IDX_HID_INFO_CHAR] = {{ESP_GATT_AUTO_RSP},
                                   {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
                                    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read}},
    // HID Information Characteristic Value
    [HIDD_LE_IDX_HID_INFO_VAL] = {{ESP_GATT_AUTO_RSP},
                                  {ESP_UUID_LEN_16, (uint8_t*)&hid_info_char_uuid, ESP_GATT_PERM_READ,
                                   sizeof(HIDInformation), sizeof(hid_info_characteristic_value),
                                   (uint8_t*)&hid_info_characteristic_value}},

    // HID Control Point Characteristic Declaration
    [HIDD_LE_IDX_HID_CTNL_PT_CHAR] = {{ESP_GATT_AUTO_RSP},
                                      {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
                                       CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_write_nr}},
    // HID Control Point Characteristic Value
    [HIDD_LE_IDX_HID_CTNL_PT_VAL] = {{ESP_GATT_AUTO_RSP},
                                     {ESP_UUID_LEN_16, (uint8_t*)&hid_control_point_uuid, ESP_GATT_PERM_WRITE,
                                      sizeof(uint8_t), 0, NULL}},

    // Report Map Characteristic Declaration
    [HIDD_LE_IDX_REPORT_MAP_CHAR] = {{ESP_GATT_AUTO_RSP},
                                     {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
                                      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read}},
    // Report Map Characteristic Value
    [HIDD_LE_IDX_REPORT_MAP_VAL] = {{ESP_GATT_AUTO_RSP},
                                    {ESP_UUID_LEN_16, (uint8_t*)&hid_report_map_uuid, ESP_GATT_PERM_READ,
                                     HIDD_LE_REPORT_MAP_MAX_LEN, sizeof(hidReportMap), (uint8_t*)&hidReportMap}},

    // Report Map Characteristic - External Report Reference Descriptor
    [HIDD_LE_IDX_REPORT_MAP_EXT_REP_REF] = {{ESP_GATT_AUTO_RSP},
                                            {ESP_UUID_LEN_16, (uint8_t*)&hid_repot_map_ext_desc_uuid,
                                             ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(uint16_t),
                                             (uint8_t*)&hidExtReportRefDesc}},

    // Protocol Mode Characteristic Declaration
    [HIDD_LE_IDX_PROTO_MODE_CHAR] = {{ESP_GATT_AUTO_RSP},
                                     {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
                                      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read_write}},
    // Protocol Mode Characteristic Value
    [HIDD_LE_IDX_PROTO_MODE_VAL] = {{ESP_GATT_AUTO_RSP},
                                    {ESP_UUID_LEN_16, (uint8_t*)&hid_proto_mode_uuid,
                                     (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint8_t),
                                     sizeof(hidProtocolMode), (uint8_t*)&hidProtocolMode}},

    [HIDD_LE_IDX_REPORT_MOUSE_IN_CHAR] = {{ESP_GATT_AUTO_RSP},
                                          {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
                                           CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                           (uint8_t*)&char_prop_read_notify}},

    [HIDD_LE_IDX_REPORT_MOUSE_IN_VAL] = {{ESP_GATT_AUTO_RSP},
                                         {ESP_UUID_LEN_16, (uint8_t*)&hid_report_uuid, ESP_GATT_PERM_READ,
                                          HIDD_LE_REPORT_MAX_LEN, 0, NULL}},

    [HIDD_LE_IDX_REPORT_MOUSE_IN_CCC] = {{ESP_GATT_AUTO_RSP},
                                         {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid,
                                          (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},

    [HIDD_LE_IDX_REPORT_MOUSE_REP_REF] = {{ESP_GATT_AUTO_RSP},
                                          {ESP_UUID_LEN_16, (uint8_t*)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ,
                                           sizeof(hidReportRefMouseIn), sizeof(hidReportRefMouseIn),
                                           hidReportRefMouseIn}},
    // Report Characteristic Declaration
    [HIDD_LE_IDX_REPORT_KEY_IN_CHAR] = {{ESP_GATT_AUTO_RSP},
                                        {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
                                         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                         (uint8_t*)&char_prop_read_notify}},
    // Report Characteristic Value
    [HIDD_LE_IDX_REPORT_KEY_IN_VAL] = {{ESP_GATT_AUTO_RSP},
                                       {ESP_UUID_LEN_16, (uint8_t*)&hid_report_uuid, ESP_GATT_PERM_READ,
                                        HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
    // Report KEY INPUT Characteristic - Client Characteristic Configuration
    // Descriptor
    [HIDD_LE_IDX_REPORT_KEY_IN_CCC] = {{ESP_GATT_AUTO_RSP},
                                       {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid,
                                        (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},
    // Report Characteristic - Report Reference Descriptor
    [HIDD_LE_IDX_REPORT_KEY_IN_REP_REF] = {{ESP_GATT_AUTO_RSP},
                                           {ESP_UUID_LEN_16, (uint8_t*)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ,
                                            sizeof(hidReportRefKeyIn), sizeof(hidReportRefKeyIn), hidReportRefKeyIn}},

    // Report Characteristic Declaration
    [HIDD_LE_IDX_REPORT_LED_OUT_CHAR] = {{ESP_GATT_AUTO_RSP},
                                         {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
                                          CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                          (uint8_t*)&char_prop_read_write}},

    [HIDD_LE_IDX_REPORT_LED_OUT_VAL] = {{ESP_GATT_AUTO_RSP},
                                        {ESP_UUID_LEN_16, (uint8_t*)&hid_report_uuid,
                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
    [HIDD_LE_IDX_REPORT_LED_OUT_REP_REF] = {{ESP_GATT_AUTO_RSP},
                                            {ESP_UUID_LEN_16, (uint8_t*)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ,
                                             sizeof(hidReportRefLedOut), sizeof(hidReportRefLedOut),
                                             hidReportRefLedOut}},

    // Report Characteristic Declaration
    [HIDD_LE_IDX_REPORT_CC_IN_CHAR] = {{ESP_GATT_AUTO_RSP},
                                       {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
                                        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                        (uint8_t*)&char_prop_read_notify}},
    // Report Characteristic Value
    [HIDD_LE_IDX_REPORT_CC_IN_VAL] = {{ESP_GATT_AUTO_RSP},
                                      {ESP_UUID_LEN_16, (uint8_t*)&hid_report_uuid, ESP_GATT_PERM_READ,
                                       HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
    // Report KEY INPUT Characteristic - Client Characteristic Configuration
    // Descriptor
    [HIDD_LE_IDX_REPORT_CC_IN_CCC] = {{ESP_GATT_AUTO_RSP},
                                      {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid,
                                       (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED), sizeof(uint16_t), 0,
                                       NULL}},
    // Report Characteristic - Report Reference Descriptor
    [HIDD_LE_IDX_REPORT_CC_IN_REP_REF] = {{ESP_GATT_AUTO_RSP},
                                          {ESP_UUID_LEN_16, (uint8_t*)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ,
                                           sizeof(hidReportRefCCIn), sizeof(hidReportRefCCIn), hidReportRefCCIn}},

    // Boot Keyboard Input Report Characteristic Declaration
    [HIDD_LE_IDX_BOOT_KB_IN_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP},
                                            {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
                                             CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                             (uint8_t*)&char_prop_read_notify}},
    // Boot Keyboard Input Report Characteristic Value
    [HIDD_LE_IDX_BOOT_KB_IN_REPORT_VAL] = {{ESP_GATT_AUTO_RSP},
                                           {ESP_UUID_LEN_16, (uint8_t*)&hid_kb_input_uuid, ESP_GATT_PERM_READ,
                                            HIDD_LE_BOOT_REPORT_MAX_LEN, 0, NULL}},
    // Boot Keyboard Input Report Characteristic - Client Characteristic
    // Configuration Descriptor
    [HIDD_LE_IDX_BOOT_KB_IN_REPORT_NTF_CFG] = {{ESP_GATT_AUTO_RSP},
                                               {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid,
                                                (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0, NULL}},

    // Boot Keyboard Output Report Characteristic Declaration
    [HIDD_LE_IDX_BOOT_KB_OUT_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP},
                                             {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid,
                                              ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                              (uint8_t*)&char_prop_read_write}},
    // Boot Keyboard Output Report Characteristic Value
    [HIDD_LE_IDX_BOOT_KB_OUT_REPORT_VAL] = {{ESP_GATT_AUTO_RSP},
                                            {ESP_UUID_LEN_16, (uint8_t*)&hid_kb_output_uuid,
                                             (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), HIDD_LE_BOOT_REPORT_MAX_LEN, 0,
                                             NULL}},

    // Boot Mouse Input Report Characteristic Declaration
    [HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP},
                                               {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid,
                                                ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                                                (uint8_t*)&char_prop_read_notify}},
    // Boot Mouse Input Report Characteristic Value
    [HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_VAL] = {{ESP_GATT_AUTO_RSP},
                                              {ESP_UUID_LEN_16, (uint8_t*)&hid_mouse_input_uuid, ESP_GATT_PERM_READ,
                                               HIDD_LE_BOOT_REPORT_MAX_LEN, 0, NULL}},
    // Boot Mouse Input Report Characteristic - Client Characteristic
    // Configuration Descriptor
    [HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_NTF_CFG] = {{ESP_GATT_AUTO_RSP},
                                                  {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid,
                                                   (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE), sizeof(uint16_t), 0,
                                                   NULL}},

    // Report Characteristic Declaration
    [HIDD_LE_IDX_REPORT_CHAR] = {{ESP_GATT_AUTO_RSP},
                                 {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
                                  CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read_write}},
    // Report Characteristic Value
    [HIDD_LE_IDX_REPORT_VAL] = {{ESP_GATT_AUTO_RSP},
                                {ESP_UUID_LEN_16, (uint8_t*)&hid_report_uuid, ESP_GATT_PERM_READ,
                                 HIDD_LE_REPORT_MAX_LEN, 0, NULL}},
    // Report Characteristic - Report Reference Descriptor
    [HIDD_LE_IDX_REPORT_REP_REF] = {{ESP_GATT_AUTO_RSP},
                                    {ESP_UUID_LEN_16, (uint8_t*)&hid_report_ref_descr_uuid, ESP_GATT_PERM_READ,
                                     sizeof(hidReportRefFeature), sizeof(hidReportRefFeature), hidReportRefFeature}},
};

static void hid_add_id_tbl(void);

void hid_gatts_callback(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
  switch (event) {
    case ESP_GATTS_REG_EVT: {
      esp_ble_gap_config_local_icon(ESP_BLE_APPEARANCE_GENERIC_HID);
      HIDCallbackParameters hidd_param;
      hidd_param.init_finish.state = param->reg.status;

      if (param->reg.app_id == HIDD_APP_ID) {
        hid_engine.gatt_if = gatts_if;
        if (hid_engine.hidd_cb != NULL) {
          (hid_engine.hidd_cb)(ESP_HIDD_EVENT_REG_FINISH, &hidd_param);
          esp_ble_gatts_create_attr_tab(battery_attribute_table, hid_engine.gatt_if, BAS_IDX_NB, 0);
        }
      }

      if (param->reg.app_id == BATTRAY_APP_ID) {
        hidd_param.init_finish.gatts_if = gatts_if;
        if (hid_engine.hidd_cb != NULL) {
          (hid_engine.hidd_cb)(ESP_BAT_EVENT_REG, &hidd_param);
        }
      }

      break;
    }
    case ESP_GATTS_CONF_EVT: {
      break;
    }
    case ESP_GATTS_CREATE_EVT:
      break;
    case ESP_GATTS_CONNECT_EVT: {
      HIDCallbackParameters cb_param = {0};
      ESP_LOGI(BLEPRF_TAG, "HID connection establish, conn_id = %x", param->connect.conn_id);

      memcpy(cb_param.connect.remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
      cb_param.connect.conn_id = param->connect.conn_id;
      hidd_clcb_alloc(param->connect.conn_id, param->connect.remote_bda);
      esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_NO_MITM);

      if (hid_engine.hidd_cb != NULL) {
        (hid_engine.hidd_cb)(ESP_HIDD_EVENT_BLE_CONNECT, &cb_param);
      }
      break;
    }
    case ESP_GATTS_DISCONNECT_EVT: {
      if (hid_engine.hidd_cb != NULL) {
        (hid_engine.hidd_cb)(ESP_HIDD_EVENT_BLE_DISCONNECT, NULL);
      }
      hidd_clcb_dealloc(param->disconnect.conn_id);
      break;
    }
    case ESP_GATTS_CLOSE_EVT:
      break;
    case ESP_GATTS_WRITE_EVT: {
      break;
    }
    case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
      if (param->add_attr_tab.num_handle == BAS_IDX_NB &&
          param->add_attr_tab.svc_uuid.uuid.uuid16 == ESP_GATT_UUID_BATTERY_SERVICE_SVC &&
          param->add_attr_tab.status == ESP_GATT_OK) {
        incl_svc.start_hdl = param->add_attr_tab.handles[BAS_IDX_SVC];
        incl_svc.end_hdl = incl_svc.start_hdl + BAS_IDX_NB - 1;
        ESP_LOGI(BLEPRF_TAG,
                 "%s(), start added the hid service to the stack database. "
                 "incl_handle = %d",
                 __func__, incl_svc.start_hdl);
        esp_ble_gatts_create_attr_tab(hidd_attribute_table, gatts_if, HIDD_LE_IDX_NB, 0);
      }

      if (param->add_attr_tab.num_handle == HIDD_LE_IDX_NB && param->add_attr_tab.status == ESP_GATT_OK) {
        memcpy(hid_engine.hidd_inst.att_tbl, param->add_attr_tab.handles, HIDD_LE_IDX_NB * sizeof(uint16_t));
        ESP_LOGI(BLEPRF_TAG, "hid svc handle = %x", hid_engine.hidd_inst.att_tbl[HIDD_LE_IDX_SVC]);
        hid_add_id_tbl();
        esp_ble_gatts_start_service(hid_engine.hidd_inst.att_tbl[HIDD_LE_IDX_SVC]);
      } else {
        esp_ble_gatts_start_service(param->add_attr_tab.handles[0]);
      }
      break;
    }

    default:
      break;
  }
}

void hidd_le_init(void) {
  // Reset the hid device target environment
  memset(&hid_engine, 0, sizeof(HIDServiceEngine));
}

void hidd_clcb_alloc(uint16_t conn_id, esp_bd_addr_t bda) {
  uint8_t i_clcb = 0;
  HIDConnectionLink* p_clcb = NULL;

  for (i_clcb = 0, p_clcb = hid_engine.hidd_clcb; i_clcb < HID_MAX_APPS; i_clcb++, p_clcb++) {
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

bool hidd_clcb_dealloc(uint16_t conn_id) {
  uint8_t i_clcb = 0;
  HIDConnectionLink* p_clcb = NULL;

  for (i_clcb = 0, p_clcb = hid_engine.hidd_clcb; i_clcb < HID_MAX_APPS; i_clcb++, p_clcb++) {
    memset(p_clcb, 0, sizeof(HIDConnectionLink));
    return true;
  }

  return false;
}

static struct GATTSProfileInstance gatts_profile_instance[PROFILE_NUM] = {
    [PROFILE_APP_IDX] =
        {
            .gatts_cb = hid_gatts_callback,
            .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        },
};

#define GATT_HANDLER "GATTS_HANDLER"
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
  ESP_LOGI(GATT_HANDLER, "Handling GATTS Event : %d, Interface: x%02X", event, gatts_if);

  if (event == ESP_GATTS_REG_EVT) {
    // Store the gatts_if for each profile
    ESP_LOGI(GATT_HANDLER, "GATTS Register Event");
    if (param->reg.status == ESP_GATT_OK) {
      gatts_profile_instance[PROFILE_APP_IDX].gatts_if = gatts_if;
    } else {
      ESP_LOGI(GATT_HANDLER, "Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
      return;
    }
  }

  do {
    int idx;
    for (idx = 0; idx < PROFILE_NUM; idx++) {
      // No particular gatt_if, need to call every profile cb function
      if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gatts_profile_instance[idx].gatts_if) {
        if (gatts_profile_instance[idx].gatts_cb) {
          gatts_profile_instance[idx].gatts_cb(event, gatts_if, param);
        }
      }
    }
  } while (0);
}

void hidd_set_attr_value(uint16_t handle, uint16_t val_len, const uint8_t* value) {
  HIDInstance* hidd_inst = &hid_engine.hidd_inst;
  if (hidd_inst->att_tbl[HIDD_LE_IDX_HID_INFO_VAL] <= handle &&
      hidd_inst->att_tbl[HIDD_LE_IDX_REPORT_REP_REF] >= handle) {
    esp_ble_gatts_set_attr_value(handle, val_len, value);
  } else {
    ESP_LOGE(BLEPRF_TAG, "%s error:Invalid handle value.", __func__);
  }
  return;
}

void hidd_get_attr_value(uint16_t handle, uint16_t* length, uint8_t** value) {
  HIDInstance* hidd_inst = &hid_engine.hidd_inst;
  if (hidd_inst->att_tbl[HIDD_LE_IDX_HID_INFO_VAL] <= handle &&
      hidd_inst->att_tbl[HIDD_LE_IDX_REPORT_REP_REF] >= handle) {
    esp_ble_gatts_get_attr_value(handle, length, (const uint8_t**)value);
  } else {
    ESP_LOGE(BLEPRF_TAG, "%s error:Invalid handle value.", __func__);
  }

  return;
}

static void hid_add_id_tbl(void) {
  // Mouse input report
  hid_rpt_map[0].id = hidReportRefMouseIn[0];
  hid_rpt_map[0].type = hidReportRefMouseIn[1];
  hid_rpt_map[0].handle = hid_engine.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_MOUSE_IN_VAL];
  hid_rpt_map[0].cccdHandle = hid_engine.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_MOUSE_IN_VAL];
  hid_rpt_map[0].mode = HID_PROTOCOL_MODE_REPORT;

  // Key input report
  hid_rpt_map[1].id = hidReportRefKeyIn[0];
  hid_rpt_map[1].type = hidReportRefKeyIn[1];
  hid_rpt_map[1].handle = hid_engine.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_KEY_IN_VAL];
  hid_rpt_map[1].cccdHandle = hid_engine.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_KEY_IN_CCC];
  hid_rpt_map[1].mode = HID_PROTOCOL_MODE_REPORT;

  // Consumer Control input report
  hid_rpt_map[2].id = hidReportRefCCIn[0];
  hid_rpt_map[2].type = hidReportRefCCIn[1];
  hid_rpt_map[2].handle = hid_engine.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_CC_IN_VAL];
  hid_rpt_map[2].cccdHandle = hid_engine.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_CC_IN_CCC];
  hid_rpt_map[2].mode = HID_PROTOCOL_MODE_REPORT;

  // LED output report
  hid_rpt_map[3].id = hidReportRefLedOut[0];
  hid_rpt_map[3].type = hidReportRefLedOut[1];
  hid_rpt_map[3].handle = hid_engine.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_LED_OUT_VAL];
  hid_rpt_map[3].cccdHandle = 0;
  hid_rpt_map[3].mode = HID_PROTOCOL_MODE_REPORT;

  // Boot keyboard input report
  // Use same ID and type as key input report
  hid_rpt_map[4].id = hidReportRefKeyIn[0];
  hid_rpt_map[4].type = hidReportRefKeyIn[1];
  hid_rpt_map[4].handle = hid_engine.hidd_inst.att_tbl[HIDD_LE_IDX_BOOT_KB_IN_REPORT_VAL];
  hid_rpt_map[4].cccdHandle = 0;
  hid_rpt_map[4].mode = HID_PROTOCOL_MODE_BOOT;

  // Boot keyboard output report
  // Use same ID and type as LED output report
  hid_rpt_map[5].id = hidReportRefLedOut[0];
  hid_rpt_map[5].type = hidReportRefLedOut[1];
  hid_rpt_map[5].handle = hid_engine.hidd_inst.att_tbl[HIDD_LE_IDX_BOOT_KB_OUT_REPORT_VAL];
  hid_rpt_map[5].cccdHandle = 0;
  hid_rpt_map[5].mode = HID_PROTOCOL_MODE_BOOT;

  // Boot mouse input report
  // Use same ID and type as mouse input report
  hid_rpt_map[6].id = hidReportRefMouseIn[0];
  hid_rpt_map[6].type = hidReportRefMouseIn[1];
  hid_rpt_map[6].handle = hid_engine.hidd_inst.att_tbl[HIDD_LE_IDX_BOOT_MOUSE_IN_REPORT_VAL];
  hid_rpt_map[6].cccdHandle = 0;
  hid_rpt_map[6].mode = HID_PROTOCOL_MODE_BOOT;

  // Feature report
  hid_rpt_map[7].id = hidReportRefFeature[0];
  hid_rpt_map[7].type = hidReportRefFeature[1];
  hid_rpt_map[7].handle = hid_engine.hidd_inst.att_tbl[HIDD_LE_IDX_REPORT_VAL];
  hid_rpt_map[7].cccdHandle = 0;
  hid_rpt_map[7].mode = HID_PROTOCOL_MODE_REPORT;

  // Setup report ID map
  hid_dev_register_reports(HID_NUM_REPORTS, hid_rpt_map);
}

void hid_device_register_callbacks(HIDEventCallback callbacks) {
  if (callbacks != NULL)
    hid_engine.hidd_cb = callbacks;
  else
    return;

  esp_ble_gatts_register_callback(gatts_event_handler);
  esp_ble_gatts_app_register(BATTRAY_APP_ID);
  esp_ble_gatts_app_register(HIDD_APP_ID);
}