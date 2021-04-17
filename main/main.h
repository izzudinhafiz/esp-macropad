#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
// #include "esp_wifi.h"
#include <esp32/rom/ets_sys.h>

#include "btconfig.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_bt.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "rotary_encoder.h"

// GPIO Defines
#define PIN_COL0 5
#define PIN_COL1 16
#define PIN_COL2 4
#define PIN_ROW0 19
#define PIN_ROW1 21
#define PIN_ROW2 22
#define PIN_ROT_A 27
#define PIN_ROT_B 14
#define PIN_ROT_SW 23
#define PIN_BATTSENSE 32  // A1_4
#define PIN_5VDET 33      // A1_5
#define PIN_COL_MASK ((1ULL << PIN_COL0) | (1ULL << PIN_COL1) | (1ULL << PIN_COL2))
#define PIN_ROW_MASK ((1ULL << PIN_ROW0) | (1ULL << PIN_ROW1) | (1ULL << PIN_ROW2))

// ADC Vref calibration = 1121mV
#define ADC_CONST 0.0025f  // 1/(3308 - 2914) 3308 is ADC value at 4.2V, 2914 is ADC value at 3.7V
#define ADC_37V 2914

// UART Defines
#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM (2)  // UART pattern;
#define DATA_LENGTH (3)
#define PAYLOAD_LENGTH (PATTERN_CHR_NUM + DATA_LENGTH)
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

// Intermcu Comm Defines
#define HOST_USB_CONN 0x01
#define HOST_USB_DISCONN 0x02
#define TEST_MESSAGE 0x03
#define ACK_REQ 0x04
#define KB_MODE 0x05
#define BATT_UPDATE 0x06
#define ROT_SW_UPDATE 0x07
#define ROT_POS_POSITIVE 0x08
#define ROT_POS_NEGATIVE 0x09
#define IMCU_ACK 0xFF

// Internal State Defines
#define VOL_UP 1
#define VOL_DOWN -1
#define VOL_NONE 0
#define KB_USB 1
#define KB_BT 0

static const char* TAG = "HWIN";
static const char* UARTTAG = "UART";
static const char* PATT = "+";

int keyboard_mode = 0;
int current_kb_mode = 0;
rotary_encoder_t* encoder = NULL;
static uint32_t pcnt_unit = 0;
QueueHandle_t uart_queue;

void hidd_event_callback(HIDEvent event, HIDCallbackParameters* param);
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
void encoder_task(void* pvParamaters);
void battery_task(void* pvParamaters);
void keyboard_task(void* pvParamaters);
void kbmode_task(void* pvParamaters);
void uart_event_task(void* pvParamaters);

void txInterMcu(uint8_t command, uint8_t data) {
  uint8_t commandBuffer[PAYLOAD_LENGTH] = {0x2b, 0x2b, 0x00, 0x00};
  commandBuffer[PATTERN_CHR_NUM] = command;
  commandBuffer[PATTERN_CHR_NUM + 1] = data;
  uart_write_bytes(EX_UART_NUM, commandBuffer, PAYLOAD_LENGTH);
}

void hardwareInit(void) {
  ESP_LOGI(TAG, "Hardware initializing");
  keyboard_mode = KB_BT;

  gpio_config_t col_config;
  col_config.intr_type = GPIO_INTR_DISABLE;
  col_config.mode = GPIO_MODE_OUTPUT;
  col_config.pin_bit_mask = PIN_COL_MASK;
  col_config.pull_down_en = 0;
  col_config.pull_up_en = 0;
  gpio_config(&col_config);
  current_kb_mode = KB_BT;

  gpio_config_t row_config;
  row_config.intr_type = GPIO_INTR_DISABLE;
  row_config.mode = GPIO_MODE_INPUT;
  row_config.pin_bit_mask = PIN_ROW_MASK;
  row_config.pull_down_en = 0;
  row_config.pull_up_en = 0;
  gpio_config(&row_config);

  gpio_config_t det5v_config;
  det5v_config.mode = GPIO_MODE_INPUT;
  det5v_config.pin_bit_mask = (1ULL << PIN_5VDET);
  det5v_config.intr_type = GPIO_INTR_DISABLE;
  det5v_config.pull_down_en = 0;
  det5v_config.pull_up_en = 0;
  gpio_config(&det5v_config);

  gpio_config_t rot_sw_config;
  rot_sw_config.mode = GPIO_MODE_INPUT;
  rot_sw_config.pin_bit_mask = (1ULL << PIN_ROT_SW);
  rot_sw_config.intr_type = GPIO_INTR_DISABLE;
  rot_sw_config.pull_down_en = 0;
  rot_sw_config.pull_up_en = 0;
  gpio_config(&rot_sw_config);
  ESP_LOGI(TAG, "GPIO Initialized");

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(
      ADC1_CHANNEL_4,
      ADC_ATTEN_DB_11);  // Full scale 2.6V (Input scaling=0.718) need to change to input scaling 0.5 (11K + 11K)
  ESP_LOGI(TAG, "ADC1_4 Initialized");

  // Create rotary encoder instance
  rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, PIN_ROT_A, PIN_ROT_B);
  ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));
  ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));
  ESP_ERROR_CHECK(encoder->start(encoder));
  ESP_LOGI(TAG, "Rot. Enc. Initialized");
}

float getBatteryVoltage(void) {
  uint16_t batteryRaw = adc1_get_raw(ADC1_CHANNEL_4);
  float batteryVoltage = ((batteryRaw - ADC_37V) * ADC_CONST) * (0.282) + 3.7;

  return batteryVoltage;
}

int scanButtons(void) {
  uint16_t ButtonStatus = 0;
  gpio_set_level(PIN_COL0, 1);
  // ets_delay_us(100);
  if (gpio_get_level(PIN_ROW0)) {
    ButtonStatus |= (1 << 1);
  }
  if (gpio_get_level(PIN_ROW1)) {
    ButtonStatus |= (1 << 2);
  }
  if (gpio_get_level(PIN_ROW2)) {
    ButtonStatus |= (1 << 2);
  }
  gpio_set_level(PIN_COL0, 0);

  gpio_set_level(PIN_COL1, 1);
  // ets_delay_us(100);
  if (gpio_get_level(PIN_ROW0)) {
    ButtonStatus |= (1 << 4);
  }
  if (gpio_get_level(PIN_ROW1)) {
    ButtonStatus |= (1 << 5);
  }
  if (gpio_get_level(PIN_ROW2)) {
    ButtonStatus |= (1 << 6);
  }
  gpio_set_level(PIN_COL1, 0);

  gpio_set_level(PIN_COL2, 1);
  // ets_delay_us(100);
  if (gpio_get_level(PIN_ROW0)) {
    ButtonStatus |= (1 << 7);
  }
  if (gpio_get_level(PIN_ROW1)) {
    ButtonStatus |= (1 << 8);
  }
  if (gpio_get_level(PIN_ROW2)) {
    ButtonStatus |= (1 << 9);
  }
  gpio_set_level(PIN_COL2, 0);

  if (gpio_get_level(PIN_ROT_SW)) {
    ButtonStatus |= (1 << 10);
  }

  return ButtonStatus;
}

void initUart(void) {
  // See UART ISR timeout issues with pattern detect
  // https://github.com/espressif/esp-idf/issues/4707
  uart_config_t uart_config = {
      .baud_rate = 38400,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  ESP_ERROR_CHECK(uart_param_config(EX_UART_NUM, &uart_config));
  ESP_ERROR_CHECK(
      uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  ESP_ERROR_CHECK(uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0));
  ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(EX_UART_NUM, *PATT, PATTERN_CHR_NUM, 9, 0, 0));
  ESP_ERROR_CHECK(uart_pattern_queue_reset(EX_UART_NUM, BUF_SIZE / 8));
  ESP_LOGI(TAG, "UART Initialized");
}

void initBT(void) {
  esp_err_t ret;

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed\n", __func__);
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed\n", __func__);
    return;
  }

  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
    return;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(HID_DEMO_TAG, "%s enable bluedroid failed\n", __func__);
    return;
  }
}

void initHID(void) {
  hid_device_profile_init();

  /// register the callback function to the gap module
  esp_ble_gap_register_callback(gap_event_handler);
  hid_device_register_callbacks(hidd_event_callback);

  /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;  // bonding with peer device after authentication
  esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;        // set the IO capability to No output No input
  uint8_t key_size = 16;                           // the key size should be 7~16 bytes
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute
to you, and the response key means which key you can distribute to the Master; If your BLE device act as a master, the
response key means you hope which types of key of the slave should distribute to you, and the init key means which key
you can distribute to the slave. */
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}

void handleComms(uint8_t* cmdBuffer) {
  switch (cmdBuffer[0]) {
    case ACK_REQ:
      txInterMcu(IMCU_ACK, 0);
      break;
    case HOST_USB_CONN:
      break;
    case HOST_USB_DISCONN:
      break;
    case KB_MODE:
      if (cmdBuffer[1] <= 1) keyboard_mode = cmdBuffer[1];
      break;
    case TEST_MESSAGE:
      break;
    default:
      break;
  }
}
