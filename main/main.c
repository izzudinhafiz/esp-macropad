#include "main.h"

void app_main(void) {
  esp_err_t ret;

  // Initialize NVS.
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  hardwareInit();  // Sets hardware GPIO
  initUart();      // Configure UART task

  initBT();   // Sets BT controller
  initHID();  // Register HID + GAP protocol callbacks
  xTaskCreate(&uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
  xTaskCreate(&keyboard_task, "keyboard_task", 2048, NULL, 5, NULL);
  xTaskCreate(&encoder_task, "encoder_task", 2048, NULL, 5, NULL);
  xTaskCreate(&battery_task, "battery_task", 2048, NULL, 5, NULL);
  xTaskCreate(&kbmode_task, "kbmode_task", 2048, NULL, 6, NULL);
}

void encoder_task(void* pvParamaters) {
  int8_t vol_mode = VOL_NONE;
  uint8_t counter_difference = 0;
  uint16_t current_counter;
  while (1) {
    if (CONFIG_LOG_DEFAULT_LEVEL == 0) {
      txInterMcu(ROT_SW_UPDATE, gpio_get_level(PIN_ROT_SW));
    }
    current_counter = encoder->get_counter_value(encoder);
    // ESP_LOGI(TAG, "Encoder value: %d", current_counter);
    if (current_counter == last_counter) {
      last_counter = current_counter;
      counter_difference = 0;
      if (CONFIG_LOG_DEFAULT_LEVEL == 0) {
        txInterMcu(ROT_POS_POSITIVE, counter_difference);
      }
      if (vol_mode != VOL_NONE) {
        if (vol_mode == VOL_UP) {
          // release volup
          if (sec_conn && (current_kb_mode == KB_BT))
            hid_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, false);
        } else {
          // release voldown
          if (sec_conn && (current_kb_mode == KB_BT))
            hid_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);
        }
        vol_mode = VOL_NONE;
      }
    } else if (current_counter > last_counter) {
      counter_difference = current_counter - last_counter;
      last_counter = current_counter;
      if (CONFIG_LOG_DEFAULT_LEVEL == 0) {
        txInterMcu(ROT_POS_POSITIVE, counter_difference);
      }
      // increase volume
      if (vol_mode != VOL_UP) {
        if (vol_mode == VOL_NONE) {
          // press volup
          if (sec_conn && (current_kb_mode == KB_BT))
            hid_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, true);
        } else {
          if (sec_conn && (current_kb_mode == KB_BT)) {
            // release voldown
            hid_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);
            // press volup
            hid_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, true);
          }
        }
        vol_mode = VOL_UP;
      }
    } else {
      counter_difference = last_counter - current_counter;
      last_counter = current_counter;
      if (CONFIG_LOG_DEFAULT_LEVEL == 0) {
        txInterMcu(ROT_POS_NEGATIVE, counter_difference);
      }
      // decrease volume
      if (vol_mode != VOL_DOWN) {
        if (vol_mode == VOL_NONE) {
          if (sec_conn && (current_kb_mode == KB_BT))
            hid_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, true);
        } else {
          if (sec_conn && (current_kb_mode == KB_BT)) {
            hid_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, false);
            hid_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, true);
          }
        }
        vol_mode = VOL_DOWN;
      }
    }
    if (counter_difference < 10) {
      counter_difference = 10;
    }

    vTaskDelay(pdMS_TO_TICKS(counter_difference));
  }
}

void battery_task(void* pvParameters) {
  uint8_t scaledBatteryVoltage = 0;
  while (1) {
    if (gpio_get_level(PIN_5VDET)) {
      ESP_LOGV(TAG, "5V Present");
    } else {
      ESP_LOGV(TAG, "5V Not Present");
    }

    ESP_LOGV(TAG, "Battery value: %f", getBatteryVoltage());
    scaledBatteryVoltage = ((int)(getBatteryVoltage() * 100)) / 2;
    if (CONFIG_LOG_DEFAULT_LEVEL == 0) {
      txInterMcu(BATT_UPDATE, scaledBatteryVoltage);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void keyboard_task(void* pvParameters) {
  uint16_t buttonStatus;
  uint8_t numKeysPressed;
  while (1) {
    ESP_LOGV(BTCONFIG_TAG, "Secure Connection is: x%02X", sec_conn);
    if (sec_conn && (current_kb_mode == KB_BT)) {
      keyboard_cmd key_values[10];
      numKeysPressed = 0;

      buttonStatus = scanButtons();
      if (buttonStatus & (1 << 1)) {
        ESP_LOGI(BTCONFIG_TAG, "Button 1 Pressed");
        key_values[numKeysPressed++] = HID_KEY_1;
      }
      if (buttonStatus & (1 << 2)) {
        ESP_LOGI(BTCONFIG_TAG, "Button 2 Pressed");
        key_values[numKeysPressed++] = HID_KEY_2;
      }
      if (buttonStatus & (1 << 3)) {
        ESP_LOGI(BTCONFIG_TAG, "Button 3 Pressed");
        key_values[numKeysPressed++] = HID_KEY_3;
      }
      if (buttonStatus & (1 << 4)) {
        ESP_LOGI(BTCONFIG_TAG, "Button 4 Pressed");
        key_values[numKeysPressed++] = HID_KEY_4;
      }
      if (buttonStatus & (1 << 5)) {
        ESP_LOGI(BTCONFIG_TAG, "Button 5 Pressed");
        key_values[numKeysPressed++] = HID_KEY_5;
      }
      if (buttonStatus & (1 << 6)) {
        ESP_LOGI(BTCONFIG_TAG, "Button 6 Pressed");
        key_values[numKeysPressed++] = HID_KEY_6;
      }
      if (buttonStatus & (1 << 7)) {
        ESP_LOGI(BTCONFIG_TAG, "Button 7 Pressed");
        key_values[numKeysPressed++] = HID_KEY_7;
      }
      if (buttonStatus & (1 << 8)) {
        ESP_LOGI(BTCONFIG_TAG, "Button 8 Pressed");
        key_values[numKeysPressed++] = HID_KEY_8;
      }
      if (buttonStatus & (1 << 9)) {
        ESP_LOGI(BTCONFIG_TAG, "Button 9 Pressed");
        key_values[numKeysPressed++] = HID_KEY_9;
      }
      if (buttonStatus & (1 << 10)) {
        if (!(buttonToggleMask & (1 << 10))) {
          ESP_LOGI(BTCONFIG_TAG, "Button 10 Pressed");
          hid_send_consumer_value(hid_conn_id, HID_CONSUMER_MUTE, true);
          buttonToggleMask |= (1 << 10);
        }
      } else {
        if ((buttonToggleMask & (1 << 10))) {
          ESP_LOGI(BTCONFIG_TAG, "Button 10 Unpressed");
          hid_send_consumer_value(hid_conn_id, HID_CONSUMER_MUTE, false);
          buttonToggleMask &= ~(1 << 10);
        }
      }

      if (numKeysPressed > 6) {
        numKeysPressed = 6;
      }
      hid_send_keyboard_value(hid_conn_id, 0, key_values, numKeysPressed);

      // esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, true);
      // vTaskDelay(3000 / portTICK_PERIOD_MS);
      // esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void kbmode_task(void* pvParamaters) {
  // Handle changes to Keyboard Mode
  // In BT mode, ESP does the key scanning
  // In USB mode, release resources and set COL pins to high impedence so ATMEGA can scan
  while (1) {
    if (current_kb_mode != keyboard_mode) {
      if (keyboard_mode == KB_BT) {
        ESP_LOGI(TAG, "Changing KB MODE to Bluetooth");
        gpio_config_t col_config;
        col_config.intr_type = GPIO_INTR_DISABLE;
        col_config.mode = GPIO_MODE_OUTPUT;
        col_config.pin_bit_mask = PIN_COL_MASK;
        col_config.pull_down_en = 0;
        col_config.pull_up_en = 0;
        gpio_config(&col_config);
        current_kb_mode = KB_BT;
      }

      if (keyboard_mode == KB_USB) {
        ESP_LOGI(TAG, "Changing KB MODE to USB");
        gpio_config_t col_config;
        col_config.intr_type = GPIO_INTR_DISABLE;
        col_config.mode = GPIO_MODE_INPUT;
        col_config.pin_bit_mask = PIN_COL_MASK;
        col_config.pull_down_en = 0;
        col_config.pull_up_en = 0;
        gpio_config(&col_config);
        current_kb_mode = KB_USB;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void uart_event_task(void* pvParamaters) {
  uart_event_t event;
  size_t buffered_size;
  uint8_t* dtmp = (uint8_t*)malloc(RD_BUF_SIZE);
  while (1) {
    if (xQueueReceive(uart_queue, (void*)&event, (portTickType)portMAX_DELAY)) {
      bzero(dtmp, RD_BUF_SIZE);
      ESP_LOGI(UARTTAG, "UART[%d] event:", EX_UART_NUM);
      switch (event.type) {
        case UART_DATA:
          uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
          ESP_LOGI(UARTTAG, "std data: %s", dtmp);
          break;
        case UART_FIFO_OVF:
          ESP_LOGE(UARTTAG, "hw fifo overflow");
          uart_flush_input(EX_UART_NUM);
          xQueueReset(uart_queue);
          break;
        case UART_BUFFER_FULL:
          ESP_LOGE(UARTTAG, "ring buffer full");
          uart_flush_input(EX_UART_NUM);
          xQueueReset(uart_queue);
          break;
        case UART_PARITY_ERR:
          ESP_LOGE(UARTTAG, "uart parity error");
          break;
        case UART_FRAME_ERR:
          ESP_LOGE(UARTTAG, "uart frame error");
          break;
        case UART_PATTERN_DET:
          uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
          int pos = uart_pattern_pop_pos(EX_UART_NUM);

          ESP_LOGI(UARTTAG, "[UART PATTERN] pos: %d, bufsize: %d", pos, buffered_size);
          if (pos == -1) {
            ESP_LOGE(UARTTAG, "UART Pattern Detect Buffer Full");
            uart_flush_input(EX_UART_NUM);
          } else {
            uint8_t payloadBuffer[PAYLOAD_LENGTH];
            memset(payloadBuffer, 0, sizeof(payloadBuffer));
            int len = pos + PATTERN_CHR_NUM;
            if (len > (PAYLOAD_LENGTH)) {
              uart_read_bytes(EX_UART_NUM, dtmp, (len - PAYLOAD_LENGTH), pdMS_TO_TICKS(100));
              uart_read_bytes(EX_UART_NUM, payloadBuffer, len, pdMS_TO_TICKS(100));
            } else if (len == (PAYLOAD_LENGTH)) {
              uart_read_bytes(EX_UART_NUM, payloadBuffer, len, pdMS_TO_TICKS(100));
            } else {
              uart_read_bytes(EX_UART_NUM, dtmp, len, pdMS_TO_TICKS(100));
            }
            ESP_LOGI(UARTTAG, "C:0x%02X D:0x%02X", payloadBuffer[0], payloadBuffer[1]);
            handleComms(payloadBuffer);
          }
          break;
        default:
          ESP_LOGI(UARTTAG, "uart event type: %d", event.type);
          break;
      }
    }
  }
  free(dtmp);
  dtmp = NULL;
  vTaskDelete(NULL);
}