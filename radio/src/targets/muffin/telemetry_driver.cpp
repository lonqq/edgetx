/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "opentx.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "rom/gpio.h"
#include "esp_log.h"
#include "soc/gpio_sig_map.h"
#define uartIn U2RXD_IN_IDX
#define uartOut U2TXD_OUT_IDX


Fifo<uint8_t, TELEMETRY_FIFO_SIZE> telemetryFifo;
uint32_t telemetryErrors = 0;
uint8_t inputInverted = 1;
static RTOS_MUTEX_HANDLE telemetryMutex;

void telemetryPortInitCommon(uint32_t baudrate, uint8_t mode, uint8_t inverted = 0)
{
    RTOS_LOCK_MUTEX(telemetryMutex);
    ESP_LOGI("Telem", "Init uart with baudrate %d, inverted? %d", baudrate, inverted);

    if(uart_is_driver_installed(EXTTEL_UART_PORT))
    {
      uart_driver_delete(EXTTEL_UART_PORT);
      // wait for removal
      vTaskDelay(200 / portTICK_RATE_MS);
    }
    //inputInverted = inverted;
    uart_config_t uart_config = {
        .baud_rate = (int)baudrate,      //ELRS_INTERNAL_BAUDRATE
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,      
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
  };

  if (mode & TELEMETRY_SERIAL_8E2) {
    uart_config.stop_bits = UART_STOP_BITS_2;
    uart_config.parity = UART_PARITY_EVEN;
  }
  else {
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.parity = UART_PARITY_DISABLE;
  }

  // We won't use a buffer for sending data.
  uart_driver_install(EXTTEL_UART_PORT, TELEMETRY_FIFO_SIZE * 2, 0, 0, NULL, 0);

  uart_param_config(EXTTEL_UART_PORT, &uart_config);
  uart_set_pin(EXTTEL_UART_PORT, EXTTEL_PIN, EXTTEL_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  RTOS_UNLOCK_MUTEX(telemetryMutex);

}

void telemetryPortInit(uint32_t baudrate, uint8_t mode)
{
  telemetryPortInitCommon(baudrate, mode, 0);
}

void telemetryPortInvertedInit(uint32_t baudrate)
{
  telemetryPortInitCommon(baudrate, TELEMETRY_SERIAL_DEFAULT, 1);
}

void telemetryBaseInit()
{
  RTOS_CREATE_MUTEX(telemetryMutex);
  telemetryPortInitCommon(ELRS_INTERNAL_BAUDRATE, TELEMETRY_SERIAL_DEFAULT, 0);
}

void IRAM_ATTR telemetryPortSetDirectionOutput()
{
  // Read all data
  // todo: move it to event/interrupt
  uint8_t data;
  int i = 0;
  int bytes = 0;
  while((bytes = uart_read_bytes(EXTTEL_UART_PORT, &data, 1, 0))){
    telemetryFifo.push(data);
    i++;
  }
  if(bytes < 0)
  {
    ESP_LOGE("Telem", "Error while reading data");
    vTaskDelay(200 / portTICK_RATE_MS);
  }
  if(i >= TELEMETRY_FIFO_SIZE){
    ESP_LOGI("Telem", "Read data %d", i); 

  }


  portDISABLE_INTERRUPTS();

  ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)EXTTEL_PIN, GPIO_FLOATING));
  ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)EXTTEL_PIN, GPIO_FLOATING));
  if (inputInverted)
  {
      ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)EXTTEL_PIN, 0));
      ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)EXTTEL_PIN, GPIO_MODE_OUTPUT));
      constexpr uint8_t MATRIX_DETACH_IN_LOW = 0x30; // routes 0 to matrix slot
      gpio_matrix_in(MATRIX_DETACH_IN_LOW, uartIn, false); // Disconnect RX from all pads
      gpio_matrix_out((gpio_num_t)EXTTEL_PIN, uartOut, true, false);
  }
  else
  {
      ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)EXTTEL_PIN, 1));
      ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)EXTTEL_PIN, GPIO_MODE_OUTPUT));
      constexpr uint8_t MATRIX_DETACH_IN_HIGH = 0x38; // routes 1 to matrix slot
      gpio_matrix_in(MATRIX_DETACH_IN_HIGH, uartIn, false); // Disconnect RX from all pads
      gpio_matrix_out((gpio_num_t)EXTTEL_PIN, uartOut, false, false);
  }

  portENABLE_INTERRUPTS();
}

void sportWaitTransmissionComplete()
{
  while(uart_wait_tx_done(EXTTEL_UART_PORT, 20 / portTICK_RATE_MS));
}

void IRAM_ATTR telemetryPortSetDirectionInput()
{

  sportWaitTransmissionComplete();
  portDISABLE_INTERRUPTS();
  
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)EXTTEL_PIN, GPIO_MODE_INPUT));
        if (inputInverted)
        {
            gpio_matrix_in((gpio_num_t)EXTTEL_PIN, uartIn, true);
            gpio_pulldown_en((gpio_num_t)EXTTEL_PIN);
            gpio_pullup_dis((gpio_num_t)EXTTEL_PIN);
        }
        else
        {
            gpio_matrix_in((gpio_num_t)EXTTEL_PIN, uartIn, false);
            gpio_pullup_en((gpio_num_t)EXTTEL_PIN);
            gpio_pulldown_dis((gpio_num_t)EXTTEL_PIN);
        }

  portENABLE_INTERRUPTS();

}

void sportSendByte(uint8_t byte)
{
  uint8_t data = byte;
  sportSendBuffer(&data, 1);
}

void sportSendBuffer(const uint8_t * buffer, uint32_t count)
{
  telemetryPortSetDirectionOutput();
  uart_write_bytes(EXTTEL_UART_PORT, buffer, count);
  uart_flush(EXTTEL_UART_PORT);
  telemetryPortSetDirectionInput();
}

void check_telemetry_exti()
{
}

// TODO we should have telemetry in an higher layer, functions above should move to a sport_driver.cpp
bool telemetryGetByte(uint8_t * byte)
{
  return false;
}

void telemetryClearFifo()
{
}

bool sportGetByte(uint8_t * byte)
{
  //return uart_read_bytes(EXTTEL_UART_PORT, data, 1, 0);
  return telemetryFifo.pop(*byte);
}


void sportStopSendByteLoop()
{
}

void sportSendByteLoop(uint8_t byte)
{
}
