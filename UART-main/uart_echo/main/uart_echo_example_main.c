/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */


#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)


#define UART_BAUD_RATE 115200            // Baud rate for communication
#define TASK_STACK_SIZE (1024)             // Buffer size for UART communication

#define OUTPUT_TXD 1
#define OUTPUT_RXD 3
#define OUTPUT_UART_PORT_NUM UART_NUM_0 

#define INPUT_TXD 17
#define INPUT_RXD 16
#define INPUT_UART_PORT_NUM UART_NUM_2 


static const char *TAG = "UART TEST";

#define BUF_SIZE (1024)

static void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(OUTPUT_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(OUTPUT_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(OUTPUT_UART_PORT_NUM, OUTPUT_TXD, OUTPUT_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    ESP_ERROR_CHECK(uart_driver_install(INPUT_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(INPUT_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(INPUT_UART_PORT_NUM, INPUT_TXD, INPUT_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {

        // Read Data from LoRa Device
        int len = uart_read_bytes(INPUT_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);

        // char* data = "hello";
        // int len = 5;  // Length of "hello"


        // sprintf("%s",data);

        // Write data back to the UART for Debugging
        uart_write_bytes(OUTPUT_UART_PORT_NUM, (const char *) data, len);
        if (len) {
            data[len] = '\0';
            ESP_LOGI(TAG, "Recv str: %s", (char *) data);
        }
    vTaskDelay(pdMS_TO_TICKS(1000));     
    }
}



void app_main(void)
{
    xTaskCreate(echo_task, "uart_echo_task", TASK_STACK_SIZE, NULL, 10, NULL);
}
