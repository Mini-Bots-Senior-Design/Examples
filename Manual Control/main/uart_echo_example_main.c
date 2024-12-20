/*
This program will run on an ESP32 to control the motors

Developed by Benji Gilbert and Noah Robitshek
*/


// String Syntax
// MOV
// w,a,s,d and p

// MOVGPS

// MOVPWM


// within move gps, we are going to have some alrogirhtims but we can figure that out later

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "handlers.h"

#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)


#define UART_BAUD_RATE 115200            // Baud rate for communication
#define TASK_STACK_SIZE (1024)             // Buffer size for UART communication

#define INPUT_TXD 1
#define INPUT_RXD 3
#define INPUT_UART_PORT_NUM UART_NUM_0 

const char *TAG_STATE = "STATE";
const char *TAG_DIRECTION = "DIRECTION";


#define BUF_SIZE (1024)


BotState state = STATE_STARTUP;

// Global Variables
char *botDirection = "S";

int leftPWM;
int rightPWM;

int latEX;
int longEX;


// Task
// This task reads the bots direction and moves the bot.
static void state_machine(void *arg) { // maybe cast?
    while(1){
        switch (state) {
            case STATE_STARTUP:
                ESP_LOGI(TAG_STATE, "%s", "The Bot is in Startup Mode");

                break;

            case STATE_MOV:
                ESP_LOGI(TAG_STATE, "%s", "The Bot is in Manual Mode");

                handle_MOV();
                break;

            case STATE_MOVGPS:
                ESP_LOGI(TAG_STATE, "%s", "The Bot is in Automatic Mode");

                // handle_MOV_GPS();
                break;

            case STATE_MOVPWM:
                ESP_LOGI(TAG_STATE, "%s", "The Bot is in PWM Manual Mode");

                // handle_MOV_PWM();
                break;

            default:
                printf("Unknown state.\n");
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(500)); 
    }  
}



void uart_init(){
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(INPUT_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(INPUT_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(INPUT_UART_PORT_NUM, INPUT_TXD, INPUT_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
}

void parse_input_string(){
    printf("This function will parse the string");

    // Parse the Input
    // Change the State of the Bot
    // If there are other commands, update those two

}


// Task
static void read_uart_task(void *arg)
{
    uart_init(); 

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {

        // Read Data from LoRa Device
        int len = uart_read_bytes(INPUT_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);

        // print user input
        if (len) {
            data[len] = '\0';
            botDirection = (char *) data;
        }

        vTaskDelay(pdMS_TO_TICKS(100));     
    }
}





void app_main(void)
{
    // Enable or Disable Logging

    // esp_log_level_set(TAG_STATE, ESP_LOG_WARN);
    // esp_log_level_set(TAG_DIRECTION, ESP_LOG_WARN);
    // esp_log_level_set(TAG_STATE, ESP_LOG_WARN);


    // This task listens to the uart port and updates the bots direction.
    xTaskCreate(read_uart_task, "read_uart_task", TASK_STACK_SIZE, NULL, 10, NULL);

    // This task reads the bots direction and moves the bot.
    xTaskCreate(state_machine, "state_machine", TASK_STACK_SIZE, NULL, 10, NULL);
}
