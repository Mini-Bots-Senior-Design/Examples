/*
This program will run on an ESP32 to control the motors

Developed by Benji Gilbert and Noah Robitshek
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "handlers.h"
#include "utils.h"
#include "globals.h"
#include "main.h"


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
