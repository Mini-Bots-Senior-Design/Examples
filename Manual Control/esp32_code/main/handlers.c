#include "handlers.h"
#include "globals.h"
#include "esp_log.h"
#include <stdio.h>



// Handle Functions
void handle_MOV(){
    switch (*botDirection) {
        case 'R':
            ESP_LOGI(TAG_DIRECTION, "%s", "Moving the Bot right");
            break;
        case 'L':
            ESP_LOGI(TAG_DIRECTION, "%s", "Moving the Bot left");
            break;
        case 'F':
            ESP_LOGI(TAG_DIRECTION, "%s", "Moving the Bot forward");
            break;
        case 'S':
            ESP_LOGI(TAG_DIRECTION, "%s", "Stopping the Bot");
            break;
        case 'B':
            ESP_LOGI(TAG_DIRECTION, "%s", "Reversing the Bot");
            break;
        default:
            printf("ERROR, Invalid Command\n");
            break;
    }
}

void handle_MOV_GPS(){
    printf("Handle MOV GPS\n");
}

void handle_MOV_PWM(){
    printf("Handle MOV PWM\n");
}

