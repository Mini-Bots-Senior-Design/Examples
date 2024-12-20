#include "handlers.h"
#include "esp_log.h"
#include <stdio.h>


// Handle Functions
void handle_MOV(){
    switch (*botDirection) {
        case 'd':
            ESP_LOGI(TAG_DIRECTION, "%s", "Moving the Bot right");
            break;
        case 'a':
            ESP_LOGI(TAG_DIRECTION, "%s", "Moving the Bot left");
            break;
        case 'w':
            ESP_LOGI(TAG_DIRECTION, "%s", "Moving the Bot forward");
            break;
        case 'p':
            ESP_LOGI(TAG_DIRECTION, "%s", "Stopping the Bot");
            break;
        case 's':
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

