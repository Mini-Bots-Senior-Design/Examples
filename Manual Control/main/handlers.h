// These files handle 

#ifndef HANDLERS_H
#define HANDLERS_H

// Declare the BotState enum
typedef enum {
    STATE_STARTUP,
    STATE_MOV,
    STATE_MOVGPS,
    STATE_MOVPWM,
} BotState;

extern const char *TAG_STATE;
extern const char *TAG_DIRECTION;

extern BotState state;

// Global Variables
extern char *botDirection;

extern int leftPWM;
extern int rightPWM;

extern int latEX;
extern int longEX;

// Handle Functions
void handle_MOV();
void handle_MOV_GPS();
void handle_MOV_PWM();

#endif // BOT_CONTROL_H