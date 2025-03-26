#include "mavlink.h"

#define SERIAL_BAUD 57600  // Adjust based on your FC's baud rate

// Function to send an arm command
void sendArmCommand() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // MAV_CMD_COMPONENT_ARM_DISARM, param1 = 1 (arm)
    mavlink_msg_command_long_pack(
        1, 200,  // System ID and Component ID
        &msg,
        1, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
}

// Function to set RC channel PWM
void setRCChannelPWM(uint8_t channel, uint16_t pwm) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_rc_channels_override_pack(
        1, 200,  // System ID and Component ID
        &msg,
        1, 0,  // Target System and Component ID
        (channel == 1) ? pwm : UINT16_MAX,
        UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
}

void setup() {
    Serial1.begin(SERIAL_BAUD);  // Change to match FC's telemetry baud rate
    delay(5000);  // Give time to establish connection

    sendArmCommand();  // Send "arm throttle"
    delay(1000);       // Wait for FC to process

    setRCChannelPWM(1, 1500);  // Send "rc 1 1500"
}

void loop() {
    // No need to loop for this simple command sequence
}