This is a repo for Uart 

Run the LoraDevice on the LoRa Device and uart_echo on the esp32.

This program will send a message from the lora device to it's Uart port every 1 second.

The uart_echo program will listen to these messages and write these messages to a different uart port (which the serial prompt can read). 

Communication between Lora and Motor Controller and back:

Setup:
LoRA Device
- Arduino C
- LoraDevice

Motor Control
- ESP-IDF
- uart_echo



