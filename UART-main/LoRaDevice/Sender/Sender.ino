#define TX_PIN 17 // UART TX Pin
#define RX_PIN 16 // UART RX Pin

HardwareSerial MySerial(2); // Use UART2

void setup() {
  // Start communication with UART2 using custom pins
  MySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Start communication with the Serial Monitor (USB Serial)
  Serial.begin(115200); // Standard baud rate for Serial Monitor
}

void loop() {
  const char* message = "Hello from UART2!";

  // Send the message via UART2
  MySerial.println(message);

  // Print the message to Serial Monitor as well
  Serial.println(message);  // This will print to the Serial Monitor

  delay(1000); // 1-second delay
}