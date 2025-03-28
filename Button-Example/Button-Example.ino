const int buttonPin = 13; // Button connected to digital pin 2
int buttonState = 0;     // Variable to store button state

void setup() {
    pinMode(buttonPin, INPUT_PULLUP); // Set pin as input with internal pull-up resistor
    Serial.begin(9600);               // Start serial communication
}

void loop() {
    buttonState = digitalRead(buttonPin); // Read button state

    if (buttonState == LOW) { // Button is pressed (LOW due to pull-up)
        Serial.println("Button Pressed");
    } else {
        Serial.println("Button Released");
    }

    delay(200); // Debounce delay
}
