const int buttonPin = 2; // Pin for the button
unsigned long pressStartTime = 0;
unsigned long lastActionTime = 0; // Track the last button activity time
bool isPressed = false;
bool spacePrinted = false; // Track whether the space has been printed

void setup()
{
    pinMode(buttonPin, INPUT_PULLUP); // Configure the button pin
    Serial.begin(9600);               // Initialize serial communication
}

void loop()
{
    int buttonState = digitalRead(buttonPin);
    unsigned long currentTime = millis();

    if (buttonState == HIGH && !isPressed)
    {
        // Button just pressed
        pressStartTime = currentTime;
        isPressed = true;
        spacePrinted = false; // Reset the space printed flag on activity
    }
    else if (buttonState == LOW && isPressed)
    {
        // Button just released
        unsigned long pressDuration = currentTime - pressStartTime;

        if (pressDuration < 300)
        {
            // Short press if held for less than 300ms
            Serial.print(".");
        }
        else
        {
            // Long press if held for 500ms or more
            Serial.print("-");
        }

        isPressed = false;
        lastActionTime = currentTime; // Update last action time
    }

    // Check for inactivity
    if (!isPressed && (currentTime - lastActionTime > 1000) && !spacePrinted)
    {
        Serial.print(" ");   // Print space after 1000ms of inactivity
        spacePrinted = true; // Ensure space is printed only once
    }

    delay(10); // Debounce delay
}
