#include <Wire.h>
#include <LiquidCrystal.h>

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define MAX_MESSAGES 16        // Number of messages to store
String messages[MAX_MESSAGES]; // Circular buffer for messages
int messageIndex = 0;          // Index for the current message

const int ledPin = 13;          // LED connected to pin 13
const int potPin = A0;          // Potentiometer connected to analog pin A0
volatile bool ledOn = false;    // Flag to indicate LED state
int lastPotValue = 0;           // Last potentiometer value

void setup()
{
    Serial.begin(9600);      // Initialize serial communication
    lcd.begin(16, 2);        // Initialize the LCD
    lcd.clear();             // Clear the display
    lcd.print("Waiting..."); // Initial message

    pinMode(ledPin, OUTPUT);         // Set LED pin as output
    attachInterrupt(digitalPinToInterrupt(d7), handleInterrupt, CHANGE); // Attach interrupt to potentiometer
}

void loop()
{
    if (Serial.available() > 0)
    {
        String message = Serial.readStringUntil('\n'); // Read the incoming message

        // Add the message to the circular buffer
        messages[messageIndex] = message;
        messageIndex = (messageIndex + 1) % MAX_MESSAGES; // Update index circularly

        // Concatenate the last 16 messages
        String displayText = "";
        for (int i = 0; i < MAX_MESSAGES; i++)
        {
            int idx = (messageIndex + i) % MAX_MESSAGES; // Compute the circular index
            if (messages[idx].length() > 0)
            { // Skip empty slots
                displayText += messages[idx];
                if (displayText.length() >= 32)
                { // Limit to 32 characters (16x2 LCD)
                    break;
                }
            }
        }

        // Display the text on the LCD
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(displayText.substring(0, 16)); // Display the first line
        lcd.setCursor(0, 1);
        if (displayText.length() > 16)
        {
            lcd.print(displayText.substring(16, 32)); // Display the second line
        }
    }

    // If interrupt flag is set, turn on LED
    if (ledOn)
    {
        digitalWrite(ledPin, HIGH);
    }
    else
    {
        digitalWrite(ledPin, LOW);
    }
}

// Interrupt service routine (ISR) to handle potentiometer or message arrival
void handleInterrupt()
{
    ledOn = true;
    delay(500);  // Small delay to stabilize the LED state
    ledOn = false;
}
