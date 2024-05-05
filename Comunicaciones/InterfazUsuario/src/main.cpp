//Usuary interface PMEC Group 2

#include <Arduino.h>
#include <Keypad.h>

//inicialize keypad
const uint8_t ROWS = 4; // rows
const uint8_t COLS = 4; // columns

char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

uint8_t rowPins[ROWS] = { 19, 18, 5, 17 }; // rows pins
uint8_t colPins[COLS] = { 16, 4, 2, 15 }; // columns pins

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

int leds[] = {13, 12, 14, 27, 26}; // Leds pins
int buttonPin = 25; // Pin push button
int emergencyPin = 33; // Pin emergency button
int buttonHumanTurnPin = 32; // Button "Turno Humano"
int buttonRobotWinsPin = 35; // Button "Gana Robot"
int buttonHumanWinsPin = 34; // Button "Gana Humano"

bool lastButtonState = LOW; // Previous state of the pushbutton to detect edges
bool emergencyState = LOW; // State of the emergency stop
bool lastButtonHumanTurnState = LOW; // Previous state of the "human turn" button
bool lastButtonRobotWinsState = LOW; // Previous state of the "robot wins" button
bool lastButtonHumanWinsState = LOW; // Previous state of the "human wins" button
unsigned long lastDebounceTime = 0;  // Timestamp of the last time the output pin state changed
unsigned long debounceDelay = 50;    // Time in milliseconds to stabilize the button's signal

int estadoActual = 0; // Current state 
int ledDificultad = 2; // Difficulty level LED indicator (initial level)
int ledDificultadAux = 0; // Auxiliary variable for the difficulty level LED
int estadoActualAux = 0; // Auxiliary variable for the current state

unsigned long blinkTimes[5] = {0, 0, 0, 0, 0}; // Array to store the last blink times for each LED
String nivel = "facil"; // Difficulty level, initialized to "easy"

void blinkLed(int ledIndex) {
  static bool ledStates[5] = {LOW, LOW, LOW, LOW, LOW}; // Static array to hold the current state (ON/OFF) of each LED
  unsigned long currentTime = millis(); // Get the current time in milliseconds
  if (currentTime - blinkTimes[ledIndex] >= 200) { // Check if 200 milliseconds have passed since the last toggle
    blinkTimes[ledIndex] = currentTime; // Update the last blink time for this LED
    ledStates[ledIndex] = !ledStates[ledIndex]; // Toggle the LED state
    digitalWrite(leds[ledIndex], ledStates[ledIndex]); // Apply the new state to the LED
  }
}

void resetSystem() {
  // Reset state variables
  estadoActual = 0;    // Set the current state to 0
  ledDificultad = 0;   // Reset difficulty LED indicator to 0
  nivel = "facil";     // Set the difficulty level to "easy"
  // You can add more variable resets if necessary
}

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 bits per second (comment mentions alternative speed of 115200)
  estadoActual = 7;    // Initialize the current state to 7
  for (int i = 0; i < 5; i++) {
    pinMode(leds[i], OUTPUT);  // Set each LED pin in the array as an output
  }
  // Set the mode for various button pins as input
  pinMode(buttonPin, INPUT);
  pinMode(emergencyPin, INPUT);
  pinMode(buttonHumanTurnPin, INPUT);
  pinMode(buttonRobotWinsPin, INPUT);
  pinMode(buttonHumanWinsPin, INPUT);
}

bool lastEmergencyState = LOW;  // Store the previous state of the emergency stop


void loop() {
  bool reading = digitalRead(emergencyPin);  // Read the current state of the emergency pin

  // If the state of the pin has changed, reset the debounce timer
  if (reading != lastEmergencyState) {
    lastDebounceTime = millis();
  }

  // If the time since the last change is greater than the debounce delay, process the state
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the state has changed, update the emergency stop state
    if (reading != emergencyState) {
      emergencyState = reading;

      if (emergencyState) {
        Serial.println("EMERGENCY STOP!!!!!");  // Print emergency stop message

        // Turn off all non-critical LEDs
        digitalWrite(leds[1], LOW);
        digitalWrite(leds[2], LOW);
        digitalWrite(leds[3], LOW);
        digitalWrite(leds[4], LOW);
        resetSystem();  // Reset the system when an emergency stop is triggered

        // Optionally, you could also turn off LED 0 if it's used as an emergency indicator
        // digitalWrite(leds[0], LOW); 
      }
    }
  }

  lastEmergencyState = reading;  // Update the last known state

  // Do not execute further code in the loop while the emergency stop is active
  if (emergencyState) {
    blinkLed(0);
    return;
  }

  digitalWrite(leds[0], LOW);
  
  char key = keypad.getKey();
  if (key) {
    for (int i = 0; i < 5; i++) {
      digitalWrite(leds[i], LOW);
    }

    switch (key) {
      case 'A':
        Serial.println("modo automatico");
        digitalWrite(leds[0], LOW); // Apagar el LED de emergencia
        estadoActual = 1;
        //ledDificultadAux = 2;

        nivel = "facil";
        Serial.println("Nivel: facil");
        digitalWrite(leds[1], HIGH);
        digitalWrite(leds[2], HIGH);
        Serial.println("Turno Humano");
        ledDificultad = 0;
        break;
      case 'B':
        Serial.println("Modo Manual");
        digitalWrite(leds[0], LOW); // Apagar el LED de emergencia
        estadoActual = 2;
        break;
      case 'C':
        Serial.println("Modo Calibracion");
        estadoActual = 10;
        break;
      case 'D':
        if (estadoActual == 1) {

          if (ledDificultadAux == 1){
            ledDificultadAux = 1;
            //Serial.println("turno robot PULSO D");

            digitalWrite(leds[3], HIGH);
            digitalWrite(leds[2], LOW);
          }
          if (ledDificultadAux == 0) { 
            ledDificultadAux = 0;
            //Serial.println("Turno Humano PULSO D");

            digitalWrite(leds[2], HIGH);
            digitalWrite(leds[3], LOW);
          }
          digitalWrite(leds[1], HIGH);

          nivel = (nivel == "facil") ? "dificil" : "facil";
          ledDificultad = 0;
          Serial.println("Nivel: " + nivel);
          if (nivel == "facil"){
            ledDificultad = 0;
          }
          if (nivel == "dificil"){
            ledDificultad = 1;
            //digitalWrite(leds[4], HIGH);
          }         
        }
        
        break;
      case '0':
        Serial.println("RESET SYSTEM");
        resetSystem();
        break;
    }
  }
  if (estadoActual == 1){
    bool currentButtonState = digitalRead(buttonPin);
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (currentButtonState != lastButtonState) {
        lastDebounceTime = millis();
        if (currentButtonState == HIGH && lastButtonState == LOW) {
          //if (estadoActual == 1) {
          Serial.println("turno robot");
          ledDificultadAux = 1;
          digitalWrite(leds[3], HIGH);
          digitalWrite(leds[2], LOW);
          //}
        }
        lastButtonState = currentButtonState;
      }
      
    }

    bool currentButtonHumanTurnState = digitalRead(buttonHumanTurnPin);
    if (currentButtonHumanTurnState != lastButtonHumanTurnState) {
      if ((millis() - lastDebounceTime) > debounceDelay) {
        if (currentButtonHumanTurnState == HIGH && lastButtonHumanTurnState == LOW) {
          Serial.println("Turno Humano");
          ledDificultadAux = 0;
          digitalWrite(leds[2], HIGH);
          digitalWrite(leds[3], LOW);
        }
        lastDebounceTime = millis();
      }
      lastButtonHumanTurnState = currentButtonHumanTurnState;
      
    }

    // Leer y manejar botón "Gana Robot"
    bool currentButtonRobotWinsState = digitalRead(buttonRobotWinsPin);
    if (currentButtonRobotWinsState != lastButtonRobotWinsState) {
      if ((millis() - lastDebounceTime) > debounceDelay) {
        if (currentButtonRobotWinsState == HIGH && lastButtonRobotWinsState == LOW) {
          Serial.println("Gana Robot");
          estadoActual = 9;
        }
        lastDebounceTime = millis();
      }
      lastButtonRobotWinsState = currentButtonRobotWinsState;
    }

    // Leer y manejar botón "Gana Humano"
    bool currentButtonHumanWinsState = digitalRead(buttonHumanWinsPin);
    if (currentButtonHumanWinsState != lastButtonHumanWinsState) {
      if ((millis() - lastDebounceTime) > debounceDelay) {
        if (currentButtonHumanWinsState == HIGH && lastButtonHumanWinsState == LOW) {
          Serial.println("Gana Humano");
          estadoActual = 8;
        }
        lastDebounceTime = millis();
      }
      lastButtonHumanWinsState = currentButtonHumanWinsState;
    }
  }
  
  if (estadoActual == 2) {
    if (key >= '1' && key <= '9') {  // Verificar si la tecla es del 1 al 9
      Serial.println(String("Casilla ") + key + " ");
      
    }
    blinkLed(1);
  } 

  if (estadoActual == 1) {
    if (ledDificultad == 0) {
      blinkLed(4);
    } else if (ledDificultad == 1) {
      digitalWrite(leds[4], HIGH);
    }
  }
  if (estadoActual == 10) {
    digitalWrite(leds[0], HIGH);
  } 
  if (estadoActual == 8){
    digitalWrite(leds[0], LOW);
    digitalWrite(leds[1], LOW);
    digitalWrite(leds[3], LOW);
    digitalWrite(leds[4], LOW);
    blinkLed(2);
  } 

  if (estadoActual == 9){
    digitalWrite(leds[0], LOW);
    digitalWrite(leds[1], LOW);
    digitalWrite(leds[2], LOW);
    digitalWrite(leds[4], LOW);
    blinkLed(3);
  }  
}