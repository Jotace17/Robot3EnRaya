//System libraries
#include <Arduino.h>
#include <Keypad.h>
//Game libraries
#include <iostream>
#include <cmath>
#include <string>
#include "TicTacToe.h"

#include "Trayectorias.h"

//######################################################### TRAYECTORIAS
int position;
float coord_z = 0.5;

const int POS = 9; // numero de posiciones en el tablero (19)
const int COORD = 3; // numero de coordenadas (x,y,z)
float gamePos[POS][COORD] = 
{
  {1, 1, coord_z}, // Posicion de inicio de X, 1-5 ↓
  {2, 2, coord_z},
  {3, 3, coord_z},
  {4, 4, coord_z},
  {5, 5, coord_z},
  {6, 6, coord_z},
  {7, 7, coord_z},
  {8, 8, coord_z},
  {9, 9, coord_z},
  
  
};
//######################################################### KEYPAD
const uint8_t ROWS = 4; // Cuatro filas
const uint8_t COLS = 4; // Cuatro columnas
// Define los pines de conexión del keypad
uint8_t rowPins[ROWS] = { 20, 21, 47, 48 }; 
uint8_t colPins[COLS] = { 45, 0, 2, 1};
// Define los caracteres del keypad
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
// Crea el objeto Keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

//######################################################### LEDS

//int leds[] = {11, 10, 9, 46, 3}; // LED 1 a 5 conectados a los pines 11...
int leds[] = {3, 46, 9, 10, 11}; // LED 1 a 5 conectados a los pines 11...
unsigned long blinkTimes[5] = {0, 0, 0, 0, 0}; // Array to store the last blink times for each LED  

//######################################################### BUTTONS
//Define turn button pin
const int pinTurnButton = 41;
// store the state of the turn button
int turnButtonState = 0;

//######################################################### GAME
// Char to store the result from CheckWin() function
char fin = ' ';


void blinkLed(int ledIndex) {
  static bool ledStates[5] = {LOW, LOW, LOW, LOW, LOW}; // Static array to hold the current state (ON/OFF) of each LED
  unsigned long currentTime = millis(); // Get the current time in milliseconds
  if (currentTime - blinkTimes[ledIndex] >= 200) { // Check if 200 milliseconds have passed since the last toggle
    blinkTimes[ledIndex] = currentTime; // Update the last blink time for this LED
    ledStates[ledIndex] = !ledStates[ledIndex]; // Toggle the LED state
    digitalWrite(leds[ledIndex], ledStates[ledIndex]); // Apply the new state to the LED
  }
}

void setup() {
   Serial.begin(115200);
   for (int i = 0; i < 5; i++) {
    pinMode(leds[i], OUTPUT); // Configura los pines de los LEDs como salidas
  }
  pinMode(pinTurnButton, INPUT_PULLDOWN); //or just INPUT
}
 
void loop() {

  char key;
  key = keypad.getKey();
  uint16_t now = millis();


  // flag that indicates if the next player can play
  bool flag = true;
  bool _dificultad = false;

  //PARPADEOS LED PARA GANADOR / EMPATE
  if (fin == 'D')
  {
    blinkLed(2);
    blinkLed(3);
  } else
  {
    if(fin != ' ')
    {
      (fin == 'X')? blinkLed(2) : blinkLed(3);
    }
  }

  if (key) {
    fin = ' ';
    // Creating an object for the Game class
    Game* game = new Game;
    // Initializing a 2D array for the gameboard
    game->CreateBoard();

    switch (key) {
      case 'A':
      {
        digitalWrite(leds[1], HIGH); // Enciende LED 2 de manera sólida
        Serial.println("modo automatico seleccionado");   
        Player player("Player X");
        while (flag)
        {
          // Player turn
          digitalWrite(leds[2], HIGH); //ENCIENDE LED 3 - TURNO DE LA X
          position = game->PlayerTurn(player);
          digitalWrite(leds[2], LOW); //APAGA LED 3 - TURNO DE LA X
          // Check if player 1 has won the game or draw
          fin = game->CheckWin();
          if (fin != ' ')
          {
            goto point;
          }
          // Machine turn
          digitalWrite(leds[3], HIGH); //ENCIENDE LED 4 - TURNO DE LA O (Maquina)
          delay(2000); //######################################################## TEMPORAL
          position = game->MachineTurn();
          digitalWrite(leds[3], LOW); //APAGA LED 4 - TURNO DE LA O (Maquina)
          // Check if any player won the game
          fin = game->CheckWin();
          // Conditions to check which player won the game or Draw
          point:
            if (fin == 'X')
            {
              Serial.println(" X wins");
              // break the loop, as a result, is reached
              flag = false;
            }
            else if (fin == 'O')
            {
              Serial.println("O Wins");
              flag = false;
            }
            else if (fin == 'D')
            {
              Serial.println("The game ended in a draw");
              flag = false;
            }
        }
        digitalWrite(leds[1], LOW); // Apaga LED 2 de manera sólida
        break;
      }

      case 'B':
      {
        Serial.println("Modo Manual Seleccionado");

        // Creating two player objects
        Player player("Player X");
        Player player2("Player O");
        // iterate until a winner or draw is reached
        while (flag)
        {    
          
          // player 1 turn
          digitalWrite(leds[2], HIGH); //ENCIENDE LED 3 - TURNO DE LA X
          position = game->PlayerTurn(player);

          digitalWrite(leds[2], LOW); //ENCIENDE LED 3 - TURNO DE LA X
          fin = game->CheckWin(); // Check if there is a winner or a draw in the match
          if (fin != ' ')
          {
            goto point2;
          }

          // player 2 turn
          digitalWrite(leds[3], HIGH); //ENCIENDE LED 4 - TURNO DE LA O
          position = game->PlayerTurn(player2);
          digitalWrite(leds[3], LOW); //APAGA LED 4 - TURNO DE LA O
          fin = game->CheckWin();// Check if there is a winner or a draw in the match
          // Conditions to check the winner or draw in the match
          point2:
            if (fin == 'X')
            {
              Serial.println(" X wins");
              // break the loop, as a result, is reached
              flag = false;
            }
            else if (fin == 'O')
            {
              Serial.println("O Wins");
              flag = false;
            }
            else if (fin == 'D')
            {
              Serial.println("The game ended in a draw");
              flag = false;
            }
        }

        break;
        
      }
        
      case 'C':
      {
        Serial.println("Modo Calibracion");
        digitalWrite(leds[0], HIGH); //ENCIENDE LED 1 DE MANERA SOLIDA
        delay(3000);
        digitalWrite(leds[0], LOW); //ENCIENDE LED 1 DE MANERA SOLIDA
        break;
      }

      case 'D':
      {
        if(game->ChangeDificulty()) //MODO DIFICIL
        {
          digitalWrite(leds[4], HIGH); //ENCIENDE LED 5 DE MANERA SOLIDA
        }else                       //MODO FACIL
        {
          digitalWrite(leds[4], LOW);//APAGA LED 5
        }

        break;
      }

      case '0':
      {
        Serial.println("No hay nada aquí");
        break;
      }
      case '*':
      {
        fin = ' ';
        digitalWrite(leds[2], LOW);//APAGA LED 2
        digitalWrite(leds[3], LOW);//APAGA LED 3

        break;
      }

    }
  }

}
