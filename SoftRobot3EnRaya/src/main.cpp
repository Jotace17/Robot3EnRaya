//System libraries
#include <Arduino.h>
#include <Keypad.h>

//Game libraries
#include <iostream>
#include <cmath>
#include <string>
#include "TicTacToe.h"

//Trayectories libraries
#include "Trayectorias.h"

//WiFi libraries for camera
#include "WiFi.h"   
#include "WiFiClient.h" 
#include "Vision.cpp" 


// Network credentials
//const char* ssid = "MdMallada";        // SSID of your WiFi network
//const char* password = "mallada06"; // Password of your WiFi network
// const char* ssid = "Jcaselles";        // SSID of your WiFi network
// const char* password = "notieneclave"; // Password of your WiFi network

// WiFiServer server(55255); // Create a server that listens on port 55355
// WiFiClient conn;          // Create a client to handle incoming connections
//String lastReceived;

//######################################################### TRAYECTORIAS
int position;
float coord_z = 0.5;

const int POS = 19; // numero de posiciones en el tablero (19)
const int COORD = 3; // numero de coordenadas (x,y,z)
float tablePos[POS][COORD] = 
{
  {310, -70, coord_z}, // Posicion de inicio de O, 1-5 ↓
  {310, -35, coord_z},
  {310, 0, coord_z},
  {310, 35, coord_z},
  {310, 70, coord_z},

  {450, -70, coord_z}, // Posicion de inicio de X, 6-10 ↓
  {450, -35, coord_z},
  {450, 0, coord_z},
  {450, 35, coord_z},
  {450, 70, coord_z},

  {340.75, 39.25, coord_z}, // Posicion casilla 1
  {340.75, 0, coord_z}, // Posicion casilla 2
  {340.75, -39.25, coord_z}, // Posicion casilla 3

  {380, 39.25, coord_z}, // Posicion casilla 4
  {380, 0, coord_z}, // Posicion casilla 5
  {380, -39.25, coord_z}, // Posicion casilla 6

  {419.25, 39.25, coord_z}, // Posicion casilla 7
  {419.25, 0, coord_z}, // Posicion casilla 8
  {419.25, -39.25, coord_z}  // Posicion casilla 9
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
int leds[] = {9, 11, 3, 46, 10}; // LED 1 a 5 conectados a los pines 11...
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
/*
void ConnectCamera()
{
  // Check for new clients and handle incoming data
  if (!conn || !conn.connected()) {
    conn = server.available();  // Check if a new client has connected
    if (conn) {
      Serial.println("Cliente conectado"); // Print "Client connected" in Spanish
    }
  }
}

String ReadCamera(bool print)
{
  ConnectCamera();
  conn.write("LECTURA_CAMARA");

  if (conn.connected() && conn.available()) { // If the client is connected and has sent data

    String recv = conn.readStringUntil('\n');
    // Read the data until a newline character
    recv = recv.substring(0, 9);
    lastReceived = recv;

    if (print)
    {
      Serial.print("RECV: ");
      Serial.println(lastReceived); // Print the received data
    }
    conn.flush(); // Ensure all outgoing data is sent back to the client
    return lastReceived;
    
  }
  return " ";
}

*/


void setup() {
  Serial.begin(115200);
  
  for (int i = 0; i < 5; i++) {
    pinMode(leds[i], OUTPUT); // Configura los pines de los LEDs como salidas
  }

  pinMode(pinTurnButton, INPUT_PULLDOWN); //or just INPUT

  //######################################################### WIFI SETUP FOR CAMERA
  WiFi.mode(WIFI_STA);    // Set WiFi to station mode
  WiFi.begin(ssid, password); // Start connecting to the network
  while (WiFi.status() != WL_CONNECTED) { // Wait until the connection is established
    digitalWrite(leds[0], HIGH);
    delay(500);  // Delay between checks
    digitalWrite(leds[0], LOW);
    delay(500);
    Serial.print("."); // Print a dot on the serial monitor as a progress indicator
  }
  Serial.println("Conectado"); // Print "Connected" in Spanish when connected
  Serial.print("IP: ");
  Serial.println(WiFi.localIP()); // Display the IP address assigned to the device

  // Start the TCP server
  server.begin();  // This line is crucial to start listening for incoming connections
  Serial.println("Servidor TCP iniciado"); // Print "TCP server started" in Spanish
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

      case 'A': // MODO AUTOMATICO
      {
        digitalWrite(leds[1], HIGH); // Enciende LED 2 de manera sólida
        Serial.println("modo automatico seleccionado");   
        Player player("Player X");
        while (flag)
        {
          // Player turn
          digitalWrite(leds[2], HIGH); //ENCIENDE LED 3 - TURNO DE LA X
          //position = game->PlayerTurn(player);

          while(turnButtonState == LOW)
          {
            turnButtonState = digitalRead(pinTurnButton);
            delay(20);
          }

          String cameraLecture = " ";
          while(cameraLecture == " ")
          {
            //cameraLecture = ReadCamera();
            cameraLecture = ReadCamera(false);
            delay(1000);
            cameraLecture = ReadCamera(true);
          }
          game->updateBoard(cameraLecture);

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

      case 'B': // MODO MANUAL
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
        
      case 'C': // MODO CALIBRACION
      {
        
        Serial.println("Modo Calibracion");
        digitalWrite(leds[0], HIGH); //ENCIENDE LED 1 DE MANERA SOLIDA

        turnButtonState = LOW;
        while(turnButtonState == LOW)
          {
            //MOVER EL ROBOT MANUALMENTE CON TECLAS

            turnButtonState = digitalRead(pinTurnButton);
            delay(20);
          }

          // LEER LOS ENCODERS Y SETEARLOS

        digitalWrite(leds[0], LOW); //ENCIENDE LED 1 DE MANERA SOLIDA
        break;
      }

      case 'D': // CAMBIO DE DIFICULTAD
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

      case '0': // VACIO
      {
        Serial.println("No hay nada aquí");
        break;
      }

      case '*': // APAGA LOS PARPADEOS DE FIL DEL JUEGO
      {
        fin = ' ';
        digitalWrite(leds[2], LOW);//APAGA LED 2
        digitalWrite(leds[3], LOW);//APAGA LED 3

        break;
      }

      case '#':
      {
        digitalWrite(leds[0], HIGH); //ENCIENDE LED 1 DE MANERA SOLIDA
        delay(3000);
        digitalWrite(leds[0], LOW); //ENCIENDE LED 1 DE MANERA SOLIDA

        digitalWrite(leds[1], HIGH); //ENCIENDE LED 1 DE MANERA SOLIDA
        delay(3000);
        digitalWrite(leds[1], LOW); //ENCIENDE LED 1 DE MANERA SOLIDA

        digitalWrite(leds[2], HIGH); //ENCIENDE LED 1 DE MANERA SOLIDA
        delay(3000);
        digitalWrite(leds[2], LOW); //ENCIENDE LED 1 DE MANERA SOLIDA

        digitalWrite(leds[3], HIGH); //ENCIENDE LED 1 DE MANERA SOLIDA
        delay(3000);
        digitalWrite(leds[3], LOW); //ENCIENDE LED 1 DE MANERA SOLIDA

        digitalWrite(leds[4], HIGH); //ENCIENDE LED 1 DE MANERA SOLIDA
        delay(3000);
        digitalWrite(leds[4], LOW); //ENCIENDE LED 1 DE MANERA SOLIDA

        break;
      }
      
    }
  }

}
