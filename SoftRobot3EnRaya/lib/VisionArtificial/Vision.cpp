#include <Arduino.h>
#include "WiFi.h"          // Include the WiFi library
#include "WiFiClient.h"    // Include the WiFiClient library

// Network credentials
// const char* ssid = "MdMallada";        // SSID of your WiFi network
// const char* password = "mallada06"; // Password of your WiFi network

const char* ssid = "Jcaselles";        // SSID of your WiFi network
const char* password = "notieneclave"; // Password of your WiFi network

WiFiServer server(55255); // Create a server that listens on port 55355
WiFiClient conn;          // Create a client to handle incoming connections

String ReadCamera(bool print);
void ConnectCamera();
char cons;
String lastReceived;

//void setup() {
    // Serial.begin(115200); // Start the serial communication at 115200 baud rate

    // WiFi.mode(WIFI_STA);    // Set WiFi to station mode
    // WiFi.begin(ssid, password); // Start connecting to the network
    // while (WiFi.status() != WL_CONNECTED) { // Wait until the connection is established
    //     delay(1000);  // Delay between checks
    //     Serial.print("."); // Print a dot on the serial monitor as a progress indicator
    // }
    // Serial.println("Conectado"); // Print "Connected" in Spanish when connected
    // Serial.print("IP: ");
    // Serial.println(WiFi.localIP()); // Display the IP address assigned to the device

    // // Start the TCP server
    // server.begin();  // This line is crucial to start listening for incoming connections
    // Serial.println("Servidor TCP iniciado"); // Print "TCP server started" in Spanish
//}

// void loop() {
  
//   if (Serial.available()>0)
//   {
//    cons = Serial.read();

//   }
//   if (cons == 'R')
//   {
//     Serial.println("\nComando 'R' recibido:");
//     ReadCamera(false);
//     delay(1000);
//     ReadCamera(true);
//     cons = 'a';
//   }
  
//   delay(200);
    
// }

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