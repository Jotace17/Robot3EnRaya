#include <Arduino.h>
#include "WiFi.h"          // Include the WiFi library
#include "WiFiClient.h"    // Include the WiFiClient library

// Network credentials
const char* ssid = "sercommBB1924";        // SSID of your WiFi network
const char* password = "GUQTSMTGHUTHH2TJ"; // Password of your WiFi network

WiFiServer server(55355); // Create a server that listens on port 55355
WiFiClient conn;          // Create a client to handle incoming connections

void setup() {
    Serial.begin(115200); // Start the serial communication at 115200 baud rate

    // Static IP configuration
    IPAddress local_ip(192, 168, 0, 19);       // Assign a static IP address to the device
    IPAddress gateway(192, 168, 0, 1);         // Gateway, usually the router IP
    IPAddress subnet(255, 255, 255, 0);        // Subnet mask
    

    // Apply the static IP settings
    WiFi.config(local_ip, gateway, subnet);

    WiFi.mode(WIFI_STA);    // Set WiFi to station mode
    WiFi.begin(ssid, password); // Start connecting to the network
    while (WiFi.status() != WL_CONNECTED) { // Wait until the connection is established
        delay(1000);  // Delay between checks
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
    // Check for new clients and handle incoming data
    if (!conn || !conn.connected()) {
        conn = server.available();  // Check if a new client has connected
        if (conn) {
            Serial.println("Cliente conectado"); // Print "Client connected" in Spanish
        }
    }

    if (conn.connected() && conn.available()) { // If the client is connected and has sent data
        String recv = conn.readStringUntil('\n'); // Read the data until a newline character
        Serial.print("RECV: ");
        Serial.println(recv); // Print the received data
        conn.flush(); // Ensure all outgoing data is sent back to the client
    }
}
