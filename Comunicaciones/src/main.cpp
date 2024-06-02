#include <Arduino.h>
#include "WiFi.h"
#include "WiFiClient.h"

const char* ssid = "sercommBB1924";
const char* password = "GUQTSMTGHUTHH2TJ";
WiFiServer server(55355);
WiFiClient conn;

void setup() {
    Serial.begin(115200);

    // Configuración de dirección IP estática, gateway, subnet, y DNS
    IPAddress local_ip(192, 168, 0, 19);
    IPAddress gateway(192, 168, 0, 1);  // Ajusta según tu red, usualmente es el router
    IPAddress subnet(255, 255, 255, 0);
    

    WiFi.config(local_ip, gateway, subnet);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("Conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    // Iniciar el servidor en el puerto especificado
    server.begin();  // Esta línea es crucial
    Serial.println("Servidor TCP iniciado");
}

void loop() {
    // Verificar si hay clientes nuevos y procesar datos
    if (!conn || !conn.connected()) {
        conn = server.available();  // Aceptar un nuevo cliente
        if (conn) {
            Serial.println("Cliente conectado");
        }
    }

    if (conn.connected() && conn.available()) {  // Verificar si el cliente envió datos
        String recv = conn.readStringUntil('\n');
        Serial.print("RECV: ");
        Serial.println(recv);
        conn.flush();  // Asegurarse de que los datos son enviados de vuelta
    }
}