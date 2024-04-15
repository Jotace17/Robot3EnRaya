#include <Arduino.h>
#include "../lib/ALMar_ESP32_Driver_L298n.cpp"
#include "../lib/ALMar_ESP32_EncoderATM203_Spi2.cpp"

#define PIN_CS_M1  10 // (chip select para encoder de motor 1) 10 for SPI3
#define PIN_CS_M2  40  // (chip select para encoder de motor 2) 
#define PIN_CS_M3  41  // (chip select para encoder de motor 3) 
// Pines comunes a todos los encóders:
#define PIN_SCLK  12 // 36 // 12 for SPI3
#define PIN_MISO  13 // 37 // 13 for SPI3
#define PIN_MOSI  11 // 35 // 11 for SPI3

#define N_MOTORS  3 // 35 // 11 for SPI3

// Define pins
int cs_pins[]={PIN_CS_M1,PIN_CS_M2,PIN_CS_M3};

AlMar::Esp32::EncoderATM203_Spi2* _enc;

void setup() {
  Serial.begin(9600);
  Serial.println("Testing motor library...");


  _enc=new AlMar::Esp32::EncoderATM203_Spi2(cs_pins,N_MOTORS,PIN_MOSI,PIN_MISO,PIN_SCLK);
  //_enc->SetZero(0);
}

void loop() {

    //int pos=_enc->Read(0); // lee encoder 0 (M1)
    int pos  = _enc->Read(0);
    int pos2 = _enc->Read(1); // lee encoder 0 (M1)

    if(pos!=0x80000000) { 
          //Serial.printf("MOTOR %i \t\t ### \t\t\t MOTOR %i\n", 0, 1);
          Serial.printf("deg: %f, read: 0x%08x \t\t### \tdeg: %f, read: 0x%08x \n", (float) (pos*360.0/4096.0), pos, (float) (pos2*360.0/4096.0), pos2);
    }
    else
    {
      //Serial.printf("\t ### ERROR LECTURA: %lu\n", pos);
    }

    delay(1000);
  }
