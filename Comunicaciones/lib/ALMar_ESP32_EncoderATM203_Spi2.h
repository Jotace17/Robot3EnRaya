#ifndef ALMar_ESP32_EncoderATM203_Spi2
#define ALMar_ESP32_EncoderATM203_Spi2

#include <stdint.h>

//! \file AlMar_ESP32_Encoder_ATM203_Spi2.h
//! \brief Read SPI encoder ATM203-V (https://www.mouser.es/datasheet/2/670/amt20_v-1775926.pdf)<br>
//!        All encoders sharing the same SPI bus (MISO, MOSI, SCLK) <br>
//!        Arduino Uno SPI:  CS=10, MOSI=11, MISO=12, SCLK=13<br>
//!        HelTec ESP32S3 SPI:  CS=8, MOSI=10, MISO=11, SCLK=9

namespace AlMar 
{
#ifdef ESP32
namespace Esp32 {

class EncoderATM203_Spi2 {
private:
    int pinCS,pinMOSI,pinMISO,pinSCLK;
    int accum=0;
    void* hDev[6];
    int nDev;
public:
    //! Constructor
    //! \param : pin_cs --> array of CS activation pins for each connected encoder (max 6)
    //! \param: n_devices --> number of devices in pin_cs array
    //! \param: pin_mosi --> common pin for MOSI 
    //! \param: pin_miso --> common pin for MISO
    //! \param: pin_sclk --> common pin for SCLK
    EncoderATM203_Spi2(const int pin_cs[],int n_devices,int pin_mosi,int pin_miso,int pin_sclk);

    //! Read encoder value for specific device
    //! \param: iDev (device index 0...n_devices-1)
    //! \return: value read (absolute value)
    int Read(int iDev);

    //! Set zero encoder value for specific device
    //! \param: iDev (device index 0...n_devices-1)
    //! \return: boolean: operation Ok
    bool SetZero(int iDev);
private:
    uint8_t SpiWriteRead(int iDev,uint8_t write);
} ;	// class AlMar::Esp32::EncoderATM203_Spi2

} // namespace Esp32

#endif // ESP32

} // namespace AlMar

#endif // ALMar_ESP32_EncoderATM203_Spi2