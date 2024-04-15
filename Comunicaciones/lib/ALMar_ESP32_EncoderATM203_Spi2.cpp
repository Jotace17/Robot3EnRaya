#include "ALMar_ESP32_EncoderATM203_Spi2.h"

#ifdef ESP32

#include <Arduino.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>

AlMar::Esp32::EncoderATM203_Spi2::EncoderATM203_Spi2(const int pin_cs[],int n_devices,int pin_mosi,int pin_miso,int pin_sclk) {

    spi_bus_config_t spi_config={
        .mosi_io_num=pin_mosi ,
        .miso_io_num=pin_miso , 
        .sclk_io_num=pin_sclk ,    // not used
        .quadwp_io_num=-1 ,    // not used
        .quadhd_io_num=-1 ,    // not used
        .data4_io_num=-1 ,    // not used
        .data5_io_num=-1 ,    // not used
        .data6_io_num=-1 ,    // not used
        .data7_io_num=-1 ,    // not used
        .max_transfer_sz=0 , // Default
        .flags = 0 , // TODO
        .intr_flags = 0
    };

    esp_err_t err=spi_bus_initialize(SPI2_HOST,&spi_config,SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(err);

    nDev=n_devices;

    for (int j=0;j<n_devices;j++) {
        spi_device_interface_config_t device={
            .command_bits=0 , 
            .address_bits=0 , 
            .dummy_bits=0 , 
            .mode= 0 ,
            .duty_cycle_pos= 0 , // Default
            .cs_ena_pretrans=0 ,
            .cs_ena_posttrans=0 ,
            .clock_speed_hz = 1000000 , // TODO: test 
            .input_delay_ns = 0 ,
            .spics_io_num = pin_cs[j] ,
            .flags = 0 , // TODO
            .queue_size = 1 , 
            .pre_cb = nullptr ,
            .post_cb = nullptr
        };
        err=spi_bus_add_device(SPI2_HOST,&device,(spi_device_handle_t*) &hDev[j]);
        ESP_ERROR_CHECK(err);
    }
}

uint8_t AlMar::Esp32::EncoderATM203_Spi2::SpiWriteRead(int iDev,uint8_t v) 
{
  uint8_t rd;
  spi_transaction_t trans = {
    .flags = 0 , // TODO
    .cmd=0 ,
    .addr = 0,
    .length = 8,
    .rxlength = 8,
    .user =nullptr ,
    .tx_buffer = &v ,
    .rx_buffer= &rd
  };
  esp_err_t err=spi_device_polling_transmit((spi_device_handle_t) hDev[iDev],&trans);
  ESP_ERROR_CHECK(err);
  //Serial.printf("Device[%i]\t###\t tx: 0x%x\t rx:0x%x\n", iDev, v, rd);
  return rd;
}

int AlMar::Esp32::EncoderATM203_Spi2::Read(int iDev)
{
  uint8_t recv;
  int count=0;
  recv=SpiWriteRead(iDev,0x10);
  while (recv==0xA5) {
    delayMicroseconds(20);
    recv=SpiWriteRead(iDev,0x0);
    count++;
    if (count>=40) {
       return 0x40000000;
        break;
    }
  }
 // printf("Recv=%02x\n",recv);
    if (recv==0x10)
  {
      uint32_t msb=SpiWriteRead(iDev,0x00) & 0x0F;
      delayMicroseconds(20);
      uint32_t lsb=SpiWriteRead(iDev,0x00);
      uint32_t pos=(msb<<8) | lsb;
//      if (msb & 0x08)
//          pos |= 0xFFFF8000;
      //Serial.printf("[MOTOR %i] ### msb: 0x%x, lsb: 0x%x, pos:%i\n", iDev, msb, lsb, pos);
      return *((int*) &pos);
  }
  else {
//    Serial.printf("Recv after %d tries = 0x%02x\n",count,recv);
    return 0x80000000;
  }

  return 0x20000000;
}

bool AlMar::Esp32::EncoderATM203_Spi2::SetZero(int iDev) {
  
  int count=0;
  uint8_t recv=SpiWriteRead(iDev,0x70);
  while (recv!=0x80) {
    delayMicroseconds(20);
    recv=SpiWriteRead(iDev,0x0);
    count++;
    if (count>=40) {
        break;
    }
  }
  return recv=0x80;
}

#endif // ESP32