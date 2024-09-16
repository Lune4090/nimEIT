/*
  EITSpresense.cpp - Library for EIT sensing system on Spresense.
  Will be released into the public domain.
*/

// MCP4018(デジタルポテンショメーター(可変抵抗器))

#include <Arduino.h>
#include "EITSpresense.h"
#include "SPI.h"
#include "assert.h"
#include <string>
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream
#include <HardwareSerial.h>


EITSpresense::EITSpresense(){
  EITSpresense(32,1,2, AD, AD, false);
}

EITSpresense::EITSpresense(int num_electrodes, int num_bands, int num_terminals, Meas_t drive_type, Meas_t meas_type, bool bluetooth_communication) {
  Serial.begin(115200);
  Serial.println("starting with ");
  Serial.println(RATE_SPI_FREQ);
  
  _num_electrodes = num_electrodes; // number of electrodes for measurement per band
  _num_meas = _num_electrodes*_num_electrodes;
  _num_bands = num_bands; // total number of bands used in measurement
  _num_terminals = num_terminals; // 2-terminal or 4-terminal measurement protocol
  _drive_type = drive_type; // protocol for electrodes used in current injection
  _meas_type = meas_type; // protocol for electrodes used in voltage reading 

  /* 各ピンのIn/Out定義 */
  
  // SPI initialization
  pinMode(PIN_SPI_MOSI, OUTPUT);
  pinMode(PIN_SPI_MISO, INPUT);
  pinMode(PIN_SPI_SCK, OUTPUT);
  pinMode(PIN_SPI_SS_AD9833, OUTPUT);

  // I2C initialization
  pinMode(PIN_I2C_SDA, OUTPUT);
  pinMode(PIN_I2C_SCL, OUTPUT);
  
  // マルチプレクサに対する電流/電圧印加ピン
  // I_in
  // I_out
  // V+
  // V-

  /* 各種外部IC初期化 */

  // 波形発生器初期化
}

/* 信号読み取り */
// TODO: 計測
void EITSpresense::take_measurements(Meas_t drive_type, Meas_t meas_type){

}

/* For SPI comminucation */
// Shift a byte out serially with the given frequency in Hz (<= 500kHz)
void EITSpresense::spi_write(uint8_t data_pin, uint8_t clock_pin, uint32_t freq, uint8_t bit_order, uint8_t mode, uint8_t bits, uint32_t val){
    uint32_t period = (freq >= 250000) ? 1 : (250000 / freq);   // Half clock period in uS
    uint8_t cpol = (mode == SPI_MODE2 || mode == SPI_MODE3);
    uint8_t cpha = (mode == SPI_MODE1 || mode == SPI_MODE3);
    uint8_t sck = cpol ? HIGH : LOW;

    uint8_t i;
    uint32_t start_time;

    Serial.print("period: ");
    Serial.println(period);
    Serial.print("SPI mode: ");
    Serial.println(mode);

    // Set clock idle for 2 periods
    digitalWrite(clock_pin, sck);
    delayMicroseconds(period*4);

    for (i = 0; i < bits; i++)  {
        start_time = micros();

        // Shift bit out
        if (bit_order == LSBFIRST)
            digitalWrite(data_pin, !!(val & (1 << i)));
        else    
            digitalWrite(data_pin, !!(val & (1 << ((bits-1) - i))));

        // Toggle clock leading edge
        sck = !sck;
        if (cpha) {
            digitalWrite(clock_pin, sck);
            while(micros() - start_time < period);
        } else {
            while(micros() - start_time < period);
            digitalWrite(clock_pin, sck);
        }

        // Toggle clock trailing edge
        start_time = micros();
        sck = !sck;
        if (cpha) {
            digitalWrite(clock_pin, sck);
            while(micros() - start_time < period);
        } else {
            while(micros() - start_time < period);
            digitalWrite(clock_pin, sck);
        }
    }
}

/* For signal generator (AD9833) */

// Wrapper for AD9833 using original SPI program
void EITSpresense::AD9833_write(uint16_t val) {
  digitalWrite(PIN_SPI_SS_AD9833, LOW);
  delayMicroseconds(500);
  spi_write(PIN_SPI_MOSI, PIN_SPI_SCK, RATE_SPI_FREQ, MSBFIRST, SPI_MODE1, 16, val);
  delayMicroseconds(500);
  digitalWrite(PIN_SPI_SS_AD9833, HIGH);
  delayMicroseconds(500);
}

// Wrapper for AD9833 using standard SPI program
uint16_t EITSpresense::AD9833_send_and_recv(uint16_t val) {
  // Start the SPI library
  SPI.begin();
  // Configure the SPI port
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
  
  // Begin data transfer
  digitalWrite(PIN_SPI_SS_AD9833, LOW);
  uint16_t content = 0;
  content = SPI.transfer16(val);
  digitalWrite(PIN_SPI_SS_AD9833, HIGH);

  // End the SPI transation
  SPI.endTransaction();
  // Disable SPI port and free pins for use as GPIO
  SPI.end();
  return content;
}

/* For MUX (TC4051BP) */

// MUX書き込み
void EITSpresense::mux_write(const int chip_select, uint8_t pin_sel, uint8_t enable){
  
}

// MUXの接続先を選んで所望の電極に接続
void EITSpresense::mux_write_to_electrode(Mux_t chip_select, uint8_t electrode_sel, uint8_t enable){
  
}
