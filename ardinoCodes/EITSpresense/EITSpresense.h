/*
  EITKitArduino.h - Library for EIT-kit Sensing Board.
  Will be released into the public domain.
*/
#ifndef EITSpresense_h
#define EITSpresense_h

#include <Arduino.h>  // Arduino 1.0
#include "SPI.h"

typedef enum { AD, OP, MONO } Meas_t;
typedef enum { SRC, SINK, VP, VN } Mux_t;

#define NUM_ELECTRODES      8
#define NUM_MEAS            NUM_ELECTRODES*NUM_ELECTRODES //used only for instantiation of measurement arrays
#define MAX_ELECTRODES      8 // maximum electrodes that can be used

// AD9833(signal generator) register addresses

// Constants
#define RATE_SPI_FREQ       250000
#define RATE_I2C_FREQ       100000

/* For Spresense */
// SPI interface
#define PIN_SPI_MOSI        11
#define PIN_SPI_MISO        12
#define PIN_SPI_SCK         13
#define PIN_SPI_SS_AD9833   2
// I2C interface
#define PIN_I2C_SDA         14
#define PIN_I2C_SCL         15

class EITSpresense
{
  public:
    EITSpresense();
    EITSpresense(int num_electrodes, int num_bands, int num_terminals, Meas_t drive_type, Meas_t meas_type, bool bluetooth_communication);

    void EITSpresense::take_measurements(Meas_t drive_type, Meas_t meas_type);
    void EITSpresense::AD9833_write(uint16_t val);
    uint16_t EITSpresense::AD9833_send_and_recv(uint16_t val);

  private:
    // Global calibration parameters

    // Measurement Settings
    int _num_electrodes = 8; // total number of electrodes for measurement per band
    int _num_meas = _num_electrodes*_num_electrodes; // total number of electrodes for measurement per band
    int _num_bands = 1; // total number of bands used in measurement
    int _num_terminals = 4; // 2-terminal or 4-terminal measurement protocol, 4 terminal is the default
    Meas_t _drive_type = AD; // protocol for electrodes used in current injection
    Meas_t _meas_type = AD; // protocol for electrodes used in voltage reading 

    void spi_write(uint8_t data_pin, uint8_t clock_pin, uint32_t freq, uint8_t bit_order, uint8_t mode, uint8_t bits, uint32_t val);

    void mux_write_to_electrode(Mux_t chip_select, uint8_t electrode_sel, uint8_t enable);
    void mux_write(const int chip_select, uint8_t pin_sel, uint8_t enable);
};

#endif