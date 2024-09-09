/*
  EITSpresense.cpp - Library for EIT sensing system on Spresense.
  Will be released into the public domain.
*/

// MCP4018(デジタルポテンショメーター(可変抵抗器))
  // AnalogDevicesの資料(https://www.analog.com/media/jp/technical-documentation/data-sheets/AD5270_AD5271_jp.pdf) p.19~ を参照
  // AD5270の制御にはSPI通信が利用され、シフトレジスタに対して送られる16bitの信号からなる10個のコードで行われる
    // 2bit: 未使用, 4bit(C3~C0): コマンドビット, 10bit(D9~D0): データビット
  // 抵抗値の制御はRDACレジスタ、保存は50-TPメモリレジスタ、
  // シフトレジスタの書き込みは以下の順に行われる
    // 1. 書き込みシーケンス開始(SYNC -> Lo)
    // 2. シフトレジスタへ送る信号をDINピンに入力
    // 3. 書き込みシーケンス終了(SYNC -> High)
    // 4. シリアル・データ・ワードのデコード
  // 具体的なコードは以下の通り
    // Command 0: __+0000+xxxxxxxxxx: NOOP
    // Command 1: __+0001+oooooooooo: シリアルレジスタデータのRDACレジスタ(抵抗を制御)への書き込み
    // Command 2: __+0010+xxxxxxxxxx: RDACワイパーレジスタの読み出し
    // Command 3: __+0011+xxxxxxxxxx: RDACレジスタの設定を50-TP(最大50回書き込み可能な不揮発性メモリ, 抵抗値を格納)に格納
    // Command 4: __+0100+xxxxxxxxxx: RDACを50-TP内の最新のデータに更新
    // Command 5: __+0101+xxxxoooooo: 次のフレームに50-TPの内容を読み出し
    // Command 6: __+0110+xxxxxxxxxx: 50-TP内の最新のプログラムされたメモリ位置を読み出し
    // Command 7: __+0111+xxxxxxxooo: シリアルレジスタデータを制御レジスタに書き込み
    // Command 8: __+1000+xxxxxxxxxx: 制御レジスタの内容を読み出し
    // Command 9: __+1001+xxxxxxxxxo: D0=0でノーマルモード, D0=1でデバイスをシャットダウン
  // 一方、上記のコマンドの内、読み出しを行うもの(2, 5, 6, 8)は、シリアルデータ出力(SDO)ピンの出力からその内容を読み取る
  // が、EIT-kitではAD5270のSDOピンは使用されていない為割愛する

#include <Arduino.h>
#include "EITSpresense.h"
#include "SPI.h"
#include "assert.h"
#include <string>
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream
#include <HardwareSerial.h>


EITSpresense::EITSpresense(){
  EITKitArduino(32,1,2, AD, AD, false);
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
  pinMode(PIN_I2C_SCK, OUTPUT);
  
  // マルチプレクサに対する電流/電圧印加ピン
  // I_in
  // I_out
  // V+
  // V-

  /* 各種外部IC初期化 */

  // 波形発生器初期化
  AD9833_write(CTRL_REG, 0b011111110011);
}

/* 信号読み取り */
// TODO: 計測
void EITSpresense::take_measurements(Meas_t drive_type, Meas_t meas_type){

}

/* For SPI comminucation */
// Shift a byte out serially with the given frequency in Hz (<= 500kHz)
void EITSpresense::spi_write(uint8_t data_pin, uint8_t clock_pin, uint32_t freq, uint8_t bit_order, uint8_t mode, uint8_t bits, uint32_t val){
    uint32_t period = (freq >= 500000) ? 1 : (500000 / freq);   // Half clock period in uS
    uint8_t cpol = (mode == SPI_MODE2 || mode == SPI_MODE3);
    uint8_t cpha = (mode == SPI_MODE1 || mode == SPI_MODE3);
    uint8_t sck = cpol ? HIGH : LOW;

    uint8_t i;
    uint32_t start_time;

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

// Wrapper for AD9833 using SPI
void EITSpresense::AD9833_write(uint16_t val) {
  digitalWrite(CHIP_SEL_AD5930, LOW);
  spi_write(MOSI_PIN, SCK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 16, data_word);
  digitalWrite(CHIP_SEL_AD5930, HIGH);
}

/* For MUX (TC4051BP) */

// MUX書き込み
void EITSpresense::mux_write(const int chip_select, uint8_t pin_sel, uint8_t enable){
  
}

// MUXの接続先を選んで所望の電極に接続
void EITSpresense::mux_write_to_electrode(Mux_t chip_select, uint8_t electrode_sel, uint8_t enable){
  
}
