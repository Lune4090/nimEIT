/*
  EITKitArduino.cpp - Library for EIT-kit Sensing Board.
  Will be released into the public domain.
*/

/* AD5930(波形生成器)/AD5270(デジタルポテンショメーター(可変抵抗器))プログラム */

// AD5930
  // AnalogDevicesの資料(https://www.analog.com/media/jp/technical-documentation/data-sheets/AD5930_JP.pdf) p.17~ を参照
  // AD5930のプログラム動作にはSPI通信が利用され、機能・オプションは16bitの信号で制御される
    // 掃引時の制御はCTRLピンのトリガーで行われる
  // 上位4bit(D15~12): 12bitレジスタの指定
    // 0000: C_REG:   制御ビット
    // 0001: N_INCR:  インクリメント数
    // 0010: Δf:      デルタ周波数下位12ビット
    // 0011: Δf:      デルタ周波数上位12ビット
    // 01__: t_INT:   インクリメント・インターバル
    // 10__: T_BURST: バースト・インターバル
    // 1100: F_START: 開始周波数の下位12ビット
    // 1101: F_START: 開始周波数の上位12ビット
    // 1110: :        予備
    // 1111: :        予備
  // 下位12bit(D11~0): 各種機能制御
    // D11: B24:            F_START, Δfレジスタの書き込み動作切り替え, 1=12LSB+12MSB(001_, 110_), 0=12LSB(0010 or 1100)/12MSB(0011 or 1101)
    // D10: DAC ENABLE:     1=DAC enable, 0=disable
    // D9 : SINE/TRI:       1=sin波出力, 0=三角波出力
    // D8 : MSBOUTEN:       1=MSBOUT pin enable, 0=disable
    // D7 : CW/BURST:       1=連続モード, 0=BURSTモード
    // D6 : INT/EXT BURST:  When D7=0, バーストの(1=)外部制御/(0=)内部制御の切り替え
    // D5 : INT/EXT INCR:   1=CTRLピンを使用した周波数の外部インクリメント, 0=自動インクリメント
    // D4 : MODE:           1=鋸歯状波掃引, 0=三角波掃引
    // D3 : SYNCSEL:        When D2=1, 1=パルス出力はEOS(掃引終了)時, 0=パルス出力は周波数インクリメント時
    // D2 : SYNCOUTEN:      1=SYNC信号出力(SYNCOP)ピン, 0=非出力
    // D1 : 予備
    // D0 : 予備

// AD5270
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
#include "EITKitArduino_mod.h"
#include "SPI.h"
#include "assert.h"
#include <string>
#include <iomanip>
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream
#include <HardwareSerial.h>

// State of MCP4252 TCON register
uint8_t tcon_reg = 0xFF;

extern volatile uint32_t F_CPU_ACTUAL;

// GPIO Pin to analog channel mapping from ~/.arduino15/packages/teensy/hardware/avr/1.59.0/cores/teensy4/analog.c (linux)
extern const uint8_t pin_to_channel[42];
// Store raw GPIO readings
uint32_t gpio_buf[MAX_SAMPLES*ADC_AVG];

EITKitArduino::EITKitArduino(int num_electrodes, int num_bands, int num_terminals, Meas_t drive_type, Meas_t meas_type, bool bluetooth_communication)
{
  Serial.begin(115200);
  Serial.println("starting with ");
  Serial.println(SPI_FREQ_FAST);
  
  _num_electrodes = num_electrodes; // number of electrodes for measurement per band
  _num_meas = _num_electrodes*_num_electrodes;
  _num_bands = num_bands; // total number of bands used in measurement
  _num_terminals = num_terminals; // 2-terminal or 4-terminal measurement protocol
  _drive_type = drive_type; // protocol for electrodes used in excitation current
  _meas_type = meas_type; // protocol for electrodes used in voltage reading 
  _bluetooth_communication = bluetooth_communication;

  /* 各ピンのIn/Out定義 */

  // spi通信におけるCS(コントロール)ピン、各ICで固有
  pinMode(CHIP_SEL_DRIVE, OUTPUT);
  pinMode(CHIP_SEL_MEAS, OUTPUT);
  pinMode(CHIP_SEL_AD5930, OUTPUT);

  // spi通信におけるMOSI, SCKピン、デイジーチェーン接続された全ICが共有
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  
  // マルチプレクサに対する電流/電圧印加ピン
  pinMode(CHIP_SEL_MUX_SRC, OUTPUT);  // I_in
  pinMode(CHIP_SEL_MUX_SINK, OUTPUT); // I_out
  pinMode(CHIP_SEL_MUX_VP, OUTPUT);   // V+
  pinMode(CHIP_SEL_MUX_VN, OUTPUT);   // V-

  // AD5930(波形生成器)をコントロールするピン
  pinMode(AD5930_INT_PIN, OUTPUT);
  pinMode(AD5930_CTRL_PIN, OUTPUT);
  pinMode(AD5930_STANDBY_PIN, OUTPUT);
  pinMode(AD5930_MSBOUT_PIN, INPUT);

  // ADC input
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  pinMode(22, INPUT);
  pinMode(23, INPUT);

  /* 各種外部IC初期化 */

  digitalWrite(CHIP_SEL_DRIVE, HIGH);
  digitalWrite(CHIP_SEL_MEAS, HIGH);
  digitalWrite(CHIP_SEL_AD5930, HIGH);

  digitalWrite(CHIP_SEL_MUX_SRC, HIGH);
  digitalWrite(CHIP_SEL_MUX_SINK, HIGH);
  digitalWrite(CHIP_SEL_MUX_VP, HIGH);
  digitalWrite(CHIP_SEL_MUX_VN, HIGH);
  
  digitalWrite(AD5930_INT_PIN, LOW);
  digitalWrite(AD5930_CTRL_PIN, LOW);
  digitalWrite(AD5930_STANDBY_PIN, LOW);

  // 制御レジスタについて、
    // F_START, Δfを分離
    // DAC enable
    // 正弦波出力
    // MSBOUT enable
    // 連続出力モード
    // CTRLピンによるバースト制御
    // CTRLピンによる周波数インクリメント
    // 三角波掃引(上がって下がる)
  // として設定
  AD5930_Write(CTRL_REG, 0b011111110011);
  AD5930_Set_Start_Freq(TEST_FREQ);

  // 一応AD5270を確実にアンロック
  AD5270_LockUnlock(CHIP_SEL_DRIVE, 0);
  AD5270_LockUnlock(CHIP_SEL_MEAS, 0);

  // AD5930の起動、以降CTRL_PINをトリガする度に周波数インクリメント
  digitalWrite(AD5930_CTRL_PIN, HIGH);
  delay(100);

  calibrateEIT();
}

/* 信号読み取り */
// TODO: 計測
void EITKitArduino::take_measurements(Meas_t drive_type, Meas_t meas_type){}

/* For SPI comminucation */
// Shift a byte out serially with the given frequency in Hz (<= 500kHz)
void EITKitArduino::spi_write(uint8_t data_pin, uint8_t clock_pin, uint32_t freq, uint8_t bit_order, uint8_t mode, uint8_t bits, uint32_t val){
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

/* For AD5270 */
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

// Write a 4-bit command and a 10-bit data word
void EITKitArduino::AD5270_Write(const int chip_sel, uint8_t cmd, uint16_t data)
{
    uint16_t data_word = (cmd << 10) | data;
    digitalWrite(chip_sel, LOW);
    delayMicroseconds(500);
    spi_write(MOSI_PIN, SCK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 16, data_word);
    delayMicroseconds(500);
    digitalWrite(chip_sel, HIGH);
}

// Enable/disable rheostat value changes
void EITKitArduino::AD5270_LockUnlock(const int chip_select, uint8_t lock){
  AD5270_Write(chip_select, CMD_WR_CTRL, lock ? 0 : 0b10);
}

// Enable/disable hardware shutdown
void EITKitArduino::AD5270_Shutdown(const int chip_select, uint8_t shutdown){
  AD5270_Write(chip_select, CMD_SHTDN, shutdown ? 1 : 0);
}

// Set the value of the digital rheostat - range is 0-0x3FF (0-100kOhm)
void EITKitArduino::AD5270_Set(const int chip_select, uint16_t val)
{
  AD5270_Write(chip_select, CMD_WR_RDAC, val);
}

/* For AD5930 */
// 上位4bit(D15~12): 12bitレジスタの指定
  // 0000: C_REG:   制御ビット
  // 0001: N_INCR:  インクリメント数
  // 0010: Δf:      デルタ周波数下位12ビット
  // 0011: Δf:      デルタ周波数上位12ビット
  // 01__: t_INT:   インクリメント・インターバル
  // 10__: T_BURST: バースト・インターバル
  // 1100: F_START: 開始周波数の下位12ビット
  // 1101: F_START: 開始周波数の上位12ビット
  // 1110: :        予備
  // 1111: :        予備
// 下位12bit(D11~0): 各種機能制御
  // D11: B24:            F_START, Δfレジスタの書き込み動作切り替え, 1=12LSB+12MSB(001_, 110_), 0=12LSB(0010 or 1100)/12MSB(0011 or 1101)
  // D10: DAC ENABLE:     1=DAC enable, 0=disable
  // D9 : SINE/TRI:       1=sin波出力, 0=三角波出力
  // D8 : MSBOUTEN:       1=MSBOUT pin enable, 0=disable
  // D7 : CW/BURST:       1=連続モード, 0=BURSTモード
  // D6 : INT/EXT BURST:  When D7=0, バーストの(1=)外部制御/(0=)内部制御の切り替え
  // D5 : INT/EXT INCR:   1=CTRLピンを使用した周波数の外部インクリメント, 0=自動インクリメント
  // D4 : MODE:           1=鋸歯状波掃引, 0=三角波掃引
  // D3 : SYNCSEL:        When D2=1, 1=パルス出力はEOS(掃引終了)時, 0=パルス出力は周波数インクリメント時
  // D2 : SYNCOUTEN:      1=SYNC信号出力(SYNCOP)ピン, 0=非出力
  // D1 : 予備
  // D0 : 予備

// Write a 12-bit data word into a register. Register addresses are 4 bits
void EITKitArduino::AD5930_Write(uint8_t reg, uint16_t data){
  uint16_t data_word = ((reg & 0x0F) << 12) | (data & 0x0FFF);
  digitalWrite(CHIP_SEL_AD5930, LOW);
  spi_write(MOSI_PIN, SCK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 16, data_word);
  digitalWrite(CHIP_SEL_AD5930, HIGH);
}

// Program the given frequency (in Hz)
void EITKitArduino::AD5930_Set_Start_Freq(uint32_t freq){
  uint32_t scaled_freq = (freq * 1.0 / AD5930_CLK_FREQ) * 0x00FFFFFF;
  uint16_t freq_low = scaled_freq & 0x0FFF;
  uint16_t freq_high = (scaled_freq >> 12) & 0x0FFF;

  AD5930_Write(SFREQ_LOW_REG, freq_low);
  AD5930_Write(SFREQ_HIGH_REG, freq_high);
}

/* For MUX(ADG731) */
// MUX書き込み
void EITKitArduino::mux_write(const int chip_select, uint8_t pin_sel, uint8_t enable){
  digitalWrite(chip_select, LOW);
  if (enable)
      spi_write(MOSI_PIN, SCK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 8, pin_sel & 0x1F);
  else
      spi_write(MOSI_PIN, SCK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 8, 0xC0 | (pin_sel & 0x1F));
  digitalWrite(chip_select, HIGH);
}

// MUXの接続先を選んで所望の電極に接続
void EITKitArduino::mux_write_to_electrode(Mux_t chip_select, uint8_t electrode_sel, uint8_t enable){
  if(electrode_sel<32){
    int cs_pin = 0;
    switch(chip_select){
      case SRC: 
        cs_pin = CHIP_SEL_MUX_SRC; break;
      case SINK: 
        cs_pin = CHIP_SEL_MUX_SINK; break;
      case VP: 
        cs_pin = CHIP_SEL_MUX_VP; break;
      case VN: 
        cs_pin = CHIP_SEL_MUX_VN; break;
    }
    mux_write(cs_pin, elec_to_mux[electrode_sel], enable);
  }else{
  }
}

/* 一旦保留 */
uint16_t EITKitArduino::analog_read()
{
    // GPIO reg bit order: 2, 3, 16, 17, 18, 19, 22, 23, 24, 25, 26, 27
    // Teensy pin order:   1, 0, 19, 18, 14, 15, 17, 16, 22, 23, 20, 21
    // All pins are on GPIO6
    
    uint16_t gpio_reg = *(&GPIO6_DR + 2) >> 16;
    uint16_t val = ((gpio_reg & 0x0200) >> 9) | // Pin 23 (GPIO 25)
                   ((gpio_reg & 0x0100) >> 7) | // Pin 22 (GPIO 24)
                   ((gpio_reg & 0x0800) >> 9) | // Pin 21 (GPIO 27)
                   ((gpio_reg & 0x0400) >> 7) | // Pin 20 (GPIO 26)
                   ((gpio_reg & 0x0003) << 4) | // Pins 19,18 (GPIO 16,17)
                    (gpio_reg & 0x00C0) |       // Pins 17,16 (GPIO 22,23)
                   ((gpio_reg & 0x0008) << 5) | // Pin 15 (GPIO 19)
                   ((gpio_reg & 0x0004) << 7);  // Pin 14 (GPIO 18)
    return val;
}

/* Read GPIO 0-31 (takes ~50.1ns) */
uint32_t EITKitArduino::gpio_read(){
  return (*(&GPIO6_DR + 2) >> 16);
}

/* Convert GPIO reading to 10-bit unsigned integer */
uint16_t EITKitArduino::gpio_convert(uint32_t gpio_reg){
  uint16_t val = ((gpio_reg & 0x0200) >> 9) | // Pin 23 (GPIO 25)
                   ((gpio_reg & 0x0100) >> 7) | // Pin 22 (GPIO 24)
                   ((gpio_reg & 0x0800) >> 9) | // Pin 21 (GPIO 27)
                   ((gpio_reg & 0x0400) >> 7) | // Pin 20 (GPIO 26)
                   ((gpio_reg & 0x0003) << 4) | // Pins 19,18 (GPIO 17,16)
                    (gpio_reg & 0x00C0) |       // Pins 17,16 (GPIO 22,23)
                   ((gpio_reg & 0x0008) << 5) | // Pin 15 (GPIO 19)
                   ((gpio_reg & 0x0004) << 7);  // Pin 14 (GPIO 18)
  return val;
}

/* キャリブレーション */
// キャリブレーション用、サンプルレートとゲインを調整
void EITKitArduino::calibrateEIT(){
  Serial.println("Beginning Calibration");

  _phase_offset = 0;
  calibrate_samples();
  calibrate_gain(AD, AD);
  Serial.println("Calibrated new");

  Serial.print("Drive gain: ");
  Serial.println(_current_gain);
  Serial.print("Measurement gain: ");
  Serial.println(_voltage_gain);
  Serial.print("Sample rate (uS per reading): ");
  Serial.println(sample_rate, 4);
  Serial.print("Samples per period: ");
  Serial.println(samples_per_period);
}

// Find the optimal number of samples to read the desired number of periods of the input signal
void EITKitArduino::calibrate_samples(){

  /* Take many samples to determine sample rate */
  num_samples = MAX_SAMPLES;
  uint32_t sample_time = read_signal(NULL, NULL, NULL, NULL, 0);
  /* Calculate sample rate and total number of samples */
  sample_rate = (float)sample_time / MAX_SAMPLES; // microsec to read each measurement ADV_AVG number of times
  samples_per_period = (1000000 / sample_rate) / TEST_FREQ; // [measurements read] / [current cycles]
  num_samples = samples_per_period * NUM_PERIODS; 
}

// Find the gains that produce the highest sinusoidal current and voltage measurements
void EITKitArduino::calibrate_gain(Meas_t drive_type, Meas_t meas_type)
{
    // Set current source electrodes to origin, set voltage measurement electrodes to overlap
    mux_write_to_electrode(SRC, 0, MUX_EN);
    mux_write_to_electrode(VP, 0, MUX_EN);
    if (drive_type == AD) {
        mux_write_to_electrode(SINK, 1, MUX_EN);
        mux_write_to_electrode(VN, 1, MUX_EN);
    } else if (drive_type == OP) {
        mux_write_to_electrode(SINK, 16, MUX_EN);
        mux_write_to_electrode(VN, 16, MUX_EN);
    }

    delay(5);

    // Set voltage measurement gain to 1 (maximum current (5V pk-pk) must be within ADC range)
    AD5270_Shutdown(CHIP_SEL_MEAS, 1);

    uint16_t i, j;
    uint32_t error_sum;
    double mag_sum;

    // Calibrate current source
    for (i = 0; i < 1024; i++) {
        _current_gain = i;
        AD5270_Set(CHIP_SEL_DRIVE, _current_gain);
        delayMicroseconds(500);

        double mag;
        uint16_t error;
        mag_sum = 0;
        error_sum = 0;
        for (j = 0; j < 10; j++) {
            read_signal(NULL, &mag, NULL, &error, 0);
            mag_sum += mag;
            error_sum += error;
        }
        mag_sum = mag_sum / 10;
        error_sum = error_sum / 10;

        // Accept the highest gain such that reading a valid sinusoid
        if (mag_sum > 0.5 && mag_sum < 2.1 && error_sum < 30)
            break;
    }

    // Set voltage measurement electrodes to the highest voltage differential point
    if (meas_type == AD) {
      mux_write_to_electrode(VP, 2, MUX_EN);
      mux_write_to_electrode(VN, 3, MUX_EN);
    } else if (meas_type == OP) {
      mux_write_to_electrode(VP, 1, MUX_EN);
      mux_write_to_electrode(VN, 17, MUX_EN);;
    }

    delay(5);

    AD5270_Shutdown(CHIP_SEL_MEAS, 0);

    // Calibrate voltage measurement
    for (i = 0; i < 1024; i++) {
        _voltage_gain = i;
        AD5270_Set(CHIP_SEL_MEAS, _voltage_gain);
        delayMicroseconds(500);

        double mag;
        uint16_t error;
        mag_sum = 0;
        error_sum = 0;
        for (j = 0; j < 10; j++) {
            read_signal(NULL, &mag, NULL, &error, 0);
            mag_sum += mag;
            error_sum += error;
        }
        mag_sum = mag_sum / 10;
        error_sum = error_sum / 10;

        // Accept the highest gain such that reading a valid sinusoid
        if (mag_sum > 0.5 && mag_sum < 2.1 && error_sum < 30)
            break;
    }
    mux_write_to_electrode(SRC, 0, MUX_DIS);  
    mux_write_to_electrode(SINK, 0, MUX_DIS);
    mux_write_to_electrode(VP, 0, MUX_DIS);
    mux_write_to_electrode(VN, 0, MUX_DIS);
}

// public set/get methods 
// set the total number of electrodes for measurement per band
void EITKitArduino::set_num_electrodes(int num_electrodes){
  _num_electrodes = num_electrodes;
}

// get the total number of electrodes for measurement per band
int EITKitArduino::get_num_electrodes(){
  return _num_electrodes;
}

// set the total number of bands used in measurement
void EITKitArduino::set_num_bands(int num_bands){
  _num_bands = num_bands;
}

// get the total number of bands used in measurement
int EITKitArduino::get_num_bands(){
  return _num_bands;
}

// set the measurement protocol: 2-terminal or 4-terminal 
void EITKitArduino::set_num_terminals(int num_terminals){
  _num_terminals = num_terminals;
}

// get the measurement protocol: 2-terminal or 4-terminal 
int EITKitArduino::get_num_terminals(){
  return _num_terminals;
}

// set the protocol for electrodes used in voltage reading 
void EITKitArduino::set_meas_type(Meas_t meas_type){
  _meas_type = meas_type;
}

// get the protocol for electrodes used in voltage reading 
Meas_t EITKitArduino::get_meas_type(){
  return _meas_type;
}

// set the protocol for electrodes used in voltage reading 
void EITKitArduino::set_drive_type(Meas_t drive_type){
  _drive_type = drive_type;
}

// get the protocol for electrodes used in voltage reading 
Meas_t EITKitArduino::get_drive_type(){
  return _drive_type;
}

// set whether to create visualization in 3d 
void EITKitArduino::set_visualize_3d(bool visualize_3d){
  _visualize_3d = visualize_3d;
}

// get whether a visualization in 3d will be created
bool EITKitArduino::get_visualize_3d(){
  return _visualize_3d;
}

// set whether to use built-in calibration 
void EITKitArduino::set_auto_calibration(bool auto_calibration){
  _auto_calibration = auto_calibration;
}

// get whether to built-in calibration will be performed
bool EITKitArduino::get_auto_calibration(){
  return _auto_calibration;
}

/* set the AC current frequency
 default value is 50kHx */
void EITKitArduino::set_current_freq(uint16_t current_freq){
  _current_freq = current_freq;
}

// get the AC current frequency
uint16_t EITKitArduino::get_current_freq(){
  return _current_freq;
}

// set the AC current gain 
void EITKitArduino::set_current_gain(uint16_t current_gain){
  _current_gain = current_gain;
}

// get the AC current gain
uint16_t EITKitArduino::get_current_gain(){
  return _current_gain;
}

// set the voltage gain 
void EITKitArduino::set_voltage_gain(uint16_t voltage_gain){
  _voltage_gain = voltage_gain;
}

// get the voltage gain
uint16_t EITKitArduino::get_voltage_gain(){
  return _voltage_gain;
}

// get magnitude measurements
double* EITKitArduino::get_magnitude_array(){
  return _signal_rms;
}

// get phase measurements
double* EITKitArduino::get_phase_array(){
  return _signal_phase;
}