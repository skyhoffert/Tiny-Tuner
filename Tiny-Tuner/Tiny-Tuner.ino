// Filename: Tiny-Tuner.ino
// Description: Arduino as a guitar tuner using 23LC1024 SRAM chip.
// Primary Author: Sky Hoffert
// Secondary Author(s): N/A
// Last Modified: 20220301
//
// Notes: https://docs.google.com/document/d/
//   13gvAUSCh_C5QKtJx3WS2gAYvz4HuGyKUleX1ZsC2ewQ/edit#
// 

///////////////////////////////////////////////////////////////////////////////
// SRAMsimple

#include <Arduino.h>
#include <SPI.h>

/************SRAM opcodes: commands to the 23LC1024 memory chip **************/
#define RDMR        5       // Read the Mode Register
#define WRMR        1       // Write to the Mode Register
#define READ        3       // Read command
#define WRITE       2       // Write command
#define RSTIO     0xFF      // Reset memory to SPI mode
#define ByteMode    0x00    // Byte mode (read/write one byte at a time)
#define Sequential  0x40    // Sequential mode (read/write blocks of memory)

byte CS = 10;               // Global variable for CS pin (default 10)

void SetMode(char Mode){  // Select single or mult byte transfer
  pinMode(CS, OUTPUT);                // set CS pin to output mode
  digitalWrite(CS, LOW);              // set SPI slave select LOW
  SPI.transfer(WRMR);                 // command to write to mode register
  SPI.transfer(Mode);                 // set for sequential mode
  digitalWrite(CS, HIGH);             // release chip select to finish command
}

void WriteFloat(uint32_t address, float data){
  SetMode(Sequential);             // set to tx/rx multiple bytes of data
  byte *temp=(byte *)&data;           // split float into 4 bytes
  digitalWrite(CS, LOW);              // start new command sequence
  SPI.transfer(WRITE);                // send WRITE command
  SPI.transfer((byte)(address >> 16));// send high byte of address
  SPI.transfer((byte)(address >> 8)); // send middle byte of address
  SPI.transfer((byte)address);        // send low byte of address
  SPI.transfer(temp, 4);              // transfer an array of data
  digitalWrite(CS, HIGH);             // set SPI slave select HIGH
}

float ReadFloat(uint32_t address){
  SetMode(Sequential);             // set to tx/rx multiple bytes of data
  byte temp[4];                       // temp array of bytes with 4 elements
  float data=0;
  digitalWrite(CS, LOW);              // start new command sequence
  SPI.transfer(READ);                 // send READ command
  SPI.transfer((byte)(address >> 16));// send high byte of address
  SPI.transfer((byte)(address >> 8)); // send middle byte of address
  SPI.transfer((byte)address);        // send low byte of address
  for(uint16_t i=0; i<4; i++){
    temp[i] = SPI.transfer(0x00);     // read the data byte
  }
  digitalWrite(CS, HIGH);             // set SPI slave select HIGH
  data = *(float *)&temp;             // 4 bytes to a float.
  return data;                // https://www.microchip.com/forums/m590535.aspx
}

// SRAMsimple
///////////////////////////////////////////////////////////////////////////////
// Main Program

#include <math.h>

#define sPi 3.1415926

inline float sCos(float val) {
  return cos(val);
}

inline float sSin(float val) {
  return sin(val);
}

inline float sLog10(float val) {
  return log10(val);
}

struct sComplex {
  float real;
  float imag;
};

int sComplexAdd(struct sComplex* dest, const sComplex* b) {
  dest->real += b->real;
  dest->imag += b->imag;
  return 0;
}

float sComplexMag(const struct sComplex* a) {
  return pow( (a->real * a->real) + (a->imag * a->imag), 0.5);
}

// float sComplexAngle(const struct sComplex* a) {
//   return atan2(a->imag, a->real);
// }

// Number of times to read from pin before computing fft.
// TODO: reduce this number to something that corresponds to 0.1 second ish?
#define k_num_reads 128
// TODO: is this number sufficient for "literal" fine tuning?
//   Larger dft means longer compute time.
#define k_dft_N 128

// Size of a memory block, 4 bytes per float entry.
#define k_mem_block_size_reads k_num_reads*4
#define k_mem_block_size_dft k_dft_N*4

// Starting addresses of various data sections.
#define k_addr_reads 0
#define k_addr_dft_real k_addr_reads + k_mem_block_size_reads
#define k_addr_dft_imag k_addr_dft_real + k_mem_block_size_dft
#define k_addr_dft_mag k_addr_dft_imag + k_mem_block_size_dft
#define k_addr_dft_freqs k_addr_dft_mag + k_mem_block_size_dft

int interval;
int i_read;

int board_alive = 0;

void Test23LC1024() {
  board_alive = 0;
  float test_float = sPi;
  WriteFloat(0, test_float);
  float readval = ReadFloat(0);

  if (test_float - readval < 0.001) {
    board_alive = 1;
    Serial.println("Board confirmed.");
  }
}

void setup()
{

  Serial.begin(9600);

  SPI.begin();

  i_read = 0;
  interval = millis();

  Test23LC1024();
  
}

void loop()
{
  
  if (board_alive == 0) {
    delay(2000);
    Serial.println("Couldn't connect to SRAM chip.");
    Test23LC1024();
    return;
  }

  int addr = k_addr_reads + i_read*4; // 4 bytes per entry.
  i_read++;

  float reading = ((float)analogRead(A0));
  WriteFloat(addr, reading);

  // If signal buffer is not full, continue reading.
  if (i_read < k_num_reads) {
    return;
  }

  Serial.println("running FFT.");

  i_read = 0; // Reset this value back to 0 for next run.
  interval = millis() - interval;

  // DFT
  for (int i = 0; i < k_dft_N; i++) {
    sComplex sum;
    sum.real = 0;
    sum.imag = 0;

    for (int s = 0; s < k_num_reads; s++) {
      sComplex re_im;
      float sigval = ReadFloat(s*4);

      // These casts may be unnecessary.
      float f_i = ((float)i);
      float f_s = ((float)s);
      float f_N = ((float)k_dft_N);

      // FFT bin computation.
      re_im.real = sigval * sCos(2 * sPi * f_i * f_s / f_N);
      re_im.imag = sigval * -sSin(2 * sPi * f_i * f_s / f_N);
      sComplexAdd(&sum, &re_im);
    }

    addr = k_addr_dft_real + i*4;
    WriteFloat(addr, sum.real);
    addr = k_addr_dft_imag + i*4;
    WriteFloat(addr, sum.imag);
  }
  // DFT

  Serial.println("DFT complete.");

  float fs = 0;

  // Compute 10*Log_10 (magnitude) of FFT bins.
  for (int i = 0; i < k_dft_N; i++) {
    sComplex s;
    addr = k_addr_dft_real + i*4;
    s.real = ReadFloat(addr);
    addr = k_addr_dft_imag + i*4;
    s.imag = ReadFloat(addr);

    // Compute mag.
    float mag = 10 * sLog10(sComplexMag(&s));
    addr = k_addr_dft_mag + i*4;
    WriteFloat(addr, mag);

    // Compute freq axis values.
    fs = k_num_reads / (((float)interval)/1000); // samples per second
    float f = i * fs / k_dft_N;
    addr = k_addr_dft_freqs + i*4;
    WriteFloat(addr, f);

    // DEBUG: Write data on serial line.
    Serial.print(f);
    Serial.print(" ");
    Serial.print(mag);
    Serial.print("\n");
  }

  Serial.print("fs = ");
  Serial.print(fs);
  Serial.println("");
  Serial.println("Mag complete.");

  // TODO: Decide things for which freq is present.
  
  interval = millis();
  
}

// Main Program
///////////////////////////////////////////////////////////////////////////////
