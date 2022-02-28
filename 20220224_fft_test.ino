// Filename: 20220224_fft_test
// Description: Testing Arduino as a guitar tuner capability.
// Primary Author: Sky Hoffert
// Secondary Author(s): N/A
// Last Modified: 20220224
//
// Notes: https://docs.google.com/document/d/13gvAUSCh_C5QKtJx3WS2gAYvz4HuGyKUleX1ZsC2ewQ/edit#
// 

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

int sComplexSet(struct sComplex* dest, float r, float i) {
  dest->real = r;
  dest->imag = i;
  return 0;
}

float sComplexMag(const struct sComplex* a) {
  return pow( (a->real * a->real) + (a->imag * a->imag), 0.5);
}

float sComplexAngle(const struct sComplex* a) {
  return atan2(a->imag, a->real);
}

#define k_num_reads 400

int interval;
int i_read;
uint8_t vals[k_num_reads];

void setup()
{

  // TODO
  Serial.begin(9600);

  i_read = 0;
  interval = millis();
  
}

void loop()
{

  vals[i_read] = ((float)analogRead(A0));
  i_read++;

  if (i_read >= k_num_reads) {
    i_read = 0;
    interval = millis() - interval;

//    Serial.print("Duration: ");
//    Serial.print(interval);
//    Serial.print(" ms for ");
//    Serial.print(k_num_reads);
//    Serial.print(" reads.\n");
    
    // Perform FFT, write to file.
    int len = k_num_reads;
    int N = 1024;
    sComplex S[N];

    // DFT
    for (int i = 0; i < N; i++) {
      sComplex sum;
      sComplexSet(&sum, 0, 0);
      for (int s = 0; s < k_num_reads; s++) {
        sComplex re_im;
        re_im.real = sig[s] * sCos(2 * sPi * i * s / n_fft);
        re_im.imag = sig[s] * -sSin(2 * sPi * i * s / n_fft);
        sComplexAdd(&sum, &re_im);
      }
      res[i].real = sum.real;
      res[i].imag = sum.imag;
    }
    // DFT
  
    for (int i = 0; i < len; i++) {
      float mag = 10 * sLog10(sComplexMag(&S[i]));
//      fprintf(fout, "%d,%3.3f,\n", i, mag);
    }
    
    interval = millis();
  }
  
}
