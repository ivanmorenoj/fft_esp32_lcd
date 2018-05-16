/*
  Example that use fft for show in a lcd display, divided by symmetric bands

  Copyright (c) Ivan Moreno 2018 
  
  This example use the arduinoFFT, you can fond the library in https://github.com/kosme/arduinoFFT
  also use a LiquidCrystal, you can fond the library in https://github.com/arduino-libraries/LiquidCrystal

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Gene ral Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#include "arduinoFFT.h"
#include <LiquidCrystal.h>

const byte characters[8][8] = {  //constantes para la sgram
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0E},
  {0x00,0x00,0x00,0x00,0x00,0x00,0x0E,0x0E},
  {0x00,0x00,0x00,0x00,0x00,0x0E,0x0E,0x0E},
  {0x00,0x00,0x00,0x00,0x0E,0x0E,0x0E,0x0E},
  {0x00,0x00,0x00,0x0E,0x0E,0x0E,0x0E,0x0E},
  {0x00,0x00,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E},
  {0x00,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E},
  {0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E},
};

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
LiquidCrystal lcd(13, 12, 14, 27, 26, 25);  // init the library
/*
These values can be changed in order to evaluate the functions
*/
#define BANDS           20  //num of bands
#define MAX_FRECUENCY   16000
#define CHANNEL         A0

const uint16_t samples = 1024; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 40000; //Hz

unsigned int sampling_period_us;
unsigned long microseconds;
unsigned int *bandFft;  //pointer to bands

const float freqFactor =  1.28;
const float base_freq = MAX_FRECUENCY/pow(freqFactor,BANDS-1);
/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

void setup(){
  Serial.begin(115200);
  bandFft = new unsigned int[BANDS]; // init the vector

  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  
  lcd.begin(20,4);  //init the lcd 20x4 
  for(uint8_t i= 0;i<8;i++){
    lcd.createChar(i, (byte *)characters[i]); //set costom character
  }
  Serial.print("Base Frecuency per Band :> ");
  Serial.println(base_freq);
  lcd.clear();
  for(float y=0,f = base_freq,i=1,last = 10;f<=MAX_FRECUENCY;f*=freqFactor){
    if(++y>4){
      y = 1;
      delay(2000);
      lcd.clear();
    }
    lcd.print("F0 BW");
    lcd.print(int(i++));
    lcd.print(": ");
    /*lcd.print(int(last));
    lcd.print(" <=> ");
    lcd.print(int(f));
    lcd.print(" >> ");*/
    lcd.print(int(last+(f-last)/2.0));
    lcd.print(" Hz");

    lcd.setCursor(0,int(y));
    last = f;
  }
  delay(1000);
}

void loop()
{
  /*SAMPLING*/
  for(int i=0; i<samples; i++)
  {
      microseconds = micros();    //Overflows after around 70 minutes!

      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() < (microseconds + sampling_period_us)){
      }
  }
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  
  //double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  
  //Serial.print("frequency most dominant >> ");
  //Serial.println(x, 6); //Print out what frequency is the most dominant.

  calcAvgbyBand(vReal,samples>>1,bandFft);
  //showFftband(bandFft);
  //Serial.println("<<=======================================================>>");
  normalizeBand(bandFft,100,17000,32);
  //showFftband(bandFft);

  set_level(bandFft);

  //while(1);
  delay(70);
}
void showFftband(unsigned int *ptrBand){
  uint16_t freq = base_freq;
  for (uint8_t i = 0; i < BANDS; ++i){
    Serial.print(" Frecuency ");
    Serial.print(freq - base_freq/2);
    Serial.print(" ->  ");
    Serial.println(bandFft[i]);
    freq = base_freq * (i+2);
  }
}
void calcAvgbyBand(double *ptrData,uint16_t bufSize,unsigned int *ptrBand){
  double average=0;
  float freq,freqbase = base_freq;
  uint16_t nSamp=0;
  uint8_t j=0;
  for(uint16_t i = 2; i<bufSize; i++){
      freq = (i * 1.0 * samplingFrequency) / samples;
      average += ptrData[i];
      nSamp++;
      if(freq > freqbase){
        average /= 1.0 + nSamp;          
        ptrBand[j++] = int(average);
        freqbase *= freqFactor;
        average = 0;
        nSamp = 0;
      }
  }
  average /= 1.0 * nSamp;
  ptrBand[j] = int(average);
}
void normalizeBand(unsigned int *ptrBand,unsigned int noise,unsigned int max_input,uint8_t max_output){
  unsigned int aux;
  for (uint8_t i = 0; i < BANDS; i++){
    aux = ptrBand[i];
    if (aux > noise){
      aux = aux > max_input? max_input : aux - noise;
      ptrBand[i] = (aux * max_output) / max_input;
    }
    else
      ptrBand[i] = 0;
  }
}
void set_level(unsigned int *ptr){
  byte a,n,x,y;
  for(y=0;y<4;++y){
    lcd.setCursor(0,y);
    for (x = 0; x < 20; ++x){
      n = (byte)*(ptr + x );
      a = n>>3;
      if (a == 3-y){
        n = n>=8? n - (n>>3)*8  : n;
        n = n>0? n-1 : ' ';
      }else
        n = a>=(3-y)? 7:' ';
      lcd.write(n);
    }
  }
}
double pow(double n,uint8_t e){
  double aux = n;
  for (int i = 0; i < e; ++i){
    aux*=n;
  }
  return aux;
}