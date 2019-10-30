/* Encoder.h
 *  
 * Encoder Library
 *  
 * Written by Michael Sidler
 * 
 */

#include "Arduino.h"
#include "Encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB, uint8_t pinX) {
  this->pinA = pinA;
  this->pinB = pinB;
  this->pinX = pinX;
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode(pinX, INPUT);
}

int16_t Encoder::getPos(){
  return pos;
}

void Encoder::changeA() {
  if (digitalRead(pinA)) { 
    if (digitalRead(pinB)) {
      pos++; // A==HIGH && B==HIGH : CW 
    }
    else {
      pos--; // A==HIGH && B==LOW : CCW
    }
  }
  else { 
    if (digitalRead(pinB)) {
      pos--; // A==LOW && B==HIGH : CCW
    }
    else {
      pos++; // A==LOW && B==LOW : CW
    }
  }
}

void Encoder::changeB() {
  if (digitalRead(pinB)) {   
    if (digitalRead(pinA)) {
      pos--; // B==HIGH && A==HIGH : CCW
    }
    else {
      pos++; // B==HIGH && A==LOW : CW
    }
  }
  else { 
    if (digitalRead(pinA)) {
      pos++; // B==LOW && A==HIGH : CW
    }
    else {
      pos--; // B==LOW && A==LOW : CCW
    }
  }
}

void Encoder::changeX() {
  if(!calibrated) {
    calibrated = true;
    pos = 0;
  }
}

bool Encoder::isCalibrated() {
  return calibrated;
}

/* Quadrature Encoder Waveforms
     _____       _____
A   |     |     |     |
____|     |_____|     |_
        _____       _____
B      |     |     |     |
   ____|     |_____|     |_
--> CW
<-- CCW
*/