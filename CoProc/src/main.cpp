/*
 * Coprocessor code to handle quadrature encoder inputs 
 * and make encoder position available to the main processor 
 * over an I2C interface.
 * 
 * Written for RoboPuppet MQP 2019-2020
 * by Michael Sidler
 */

#include <Arduino.h>
#include <i2c_t3.h>
#include <Encoder.h>
#include <ECP_Registers.h>

const uint8_t DUMMY_MODE = 1;

const uint8_t DEBUG_PIN_REC = 31;
const uint8_t DEBUG_PIN_REQ = 32;

const uint8_t ENCODER0_PINA = 0;
const uint8_t ENCODER0_PINB = 1;
const uint8_t ENCODER0_PINX = 2;
const uint8_t ENCODER1_PINA = 3;
const uint8_t ENCODER1_PINB = 4;
const uint8_t ENCODER1_PINX = 5;
const uint8_t ENCODER2_PINA = 6;
const uint8_t ENCODER2_PINB = 7;
const uint8_t ENCODER2_PINX = 8;
const uint8_t ENCODER3_PINA = 9;
const uint8_t ENCODER3_PINB = 10;
const uint8_t ENCODER3_PINX = 11;
const uint8_t ENCODER4_PINA = 12;
const uint8_t ENCODER4_PINB = 13;
const uint8_t ENCODER4_PINX = 14;
const uint8_t ENCODER5_PINA = 15;
const uint8_t ENCODER5_PINB = 16;
const uint8_t ENCODER5_PINX = 17;
// SDA = pin 18
// SCL = pin 19

Encoder enc0(ENCODER0_PINA,ENCODER0_PINB,ENCODER0_PINX);
Encoder enc1(ENCODER1_PINA,ENCODER1_PINB,ENCODER1_PINX);
Encoder enc2(ENCODER2_PINA,ENCODER2_PINB,ENCODER2_PINX);
Encoder enc3(ENCODER3_PINA,ENCODER3_PINB,ENCODER3_PINX);
Encoder enc4(ENCODER4_PINA,ENCODER4_PINB,ENCODER4_PINX);
Encoder enc5(ENCODER5_PINA,ENCODER5_PINB,ENCODER5_PINX);

uint8_t registerRequested[1];

void E0A_ISR() {enc0.changeA();}
void E0B_ISR() {enc0.changeB();}
void E0X_ISR() {enc0.changeX();}
void E1A_ISR() {enc1.changeA();}
void E1B_ISR() {enc1.changeB();}
void E1X_ISR() {enc1.changeX();}
void E2A_ISR() {enc2.changeA();}
void E2B_ISR() {enc2.changeB();}
void E2X_ISR() {enc2.changeX();}
void E3A_ISR() {enc3.changeA();}
void E3B_ISR() {enc3.changeB();}
void E3X_ISR() {enc3.changeX();}
void E4A_ISR() {enc4.changeA();}
void E4B_ISR() {enc4.changeB();}
void E4X_ISR() {enc4.changeX();}
void E5A_ISR() {enc5.changeA();}
void E5B_ISR() {enc5.changeB();}
void E5X_ISR() {enc5.changeX();}


void I2C_REG_SET(size_t numBytes) {
  digitalWrite(DEBUG_PIN_REC,HIGH);
  Wire.read(registerRequested, numBytes);
  digitalWrite(DEBUG_PIN_REC,LOW);
}

void I2C_SEND() {
  Serial.println("SEND");
  Serial.print("    regRequested: ");
  Serial.println((char)registerRequested[0],HEX);
  switch(registerRequested[0]) {
    case REG_E0_ANGLE_MSB:
      if(DUMMY_MODE) Wire.write(REG_E0_ANGLE_MSB);
      else Wire.write((uint8_t)(enc0.getPos()>>8));
      break;
    case REG_E0_ANGLE_LSB:
      if(DUMMY_MODE) Wire.write(REG_E0_ANGLE_LSB);
      else Wire.write((uint8_t)(enc0.getPos()));
      break;
    case REG_E1_ANGLE_MSB:
      if(DUMMY_MODE) Wire.write(REG_E1_ANGLE_MSB);
      else Wire.write((uint8_t)(enc1.getPos()>>8));
      break;
    case REG_E1_ANGLE_LSB:
      if(DUMMY_MODE) Wire.write(REG_E1_ANGLE_LSB);
      else Wire.write((uint8_t)(enc1.getPos()));
      break;
    case REG_E2_ANGLE_MSB:
      if(DUMMY_MODE) Wire.write(REG_E2_ANGLE_MSB);
      else Wire.write((uint8_t)(enc2.getPos()>>8));
      break;
    case REG_E2_ANGLE_LSB:
      if(DUMMY_MODE) Wire.write(REG_E2_ANGLE_LSB);
      else Wire.write((uint8_t)(enc2.getPos()));
      break;
    case REG_E3_ANGLE_MSB:
      if(DUMMY_MODE) Wire.write(REG_E3_ANGLE_MSB);
      else Wire.write((uint8_t)(enc3.getPos()>>8));
      break;
    case REG_E3_ANGLE_LSB:
      if(DUMMY_MODE) Wire.write(REG_E3_ANGLE_LSB);
      else Wire.write((uint8_t)(enc3.getPos()));
      break;
    case REG_E4_ANGLE_MSB:
      if(DUMMY_MODE) Wire.write(REG_E4_ANGLE_MSB);
      else Wire.write((uint8_t)(enc4.getPos()>>8));
      break;
    case REG_E4_ANGLE_LSB:
      if(DUMMY_MODE) Wire.write(REG_E4_ANGLE_LSB);
      else Wire.write((uint8_t)(enc4.getPos()));
      break;
    case REG_E5_ANGLE_MSB:
      if(DUMMY_MODE) Wire.write(REG_E5_ANGLE_MSB);
      else Wire.write((uint8_t)(enc5.getPos()>>8));
      break;
    case REG_E5_ANGLE_LSB:
      if(DUMMY_MODE) Wire.write(REG_E5_ANGLE_LSB);
      else Wire.write((uint8_t)(enc5.getPos()));
      break;
    case REG_CAL_FINISHED:
      if(DUMMY_MODE) Wire.write(REG_CAL_FINISHED);
      else {
        uint8_t calState = 0b11000000;
        calState |= (enc0.isCalibrated());
        calState |= (enc1.isCalibrated()<<1);
        calState |= (enc2.isCalibrated()<<2);
        calState |= (enc3.isCalibrated()<<3);
        calState |= (enc4.isCalibrated()<<4);
        calState |= (enc5.isCalibrated()<<5);
        Wire.write(calState);
      }
      break;
    default:
      Wire.write(0xFA);
      break;
  }
}

void setup() {

  registerRequested[0] = 0x00;

  pinMode(DEBUG_PIN_REC,OUTPUT);
  pinMode(DEBUG_PIN_REQ,OUTPUT);

  //Serial.begin(9600);

  //while(!Serial.available()){}
  //delay(5000);
  //Serial.println("Starting Setup");

  // Attach all of the interrupts to ISRs
  attachInterrupt(digitalPinToInterrupt(ENCODER0_PINA), E0A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER0_PINB), E0B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER0_PINX), E0X_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PINA), E1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PINB), E1B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PINX), E1X_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PINA), E2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PINB), E2B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PINX), E2X_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_PINA), E3A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_PINB), E3B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_PINX), E3X_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_PINA), E4A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_PINB), E4B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_PINX), E4X_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER5_PINA), E5A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER5_PINB), E5B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER5_PINX), E5X_ISR, CHANGE);


  // Join the I2C bus with the specified address
  Wire.begin(I2C_SLAVE,SLAVE_ADDR,I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);

  // Register a callback for receiving data from the I2C bus
  Wire.onReceive(I2C_REG_SET);

  // Register a callback for when the master device wants data
  Wire.onRequest(I2C_SEND);

  //Serial.println("Finished Setup");
}

void loop() {
  // everything is done in ISRs
}