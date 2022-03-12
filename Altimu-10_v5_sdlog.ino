#include <SPI.h>
#include <Wire.h>
#include <LSM6.h>
#include <LPS.h>
#include "SX128XLT.h"
#include <SoftwareSerial.h>

#define SD_CS 9
#define NSS 10
#define RFBUSY 6
#define NRESET 5
#define DIO1 2
#define LED1 4

#define LORA_DEVICE DEVICE_SX1280                //we need to define the device we are using
const uint32_t Frequency = 2445000000;           //frequency of transmissions in hz
const int32_t Offset = 0;                        //offset frequency in hz for calibration purposes
const uint8_t Bandwidth = LORA_BW_0800;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF8;        //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;            //LoRa coding rate
const uint16_t Calibration = 11350;              //Manual Ranging calibrarion value

const int8_t RangingTXPower = 10;                //Transmit power used
const uint32_t RangingAddress = 16;              //must match address in recever


SoftwareSerial piserial(A0, A1);

LSM6 gyro_acc;
LPS ps;
SX128XLT tof;

//float accel = 3.1459;
//float gyro = 10.2;
unsigned long start_time;
int x=0;
unsigned long startertime;

void setup() {
  piserial.begin(9600);

  //I2C_Init();
  Wire.begin();

  //accel init
  gyro_acc.init();
  gyro_acc.enableDefault();
  gyro_acc.writeReg(LSM6::CTRL1_XL, 0x34);

  //gyro init
  gyro_acc.writeReg(LSM6::CTRL2_G, 0x4C);

//   gyro_acc.readGyro();
//   gyro_x = gyro_acc.g.x;
//   gyro_y = gyro_acc.g.y;
//   gyro_z = gyro_acc.g.z;
//
//  gyro_acc.readAcc();
//  accel_x = gyro_acc.a.x >> 4;
//  accel_y = gyro_acc.a.y >> 4;
//  accel_z = gyro_acc.a.z >> 4;
//  
  Serial.begin(9600);
  SPI.begin();
  while(!Serial){
  }

  if (!tof.begin(NSS, NRESET, RFBUSY, DIO1, DEVICE_SX1280)) {
    Serial.println("ToF error");
    Serial.flush();
    exit(0);
  }

  tof.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_MASTER);
  
  if(!ps.init()){
    Serial.println("Barometer error!");
  }
  ps.enableDefault();
}

void loop() {
  double distance = -1;
  tof.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_MASTER);

  start_time = millis();
  for (int i = 0; i < 10; i++) {
    tof.transmitRanging(RangingAddress, 2000, RangingTXPower, WAIT_TX);
    uint16_t irqStatus = tof.readIrqStatus();
    if (irqStatus & IRQ_RANGING_MASTER_RESULT_VALID) {
      Serial.print("Ranging result valid: ");
      int32_t range_result = tof.getRangingResultRegValue(RANGING_RESULT_RAW);
      distance = tof.getRangingDistance(RANGING_RESULT_RAW, range_result, 1);
      Serial.println(distance);
      break;
    } else {
      Serial.println("Ranging result NOT valid");
    }
  }
  
  gyro_acc.readGyro();
  gyro_acc.readAcc();
  long pressure = ps.readPressureRaw();
  int temp = ps.readTemperatureRaw();
  struct {
    uint32_t marker;
    uint32_t time;
    int32_t pressure;
    int16_t xacc, yacc, zacc;
    int16_t xgy, ygy, zgy;
    int16_t temp;
    double distance;
  } packet = {
    0x5555AAAA, start_time, pressure,
    gyro_acc.a.x, gyro_acc.a.y, gyro_acc.a.z,
    gyro_acc.g.x, gyro_acc.g.y, gyro_acc.g.z,
    temp, distance
  };

  
  tof.setMode(MODE_STDBY_RC);
  tof.setRegulatorMode(USE_LDO);
  tof.setPacketType(PACKET_TYPE_LORA);
  tof.setRfFrequency(Frequency, Offset);
  tof.setBufferBaseAddress(0, 0);
  tof.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
  tof.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  tof.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
  tof.setHighSensitivity();
  Serial.flush();
  uint8_t len = tof.transmit((byte*)&packet, sizeof(packet), 1000, RangingTXPower, WAIT_TX);
  Serial.print("transmitted: ");
  Serial.println(len);
  piserial.write((byte*)&packet, sizeof(packet));
  piserial.flush();
}
