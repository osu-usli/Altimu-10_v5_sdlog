#include <SPI.h>
#include <Wire.h>
#include <LSM6.h>
#include <LPS.h>
#include "SX128XLT.h"
#include <SoftwareSerial.h>

#include "LowPass.h"

#define SD_CS 9
#define NSS 10
#define RFBUSY 6
#define NRESET 5
#define DIO1 2
#define LED1 4

#define RANGING_TIME 1000
#define IMU_INTERVAL 100

#define IMU_SAMPLES 10
#define ACC_SCALE 2048
#define LAUNCH_THRESHOLD 4.0

#define LORA_DEVICE DEVICE_SX1280                //we need to define the device we are using
const uint32_t Frequency = 2445000000;           //frequency of transmissions in hz
const int32_t Offset = 0;                        //offset frequency in hz for calibration purposes
const uint8_t Bandwidth = LORA_BW_0800;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF8;        //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;            //LoRa coding rate
const uint16_t Calibration = 11350;              //Manual Ranging calibrarion value

const int8_t RangingTXPower = 12;                //Transmit power used
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

  Serial.begin(115200);
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
  static unsigned long last_imu_time;
  static long launch_pressure;
  static LowPass<long, IMU_SAMPLES> pressure;
  static LowPass<float, IMU_SAMPLES> acceleration;
  
  double distance = -1;
  tof.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_MASTER);

  double latest_acc;
  long latest_pressure;

  start_time = millis();
  while (millis() < start_time + RANGING_TIME) {
    tof.transmitRanging(RangingAddress, 500, RangingTXPower, NO_WAIT);

    while (!digitalRead(DIO1)) {
      if (last_imu_time + IMU_INTERVAL < millis()) {
        last_imu_time = millis();
        gyro_acc.readAcc();

        double x = gyro_acc.a.x / (double)ACC_SCALE;
        double y = gyro_acc.a.y / (double)ACC_SCALE;
        double z = gyro_acc.a.z / (double)ACC_SCALE;
        latest_acc = acceleration.update(sqrt(x*x + y*y + z*z));
        latest_pressure = pressure.update(ps.readPressureRaw());

        if (latest_acc > LAUNCH_THRESHOLD) {
          // TODO: send a signal to the Pis
          launch_pressure = latest_pressure;
        }
      }
    }
    
    uint16_t irqStatus = tof.readIrqStatus();
    if (irqStatus & IRQ_RANGING_MASTER_RESULT_VALID) {
      Serial.print("Ranging result valid: ");
      int32_t range_result = tof.getRangingResultRegValue(RANGING_RESULT_RAW);
      distance = tof.getRangingDistance(RANGING_RESULT_RAW, range_result, 1);
      Serial.println(distance);
    } else {
      Serial.println("Ranging result NOT valid");
    }
  }
  
  int temp = ps.readTemperatureRaw();
  
  struct {
    uint32_t marker;
    uint32_t time;
    int32_t pressure;
    double acc;
    int16_t temp;
    double distance;
  } packet = {
    0x5555AAAA, start_time, latest_pressure, latest_acc, temp, distance
  };

  
  tof.setMode(MODE_STDBY_RC);
  tof.setRegulatorMode(USE_LDO);
  tof.setPacketType(PACKET_TYPE_LORA);
  tof.setRfFrequency(Frequency, Offset);
  tof.setBufferBaseAddress(0, 0);
  tof.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
  tof.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 250, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  tof.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
  tof.setHighSensitivity();
  Serial.flush();
  uint8_t len;
  for (int i = 0; i < 10; i++) {
    len = tof.transmit((byte*)&packet, sizeof(packet), 1000, RangingTXPower, NO_WAIT);
  }
  Serial.print("transmitted: ");
  Serial.println(len);
  piserial.write((byte*)&packet, sizeof(packet));
  piserial.flush();
}
