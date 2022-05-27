#include <SPI.h>
#include <Wire.h>
#include <LSM6.h>
#include <LPS.h>
#include "SX128XLT.h"
#include <SoftwareSerial.h>
#include <limits.h>

#include "LowPass.h"

#define SD_CS 9
#define NSS 10
#define RFBUSY 6
#define NRESET 5
#define DIO1 2
#define LED1 4

#define IMU_INTERVAL 100

// FIXME: the following line has an integer overflow and should be changed to (1000L*150).
// This bug caused the timeout to fire after about 18 seconds, rather than the full 2.5 minutes.
// In the interest of preserving the competition behavior, this line has not been updated. However,
// if a future team reuses this code -- be sure to fix it!
#define LANDING_TIMEOUT (1000*150)

#define IMU_SAMPLES 10
#define ACC_SCALE 2048
#define LAUNCH_THRESHOLD_G 4.0
#define LANDING_THRESHOLD_MIN_G 0.8
#define LANDING_THRESHOLD_MAX_G 1.2
#define LANDING_THRESHOLD_NUM_SAMPLES 100

// The horizontal displacement from the launchpad to the tent in feet (east is positive)
#define TENT_X 0

// The vertical displacement from the launchpad to the tent in feet (south is positive)
#define TENT_Y -350

#define GRID_SQUARE_SIZE_FEET 250
#define FIELD_SIZE_SQUARES 20

#define LORA_DEVICE DEVICE_SX1280                //we need to define the device we are using
const uint32_t Frequency = 2445000000;           //frequency of transmissions in hz
const int32_t Offset = 0;                        //offset frequency in hz for calibration purposes
const uint8_t Bandwidth = LORA_BW_0800;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF8;        //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;            //LoRa coding rate
const uint16_t Calibration = 11350;              //Manual Ranging calibrarion value

const int8_t RangingTXPower = 12;                //Transmit power used
const uint32_t RangingAddress = 16;              //must match address in recever


SoftwareSerial piserial1(A0, A1);
SoftwareSerial piserial2(7, 8);

#define PI1_FLAG A3
#define PI2_FLAG A2

LSM6 gyro_acc;
SX128XLT tof;

void setup() {
  // Tell the Pis to stand by waiting for launch
  pinMode(PI1_FLAG, OUTPUT);
  digitalWrite(PI1_FLAG, LOW);
  pinMode(PI2_FLAG, OUTPUT);
  digitalWrite(PI2_FLAG, LOW);

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
  while (!Serial) {
  }

  if (!tof.begin(NSS, NRESET, RFBUSY, DIO1, DEVICE_SX1280)) {
    Serial.println("ToF error");
    Serial.flush();
    exit(0);
  }

  tof.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_MASTER);
}

void loop() {
  static unsigned long last_imu_time;
  static bool launched;
  static LowPass<float, IMU_SAMPLES> acceleration;
  static LowPass<float, IMU_SAMPLES> distance;
  static unsigned landing_samples;
  static double latest_acc;
  static double latest_distance;
  static unsigned long launch_time;
  
  tof.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_MASTER);


  // Transmit a ToF packet.
  tof.transmitRanging(RangingAddress, 500, RangingTXPower, NO_WAIT);

  // Until we recieve a ranging response, or we timeout...
  while (!digitalRead(DIO1)) {
    // Should we poll the IMU?
    if (last_imu_time + IMU_INTERVAL < millis()) {
      last_imu_time = millis();
      gyro_acc.readAcc();

      // Compute total acceleration and run it through the low-pass filter.
      double x = gyro_acc.a.x / (double)ACC_SCALE;
      double y = gyro_acc.a.y / (double)ACC_SCALE;
      double z = gyro_acc.a.z / (double)ACC_SCALE;
      latest_acc = acceleration.update(sqrt(x * x + y * y + z * z));

      if (!launched && latest_acc > LAUNCH_THRESHOLD_G) {
        // Acceleration exceeds the launch threshold; tell the Pis to start capturing images.
        digitalWrite(PI1_FLAG, HIGH);
        digitalWrite(PI2_FLAG, HIGH);
        Serial.print("Launched: ");
        Serial.println(latest_acc);
        launched = true;
        launch_time = millis();
      } else if (launched && latest_acc > LANDING_THRESHOLD_MIN_G && latest_acc < LANDING_THRESHOLD_MAX_G) {
        // Acceleration is within the landing range.
        if (++landing_samples > LANDING_THRESHOLD_NUM_SAMPLES) {
          // Acceleration has been within the landing range for a while now.
          // we've landed!
          Serial.println("Landed");
          landed(latest_distance);
        }
      } else {
        landing_samples = 0;
      }
    }

    if (launched && millis() - launch_time > LANDING_TIMEOUT) {
      // We haven't detect the landing, but it's so long since launch we must have landed by now.
      Serial.println("Launch timeout");
      landed(latest_distance);
    }
  }

  // What was the result of the ToF ranging?
  uint16_t irqStatus = tof.readIrqStatus();
  if (irqStatus & IRQ_RANGING_MASTER_RESULT_VALID) {
    Serial.print("Ranging result valid: ");
    int32_t range_result = tof.getRangingResultRegValue(RANGING_RESULT_RAW);
    float distance_sample = tof.getRangingDistance(RANGING_RESULT_RAW, range_result, 1);
    distance_sample = distance_sample * 10 / 3; // convert to feet
    Serial.println(distance_sample);
    latest_distance = distance.update(distance_sample);
  } else {
    Serial.println("Ranging result NOT valid");
  }
}

void landed(float tof_distance) {
  // Tell the Pis to start computing the result.
  digitalWrite(PI1_FLAG, LOW);
  digitalWrite(PI2_FLAG, LOW);

  uint32_t colors[3] = {0};
  uint32_t colors_2[3] = {0};

  Serial.println("Waiting for pi1...");
  // pi1 will print once its flag goes high
  // in case something is wrong/desynced, pulse its flag to make sure it eventually gets the message
  piserial1.begin(9600);
  while (!piserial1.available()) {
    digitalWrite(PI1_FLAG, !digitalRead(PI1_FLAG));
    delay(1000);
  }
  piserial1.readBytes((byte*)colors, sizeof(colors));

  Serial.println("Waiting for pi2...");
  piserial2.begin(9600);
  while (!piserial2.available()) {
    digitalWrite(PI2_FLAG, !digitalRead(PI2_FLAG));
    delay(1000);
  }
  piserial2.readBytes((byte*)colors_2, sizeof(colors_2));

  for (int i = 0; i < 3; i++) colors[i] += colors_2[i];

  // Colors are returned in the order green (west-facing), orange (north), blue (east)
  // Choose the best adjacent pair
  float color_a;
  float color_b;
  float angle;
  if (colors[0] + colors[1] > colors[1] + colors[2]) {
    Serial.println("Using green & orange");
    color_a = colors[0];
    color_b = colors[1];
    angle = 0;
  } else {
    Serial.println("Using orange & blue");
    color_a = colors[1];
    color_b = colors[2];
    angle = M_PI / 2;
  }

  // If we're facing fully towards color_a, angle += 0
  // If we're facing fully towards color_b, angle += 90
  angle += atan2(color_b, color_a);

  float x = TENT_X - tof_distance * cos(angle);
  float y = TENT_Y - tof_distance * sin(angle);

  int grid_square_x = round(x / GRID_SQUARE_SIZE_FEET) + FIELD_SIZE_SQUARES / 2;
  int grid_square_y = round(y / GRID_SQUARE_SIZE_FEET) + FIELD_SIZE_SQUARES / 2;

  int16_t grid_square = grid_square_y * FIELD_SIZE_SQUARES + grid_square_x + 1; // 1-based index

  Serial.print("gridsquare "); Serial.print(grid_square);
  Serial.print(" ("); Serial.print(grid_square_x);
  Serial.print(", "); Serial.print(grid_square_y); Serial.println(")");

  // Switch the radio to data mode.
  tof.setMode(MODE_STDBY_RC);
  tof.setRegulatorMode(USE_LDO);
  tof.setPacketType(PACKET_TYPE_LORA);
  tof.setRfFrequency(Frequency, Offset);
  tof.setBufferBaseAddress(0, 0);
  tof.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
  tof.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 250, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  tof.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
  tof.setHighSensitivity();

  // Repeatedly transmit the grid square until somebody powers us off.
  for (;;) {
    tof.transmit((byte*)&grid_square, sizeof(grid_square), 1000, RangingTXPower, WAIT_TX);
  }
}
