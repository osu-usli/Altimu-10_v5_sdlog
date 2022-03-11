#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <LSM6.h>
//below for magnetometer
#include <LIS3MDL.h>
#include <LPS.h>

LSM6 gyro_acc;
LIS3MDL mag;
LPS ps;

File filep;
//float accel = 3.1459;
//float gyro = 10.2;
unsigned long start_time;
int x=0;
unsigned long startertime;

void setup() {

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
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  while(!Serial){
  }
  if(!mag.init()){
    Serial.println("Magnetometer error!");
  }
  mag.enableDefault();
  if(!ps.init()){
    Serial.println("Barometer error!");
  }
  ps.enableDefault();
  while(1){
  if(!SD.begin(10)){
    Serial.println("Card Error!");
    delay(1000);
  }
  else{
    break;
  }
  }
  Serial.println("Initializing Card...");
  
  filep = SD.open("imu4.txt", FILE_WRITE);
  if(filep){
    Serial.println("Writing to Card...");
  }
  else{
    Serial.println("Initialization failed");
    while(1);
  }
  filep.close();
}

void loop() {
  filep = SD.open("imu.bin", FILE_WRITE);
  startertime = millis();
  for(int i=0;i<100;i++){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    start_time = millis();
    if(start_time - startertime >= 100){
      startertime=start_time;
      mag.read();
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
      int16_t xmag, ymag, zmag;
      int16_t temp;
    } packet = {
      0x5555AAAA, start_time, pressure,
      gyro_acc.a.x, gyro_acc.a.y, gyro_acc.a.z,
      gyro_acc.g.x, gyro_acc.g.y, gyro_acc.g.z,
      mag.m.x, mag.m.y, mag.m.z,
      temp
    };
    filep.write((uint8_t*)&packet, sizeof(packet));
  }
  filep.close();
}
