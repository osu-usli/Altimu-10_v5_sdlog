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

float gyro[3];
float accel[3];

int gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z; 

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
  if(!SD.begin(10)){
    Serial.println("Card Error!");
      while(1){};
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
  //filep = SD.open("imu4.txt", FILE_WRITE);
//  SD.remove("imu4.txt");
}
void loop() {
  filep = SD.open("imu4.txt", FILE_WRITE);
  startertime = millis();
  for(int i=0;i<100;i++){
    start_time = millis();
    if(start_time - startertime >= 100){
      startertime=start_time;
      mag.read();
    }
//    Serial.println("Gyro");
    gyro_acc.readGyro();
//   gyro_x = gyro_acc.g.x;
//   gyro_y = gyro_acc.g.y;
//   gyro_z = gyro_acc.g.z;

//    Serial.println("Accel");
    gyro_acc.readAcc();
//   accel_x = gyro_acc.a.x >> 4;
//   accel_y = gyro_acc.a.y >> 4;
//   accel_z = gyro_acc.a.z >> 4;
  
    //x
    accel[0] = gyro_acc.a.x;
    //y
    accel[1] = gyro_acc.a.y;
    //z
    accel[2] = gyro_acc.a.z;

    //x
    gyro[0] = gyro_acc.g.x;
    //y
    gyro[1] = gyro_acc.g.y;
    //z
    gyro[2] = gyro_acc.g.z;

    long pressure = ps.readPressureRaw();
    int temp = ps.readTemperatureRaw();

//    start_time=start_time+","+accel+","+gyro;
//    filep.print(start_time);
//    filep.print(",");
    filep.println((String)start_time+","+accel[0]+","+accel[1]+","+accel[2]+","+gyro[0]+","+gyro[1]+","+gyro[2] + "," + pressure + "," + temp + "," + mag.m.x + "," + mag.m.y + "," + mag.m.z);
//    filep.print(accel[1]);
//    filep.print(",");
//    filep.print(accel[2]);
//    filep.print(",");
//    filep.print(gyro[0]);
//    filep.print(",");
//    filep.print(gyro[1]);
//    filep.print(",");
//    filep.println(gyro[2]);
//    Serial.println(millis());
  }
  //start_time = millis();
//  filep = SD.open("imu2.txt", FILE_WRITE);
//  filep.print(start_time);
//  filep.print(",");
//  filep.print(accel);
//  filep.print(",");
//  filep.println(gyro);
 // start_time=start_time+","+accel+","+gyro;
//  filep.println(start_time);
//  Serial.println(millis());
  filep.close();
}
