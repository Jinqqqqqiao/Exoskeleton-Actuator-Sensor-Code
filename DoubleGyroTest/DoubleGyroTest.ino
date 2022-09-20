/***************************************************
Please Check The performance of two gyroscope first
Find out drift of sensors
Check the performance of low pass filter
 **************************************************/

#include <Wire.h>
const int MPU1 = 0x68;                             // MPU6050 I2C Address(leg)
const int MPU2 = 0x69;                             // MPU6050 I2C Address(foot)
float AccX1, AccY1, AccZ1, GyroX1, GyroY1, GyroZ1; // Raw Data Collection from MPU1
float AccX2, AccY2, AccZ2, GyroX2, GyroY2, GyroZ2; // Raw Data Collection from MPU2
float yaw1, yaw2;                                  // Yaw Angle from MPU1&2     
float GyroErrorZ1, GyroErrorZ2;                    // Calculate Error for MPU1&2
float GyroZ1Filt = 0;
float GyroZ2Filt = 0;
float GyroZ1Pre = 0;
float GyroZ2Pre = 0;
float AnkleAngle = 0;
int c = 0;
int d = 0;
long currT = 0;           // Current Time
float deltaT = 0;         // Time Difference
long preT = 0;            // Previous Time


void setup() {
  Serial.begin(9600);
  //Set MPU1
  Wire.begin();                      
  Wire.beginTransmission(MPU1);       
  Wire.write(0x6B);                 
  Wire.write(0x00);                  
  Wire.endTransmission(true);
  //Set MPU2        
  Wire.begin();                      
  Wire.beginTransmission(MPU2);       
  Wire.write(0x6B);                  
  Wire.write(0x00);                 
  Wire.endTransmission(true);
  //Calculate error for each MPU        
  calculate_IMU_error1(MPU1);
  calculate_IMU_error2(MPU2);

  delay(20);
}

void loop() {
currT = micros();                                    // Start Current Time Microsecond
deltaT = ((float)(currT-preT)) / (1e6);              // Calculate Delta Time
preT = currT; 
 
ReadMpu1(MPU1);
ReadMpu2(MPU2);

//Low-pass filter(25Hz)
GyroZ1Filt = 0.878 * GyroZ1Filt+0.0775 * GyroZ1+0.0728 * GyroZ1Pre; // Apply low pass filter (25Hz)
GyroZ2Filt = 0.878 * GyroZ2Filt+0.0775 * GyroZ2+0.0728 * GyroZ2Pre;
GyroZ1Pre = GyroZ1;                                                 // Store previous angular velocity
GyroZ2Pre = GyroZ2;

// Calculate Yaw Angle & Ankle Angle
yaw1 =  yaw1 + GyroZ1Filt * deltaT;                                 // Caculate rotation angle by integral
yaw2 =  yaw2 + GyroZ2Filt * deltaT;
AnkleAngle = yaw2-yaw1;


Serial.print(yaw1);
Serial.print(",   ");
Serial.print(yaw2);
Serial.print(",     ");
Serial.print(AnkleAngle);
Serial.print(",      ");
Serial.print(GyroZ2);
Serial.print(",   ");
Serial.print(GyroZ1);
Serial.println(",      ");
}

void ReadMpu1(const int MPU){
  //Read acceleromter data
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)见datasheet
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  AccX1 = (Wire.read() << 8 | Wire.read()) / 16384.0; 
  AccY1 = (Wire.read() << 8 | Wire.read()) / 16384.0; 
  AccZ1 = (Wire.read() << 8 | Wire.read()) / 16384.0; 
  //Read gyroscope data
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX1 = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY1 = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ1 = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values

  GyroZ1 = GyroZ1 - GyroErrorZ1;
}

void ReadMpu2(const int MPU){
    //Read acceleromter data
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)见datasheet
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  AccX2 = (Wire.read() << 8 | Wire.read()) / 16384.0; 
  AccY2 = (Wire.read() << 8 | Wire.read()) / 16384.0; 
  AccZ2 = (Wire.read() << 8 | Wire.read()) / 16384.0; 
  //Read gyroscope data
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX2 = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY2 = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ2 = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroZ2 = GyroZ2 - GyroErrorZ2;
}

void calculate_IMU_error1(const int MPU) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX1 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY1 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ1 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
 
  while (c < 2000) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX1 = Wire.read() << 8 | Wire.read();
    GyroY1 = Wire.read() << 8 | Wire.read();
    GyroZ1 = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorZ1 = GyroErrorZ1 + (GyroZ1 / 131.0);
    c++;
  }
  GyroErrorZ1 = GyroErrorZ1 / 2000;
}

void calculate_IMU_error2(const int MPU) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;

  while (d < 2000) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX2 = Wire.read() << 8 | Wire.read();
    GyroY2 = Wire.read() << 8 | Wire.read();
    GyroZ2 = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorZ2 = GyroErrorZ2 + (GyroZ2 / 131.0);
    d++;
  }
  GyroErrorZ2 = GyroErrorZ2 / 2000;
}
