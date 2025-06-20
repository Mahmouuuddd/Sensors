#include <Adafruit_HMC5883_U.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPS++.h>

Adafruit_HMC5883_Unified mag;

Adafruit_MPU6050 mpu;

TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);
  }

  if (!mag.begin()) {
    Serial.println("Failed to initialize LSM303!");
    while (1);
  }
}

void loop() {
  // Read GPS data
  while (Serial.available() > 0) {
    if (gps.encode(Serial.read())) {
      if (gps.location.isValid()) {
        float gpsLatitude = gps.location.lat();
        float gpsLongitude = gps.location.lng();
        float gpsAltitude = gps.altitude.meters();
        
        // Send GPS data over serial port
        Serial.print("GPS:");
        Serial.print(gpsLatitude, 6); // Print with 6 decimal places
        Serial.print(",");
        Serial.print(gpsLongitude, 6);
        Serial.print(",");
        Serial.print(gpsAltitude, 2); // Print with 2 decimal places
        Serial.println();
      }
    }
  }
  
  // Read IMU data
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  
  float imuAccelerationX = accel.acceleration.x;
  float imuAccelerationY = accel.acceleration.y;
  float imuAccelerationZ = accel.acceleration.z;
  
  float imuGyroX = gyro.gyro.x;
  float imuGyroY = gyro.gyro.y;
  float imuGyroZ = gyro.gyro.z;
  
  // Send IMU data over serial port
  Serial.print("IMU:");
  Serial.print(imuAccelerationX);
  Serial.print(",");
  Serial.print(imuAccelerationY);
  Serial.print(",");
  Serial.print(imuAccelerationZ);
  Serial.print(",");
  Serial.print(imuGyroX);
  Serial.print(",");
  Serial.print(imuGyroY);
  Serial.print(",");
  Serial.print(imuGyroZ);
  Serial.println();

  // Read Compass data
  sensors_event_t mag_data;
  mag.getEvent(&mag_data);

  float compassX = mag_data.magnetic.x;
  float compassY = mag_data.magnetic.y;
  float compassZ = mag_data.magnetic.z;

  // Send Compass data over serial port
  Serial.print("Compass:");
  Serial.print(compassX);
  Serial.print(",");
  Serial.print(compassY);
  Serial.print(",");
  Serial.print(compassZ);
  Serial.println();
  
  delay(100); // Adjust the delay according to your requirements
}
