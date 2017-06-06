// GYROOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO //
#include "I2Cdev.h"
#include "MPU6050.h"
//#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define g 9.8
#define COUNTS_TO_1g 16384
#define COUNTS_TO_0ROT 131


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro (0x68);
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
float dx, dy, dz = 0;
int16_t gx, gy, gz;

double pms = 0;
float heading_current = 0;
bool blinkState = false;



long AX_offset, AY_offset ,AZ_offset, GX_offset, GY_offset, GZ_offset;
unsigned long gyro_time;
void gyroSetup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(19200);

    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    Serial.println("Updating internal sensor offsets...");

    gyro_time = millis();
    calibration();
}

float GyroReading() {
  int16_t gz_val = accelgyro.getRotationZ();
  unsigned long gyro_end_time = millis();
  float heading_change = (gz_val - GZ_offset)/(float)COUNTS_TO_0ROT * (gyro_end_time - gyro_time) / 1000.0;
  if (abs(heading_change) < 0.01) { // if it is a small signal, ignore since it is probably noise
    heading_change = 0;
  }
  gyro_time = gyro_end_time;
  return heading_change;
}

void track_heading() {
  int16_t gz_val = accelgyro.getRotationZ();
  Serial.println(gz_val);
//  Serial.print("RZ: ");Serial.print(gz_val/(float)COUNTS_TO_0ROT); Serial.print("\t\n");
  float heading_change = (gz_val - GZ_offset)/(float)COUNTS_TO_0ROT * (millis() - pms) / 1000.0;
  if (abs(heading_change) > 0.01) {
    heading_current += heading_change;
  }
  pms = millis();
  Serial.print("Heading: "); Serial.println(heading_current);
}

void calibration() {
  Serial.println("Calibration Started");
  // set everything to zero
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
    
  int Points = 100;
  long AX = 0;
  long AY = 0;
  long AZ = 0;
  long GX = 0;
  long GY = 0;
  long GZ = 0;
  // record n values
  for (int i = 0; i < Points; i++){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // Running Average since I know the number of points before hand
    AX += ax/Points;
    AY += ay/Points;
    AZ += az/Points;
    GX += gx/Points;
    GY += gy/Points;
    GZ += gz/Points;
    delay(1);
  }

  // do some adjustment so that everything goes to 0
  AX_offset = -AX/8;
  AY_offset = -AY/8;
  AZ_offset = (16384 - AZ)/8; //assuming gravity is in the z direction. 16384 is what 1g should be
  GX_offset = -GX;
  GY_offset = -GY/4;
  GZ_offset = -GZ/4;
  
  // Display offsets -- only using GZ
  PrintT("AX", (float)AX_offset);
  PrintT("AY", (float)AY_offset);
  PrintN("AZ", (float)AZ_offset);
  PrintT("GX", (float)GX_offset);
  PrintT("GY", (float)GY_offset);
  PrintN("GZ", (float)GZ_offset);
}


