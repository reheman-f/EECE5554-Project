#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define SDA_PIN 41
#define SCL_PIN 40
// #define SDA_PIN 7
// #define SCL_PIN 6
#define LSM_I2C_ADDR1 0x6A
#define LIS3_I2C_ADDR1 0x1C
#define LSM_I2C_ADDR2 0x6B
#define LIS3_I2C_ADDR2 0x1E
#define BNO085_ADDR 0x4A
#define BNO_I2C_ADDR  BNO055_ADDRESS_A   // 0x28 if ADR pin is GND, 0x29 if ADR is VCC


Adafruit_LSM6DSOX sox1;
Adafruit_LIS3MDL lis3mdl1;
Adafruit_LSM6DSOX sox2;
Adafruit_LIS3MDL lis3mdl2;
Adafruit_BNO055 bno = Adafruit_BNO055(1, BNO_I2C_ADDR, &Wire);

// --- timing variables ---
const uint32_t SAMPLE_PERIOD_US = 50000;  // 20 Hz
uint32_t lastSampleTimeUs = 0;

void setup() {
  Serial.begin(115200);
  
  while (!Serial)
  {
    delay(10);
  }

  Serial.println("Sensor Check");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  while (!sox1.begin_I2C(LSM_I2C_ADDR1, &Wire, 1)) {
    Serial.println("Failed to find LSM6DSOX 1 chip");
    delay(1000);
    sox1.reset();
  }

  while (!lis3mdl1.begin_I2C(LIS3_I2C_ADDR1, &Wire)) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Failed to find LIS3MDL 1 chip");
    delay(1000);
    lis3mdl1.reset();
  }

  while (!sox2.begin_I2C(LSM_I2C_ADDR2, &Wire, 2)) {
    Serial.println("Failed to find LSM6DSOX 2 chip");
    delay(1000);
    sox2.reset();
  }

  while (!lis3mdl2.begin_I2C(LIS3_I2C_ADDR2, &Wire)) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Failed to find LIS3MDL 2 chip");
    delay(1000);
    lis3mdl2.reset();
  }

  while (!bno.begin()) {
    Serial.println("Failed to find BNO055 chip");
    delay(1000);
  }

  Serial.println("All sensors Found!");

  lis3mdl1.setPerformanceMode(LIS3MDL_HIGHMODE);
  lis3mdl1.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl1.setDataRate(LIS3MDL_DATARATE_560_HZ);
  lis3mdl1.setRange(LIS3MDL_RANGE_4_GAUSS);

  sox1.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  sox1.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  sox1.setAccelDataRate(LSM6DS_RATE_833_HZ);
  sox1.setGyroDataRate(LSM6DS_RATE_833_HZ);

  lis3mdl2.setPerformanceMode(LIS3MDL_HIGHMODE);
  lis3mdl2.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl2.setDataRate(LIS3MDL_DATARATE_560_HZ);
  lis3mdl2.setRange(LIS3MDL_RANGE_4_GAUSS);

  sox2.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  sox2.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  sox2.setAccelDataRate(LSM6DS_RATE_833_HZ);
  sox2.setGyroDataRate(LSM6DS_RATE_833_HZ);
  
  lastSampleTimeUs = micros();
}

sensors_event_t accel[2] = {0};
sensors_event_t gyro[2] = {0};
sensors_event_t mag[2] = {0};
sensors_event_t temp[2] = {0};

void loop() {
  uint32_t nowUs = micros();
  if (nowUs - lastSampleTimeUs >= SAMPLE_PERIOD_US) 
  {
    lastSampleTimeUs = nowUs;

    // LSM

    if (sox1.getEvent(&accel[0], &gyro[0], &temp[0]) && lis3mdl1.getEvent(&mag[0]))
    {
      Serial.print("$VNYMR,1,");
      Serial.print(micros()); Serial.print(",");
      Serial.print(0, 3); Serial.print(",");  
      Serial.print(0, 3); Serial.print(",");
      Serial.print(0, 3); Serial.print(",");
      Serial.print((mag[0].magnetic.x/100), 3); Serial.print(",");  // last-known mag    // Needs to be in Gauss
      Serial.print((mag[0].magnetic.y/100), 3); Serial.print(",");
      Serial.print((mag[0].magnetic.z/100), 3); Serial.print(",");
      Serial.print(accel[0].acceleration.x, 3); Serial.print(",");     // Needs to be in m/s2
      Serial.print(accel[0].acceleration.y, 3); Serial.print(",");
      Serial.print(accel[0].acceleration.z, 3); Serial.print(",");
      Serial.print(gyro[0].gyro.x, 3); Serial.print(",");              // Needs to be in rad/s
      Serial.print(gyro[0].gyro.y, 3); Serial.print(",");
      Serial.print(gyro[0].gyro.z, 3); Serial.print("\n");
    }

    if (sox2.getEvent(&accel[1], &gyro[1], &temp[1]) && lis3mdl2.getEvent(&mag[1]))
    {
      Serial.print("$VNYMR,2,");
      Serial.print(micros()); Serial.print(",");
      Serial.print(0, 3); Serial.print(",");  // last-known mag
      Serial.print(0, 3); Serial.print(",");
      Serial.print(0, 3); Serial.print(",");
      Serial.print((mag[1].magnetic.x/100), 3); Serial.print(",");  // last-known mag    // Needs to be in Gauss
      Serial.print((mag[1].magnetic.y/100), 3); Serial.print(",");
      Serial.print((mag[1].magnetic.z/100), 3); Serial.print(",");
      Serial.print(accel[1].acceleration.x, 3); Serial.print(",");     // Needs to be in m/s2
      Serial.print(accel[1].acceleration.y, 3); Serial.print(",");
      Serial.print(accel[1].acceleration.z, 3); Serial.print(",");
      Serial.print(gyro[1].gyro.x, 3); Serial.print(",");              // Needs to be in rad/s
      Serial.print(gyro[1].gyro.y, 3); Serial.print(",");
      Serial.print(gyro[1].gyro.z, 3); Serial.print("\n");
    }
    // Clearing Buffers
    for (int i = 0; i < 2; i++)
    {
      accel[i] = {0};
      gyro[i] = {0};
      mag[i] = {0};
      temp[i] = {0};
    }

    // BNOO055
    sensors_event_t accelEvent;    // m/s^2
    sensors_event_t gyroEvent;     // rad/s
    sensors_event_t magEvent;      // µT


    bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);  // m/s^2
    bno.getEvent(&gyroEvent,  Adafruit_BNO055::VECTOR_GYROSCOPE);      // rad/s
    bno.getEvent(&magEvent,   Adafruit_BNO055::VECTOR_MAGNETOMETER);   // uT

    // Map orientation to yaw / pitch / roll (VNYMR expects: Yaw, Pitch, Roll)
    // float yaw   = orientEvent.orientation.x;  // heading
    // float roll  = orientEvent.orientation.y;
    // float pitch = orientEvent.orientation.z;

    // Output VNYMR-style 
    Serial.print("$VNYMR,3,");
    Serial.print(micros()); Serial.print(",");
    Serial.print(0,   3);  Serial.print(",");   // Yaw (deg)
    Serial.print(0, 3);  Serial.print(",");   // Pitch (deg)
    Serial.print(0,  3);  Serial.print(",");   // Roll (deg)

    Serial.print((magEvent.magnetic.x / 100.0f), 3); Serial.print(",");   // Mag X (gauss)
    Serial.print((magEvent.magnetic.y / 100.0f), 3); Serial.print(",");
    Serial.print((magEvent.magnetic.z / 100.0f), 3); Serial.print(",");
    Serial.print(accelEvent.acceleration.x, 3);     Serial.print(",");   // Accel X (m/s^2)
    Serial.print(accelEvent.acceleration.y, 3);     Serial.print(",");
    Serial.print(accelEvent.acceleration.z, 3);     Serial.print(",");
    Serial.print(gyroEvent.gyro.x, 3);     Serial.print(",");   // Gyro X (rad/s)
    Serial.print(gyroEvent.gyro.y, 3);     Serial.print(",");
    Serial.print(gyroEvent.gyro.z, 3);     Serial.print("\n");
    

  }


}
