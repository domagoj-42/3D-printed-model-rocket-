#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP085.h"

MPU6050 accelgyro;
HMC5883L mag;
BMP085 presstemp;

//accelerometer
int16_t ax;
int16_t ay;
int16_t az;
//gyroscope
int16_t gx;
int16_t gy;
int16_t gz;
//magnetometer
int16_t mx;
int16_t my;
int16_t mz;
//barometer/thermometer
float temperature;
float pressure;
float altitude;
//time
unsigned long t; //measurement time
unsigned long reference_time; 

void setup() {
   Serial.begin(57600);   //intitialize serial communication
   Wire.begin();    //initialize i2c communication
   accelgyro.setI2CMasterModeEnabled(false);
   accelgyro.setI2CBypassEnabled(true) ;
   accelgyro.setSleepEnabled(false);

   //initialize sensors
   accelgyro.initialize();
   mag.initialize();
   presstemp.initialize();

   accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
   accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
   mag.setMode(HMC5883L_MODE_CONTINUOUS);

   //test connections
   Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
   Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
   Serial.println(presstemp.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");
   Serial.println("t\tax\tay\taz\tgx\tgy\tgz\tmx\tmy\tmz\ttemp\tpress\talt");

}

void loop() {
   reference_time = millis();
   if (reference_time - t >= 14){
     //read time
     t = millis();
     
     //read accelrometer and gyroscope
     accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
     
     //read magentometer
     mag.getHeading(&mx, &my, &mz);
     
     //set BMP180 to temperature mode and read temperature
     presstemp.setControl(BMP085_MODE_TEMPERATURE);
     temperature = presstemp.getTemperatureC();
  
     //set BMP180 to pressure mode, read pressure and calculate altitude
     presstemp.setControl(BMP085_MODE_PRESSURE_2);
     pressure = presstemp.getPressure();
     altitude = presstemp.getAltitude(pressure);
  
     //get time just before it is outputted, marks begging of outputting data
     Serial.print(t);
     Serial.print("\t");
  
     // output accelerometer
     Serial.print(ax);
     Serial.print("\t");
     Serial.print(ay);
     Serial.print("\t");
     Serial.print(az);
     Serial.print("\t");
  
     //output gyroscope
     Serial.print(gx);
     Serial.print("\t");
     Serial.print(gy);
     Serial.print("\t");
     Serial.print(gz);
     Serial.print("\t");
  
     //output magentometer
     Serial.print(mx);
     Serial.print("\t");
     Serial.print(my);
     Serial.print("\t");
     Serial.print(mz);
     Serial.print("\t");
  
     //output pressure/temperature
     Serial.print(temperature);
     Serial.print("\t");
     Serial.print(pressure);
     Serial.print("\t");
     Serial.print(altitude);
     Serial.print("\n\r");
   }
}
