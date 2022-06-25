#include <Wire.h>
#include "I2Cdev.h" //I2C kütüphanesi
#include "MPU6050.h" //Mpu6050 kütüphanesi
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11); // RX, TX

#define TCAADDR 0x70

/* Assign a unique ID to sensors at the same time */
//MPU6050 accelgyro1 = MPU6050(1);
//MPU6050 accelgyro2 = MPU6050(2);
MPU6050 accelgyro;

int16_t ax, ay, az; //ivme tanımlama
int16_t gx, gy, gz; //gyro tanımlama

int16_t ax2, ay2, az2; //ivme tanımlama
int16_t gx2, gy2, gz2; //gyro tanımlama

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


void setup(void) 
{
  Wire.begin();
  BTSerial.begin(115200);
  BTSerial.println("MPU6050 Test"); Serial.println("");
  
  /* Initialise the 1st sensor */
  tcaselect(2);
  accelgyro.initialize();
    /* There was a problem detecting the MPU6050 ... check your connections */
    BTSerial.println("Test cihazı bağlantıları...");
    BTSerial.println(accelgyro.testConnection() ? "MPU6050 bağlantı başarılı" : "MPU6050 bağlantısı başarısız");  

  /* Initialise the 1st sensor */
  tcaselect(7);
  accelgyro.initialize();
    /* There was a problem detecting the MPU6050 ... check your connections */
    BTSerial.println("Test cihazı bağlantıları...");
    BTSerial.println(accelgyro.testConnection() ? "MPU6050 bağlantı başarılı" : "MPU6050 bağlantısı başarısız"); 
}

void loop(void) 
{
  tcaselect(2);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // ivme ve gyro değerlerini okuma açısal ivmeleri ve gyro değerlerini ekrana yazdırma
  BTSerial.print("a/g 1 :\t");
  BTSerial.print(ax); Serial.print("\t");
  BTSerial.print(ay); Serial.print("\t");
  BTSerial.print(az); Serial.print("\t");
  BTSerial.print(gx); Serial.print("\t");
  BTSerial.print(gy); Serial.print("\t");
  BTSerial.println(gz);
  delay(100);
  
  tcaselect(7);
  accelgyro.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2); // ivme ve gyro değerlerini okuma açısal ivmeleri ve gyro değerlerini ekrana yazdırma
  BTSerial.print("A/G 2:\t");
  BTSerial.print(ax2); Serial.print("\t");
  BTSerial.print(ay2); Serial.print("\t");
  BTSerial.print(az2); Serial.print("\t");
  BTSerial.print(gx2); Serial.print("\t");
  BTSerial.print(gy2); Serial.print("\t");
  BTSerial.println(gz2);
  
  delay(500);
}

