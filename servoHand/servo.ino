#include <Servo.h>
#include <Wire.h>
#include "I2Cdev.h" //I2C kütüphanesi
#include "MPU6050.h" //Mpu6050 kütüphanesi
#include <SoftwareSerial.h>

MPU6050 accelgyro;
Servo mServo;
int val;
int prevVal;

int16_t ax, ay, az; //ivme tanımlama
int16_t gx, gy, gz; //gyro tanımlama

void setup(void) 
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("MPU6050 Test"); Serial.println("");

  accelgyro.initialize();
    /* There was a problem detecting the MPU6050 ... check your connections */
  Serial.println("Test cihazı bağlantıları...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 bağlantı başarılı" : "MPU6050 bağlantısı başarısız");  

  mServo.attach(9);
}

void loop(void) 
{
  int i;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // ivme ve gyro değerlerini okuma açısal ivmeleri ve gyro değerlerini ekrana yazdırma
 // Serial.print("a/g 1 :\t");
 // Serial.print(ax); Serial.print("\t");
 // Serial.print(ay); Serial.print("\t");
 // Serial.print(az); Serial.print("\t");
 // Serial.print(gx); Serial.print("\t");
 // Serial.print(gy); Serial.print("\t");
 // Serial.println(gz);
 // delay(100);
// for (i=0;i<20;i++){
    val = map(ay, -17000, 17000, 0, 255);
// }
// val=val/20;
  
  //if(val!=prevVal)
  if(((val-prevVal)>10)||((val-prevVal)<-10))
  {
    mServo.write(val);
    prevVal=val;
  }
  
  delay(100);
}


