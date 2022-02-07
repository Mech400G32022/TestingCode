#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SD.h>
#include <String.h>


#define ENABLE 0
#define INA 1
#define INB 2
#define M_PWM 3


Adafruit_BNO055 bno1 = Adafruit_BNO055(1, BNO055_ADDRESS_A, &Wire1);

void setup() {
  pinMode(ENABLE, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(M_PWM, OUTPUT);

  Serial.begin(9600);

  if(!bno1.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected on SDA 1 ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  bno1.setExtCrystalUse(true);

  digitalWrite(ENABLE, HIGH);
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
  analogWriteFrequency(M_PWM, 18311);
  analogWriteResolution(13);
}

float lastVal = 0;

void loop() {
  // put your main code here, to run repeatedly:
  imu::Vector<3> euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro1 = bno1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  delay(10);  
  Serial.println(gyro1.y());
  //Serial.println(20*euler1.y());

  uint8_t status_reg = 0;
  uint8_t self_test = 0;
  uint8_t sys_err = 0;
  bno1.getSystemStatus(&status_reg, &self_test, &sys_err);
  //Serial.print(status_reg);
  //Serial.print(", ");
  //Serial.print(self_test);
  //Serial.print(", ");
  //Serial.println(sys_err);

  if (status_reg != 5){
    digitalWrite(13, HIGH);
    Serial.println("Reconnecting");
    if(!bno1.begin())
      {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected on SDA 1 ... Check your wiring or I2C ADDR!");
        
            digitalWrite(ENABLE, LOW);
            digitalWrite(INA, LOW);
            digitalWrite(INB, LOW);
      }
  } else {
    digitalWrite(13, LOW);
    setPower(int(800*euler1.y()));
    
    lastVal = euler1.y();
  }
}

void setPower(int power){
  if (power < 0){
      power = -power;
      digitalWrite(INA, LOW);
      digitalWrite(INB, HIGH);
        digitalWrite(ENABLE, HIGH);
  } else {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
    digitalWrite(ENABLE, HIGH);

  }
  if (power > 7000){
    power = 7000;
  }
  
  analogWrite(M_PWM, power);
}
