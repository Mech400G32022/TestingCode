#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SD.h>
#include <PololuMotorDriver.h>
#include <EEPROM.h>
#include <Encoder.h>
#include <String.h>


//Pin Definitions
#define INA1 2
#define INB1 3
#define M1_PWM 4
#define ENCA_1 38
#define ENCB_1 39
#define INA2 5
#define INB2 6
#define M2_PWM 7



PololuDcMotor motor1 = PololuDcMotor(INA1, INB1, M1_PWM);
Encoder enc1(ENCA_1, ENCB_1);
PololuDcMotor motor2 = PololuDcMotor(INA2, INB2, M2_PWM);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

int m_speed = 0;
long last_enc_val = 0;
void loop() {
  // put your main code here, to run repeatedly:
  if (enc1.read() != last_enc_val) {
    motor1.setPower(0);
    Serial.println(m_speed);
  } else {
    last_enc_val = enc1.read();
    m_speed += 5;
    motor1.setPower(m_speed);
    delay(10);
  }
}
