
#include <Encoder.h>
#include <SD.h>
#include <PololuMotorDriver.h>
#include <String.h>

#define INA2 5
#define INB2 6
#define M2_PWM 7

char fileName[16];
File dataFile;

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(38, 39);
PololuDcMotor motor =  PololuDcMotor(INA2, INB2, M2_PWM);
//   avoid using pins with LEDs attached

void setup() {

  Serial.begin(115200);

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }
    
     int i = 0;
    sprintf(fileName, "motorEnc%d.csv", i);
    while (SD.exists(fileName)){
      i++;   
      sprintf(fileName, "motorEnc%d.csv", i);
    }
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
      
      dataFile.println("time,tic, power");
    } else {
      Serial.println("No file could be opened");
    }
}

long lastPos = -99;
void loop() {
  int power = 0;
  if (millis() > 3000) {
    power = 5000;
  }
  if (millis() > 5000) {
    power = 0;
  }
  if (millis() > 6000) {
    if (dataFile)
      dataFile.close();
  }
  
  long newPosition = myEnc.read();
  if (lastPos != newPosition && dataFile) {
    String dataString = String(micros());
    dataString += ",";
    dataString += String(newPosition);
    dataString += ",";
    dataString += String(power);
    
    dataFile.println(dataString);
  }
  lastPos = newPosition;
  motor.setPower(power);
}
