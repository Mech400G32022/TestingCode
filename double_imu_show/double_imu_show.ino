#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SD.h>
#include <String.h>


#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno1 = Adafruit_BNO055(1, BNO055_ADDRESS_A, &Wire1);
Adafruit_BNO055 bno2 = Adafruit_BNO055(2, BNO055_ADDRESS_A, &Wire);

//Max of 999,999 log files.
char fileName[16];


void setup() {
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno1.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected on SDA 1 ... Check your wiring or I2C ADDR!");
    while(1);
  }

  if(!bno2.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected on SDA 0 ... Check your wiring or I2C ADDR!");
    while(1);
  }

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }

  int i = 0;
  sprintf(fileName, "dataLog%d.txt", i);
  while (SD.exists(fileName)){
    i++;   
    sprintf(fileName, "dataLog%d.txt", i);
  }
  delay(1000);
  
  bno1.setExtCrystalUse(true);
  bno2.setExtCrystalUse(true);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  imu::Vector<3> euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> euler2 = bno2.getVector(Adafruit_BNO055::VECTOR_EULER);

  String dataString1 = "X1: ";
  dataString1 += String(euler1.x());
  dataString1 += " Y1: ";
  dataString1 += String(euler1.y());
  dataString1 += " Z1: ";
  dataString1 += String(euler1.z());
  dataString1 += "\t\t";

  String dataString2 = "X2: ";
  dataString2 += String(euler2.x());
  dataString2 += " Y2: ";
  dataString2 += String(euler2.y());
  dataString2 += " Z2: ";
  dataString2 += String(euler2.z());
  dataString2 += "\t\t";


  /* Display the floating point data */
  Serial.println(dataString1);
  Serial.println(dataString2);

  File dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString1);
    dataFile.println(dataString2);
    dataFile.close();
  } 

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
