#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SD.h>
#include <PololuMotorDriver.h>
#include <EEPROM.h>
#include <String.h>

//Pin Definitions
#define INA1 2
#define INB1 3
#define M1_PWM 4
#define INA2 5
#define INB2 6
#define M2_PWM 7

//Hardware
Adafruit_BNO055 plateImu = Adafruit_BNO055(1, BNO055_ADDRESS_A, &Wire1);
Adafruit_BNO055 baseImu = Adafruit_BNO055(2, BNO055_ADDRESS_A, &Wire);


volatile PololuDcMotor motor1 = PololuDcMotor(INA1, INB1, M1_PWM);
volatile PololuDcMotor motor2 = PololuDcMotor(INA2, INB2, M2_PWM);

#define F_MC_LOOP_HZ 1000
IntervalTimer motorControlTimer;

#define F_IMU_LOOP_HZ 100
IntervalTimer positionControlTimer;

#define DEBUG_SERIAL

//If we don't want serial all calls will be removed by preprocessor
#ifdef DEBUG_SERIAL
  #define SerialPrintLn(m) Serial.println(m)
#else
  #define SerialPrintLn(m)
#endif

#define DEBUG_SD
#define FILENAME_PATTERN "syslog%d.csv"
#define CSV_HEADER "time (ms),y1,z1,y2,z2"

//Define the index of the data to write to the file
//Ensure that the CSV_HEADER and CSV_LEN of the buffer match.
#define CSV_TIME  0
#define CSV_Y1    1
#define CSV_Z1    2
#define CSV_Y2    3
#define CSV_Z2    4

#define CSV_LEN   5
volatile double csv_buffer[CSV_LEN];

//If the buffer has no length we shouldn't be writing to file.
//Save computation by removing need for bounds checking
#if CSV_LEN <= 0
  #undef DEBUG_SD
#endif


//We are willing to accept losing the last 10 readings from the IMU on power down.
#define FLUSH_PERIOD_MS 10 * 1000 / F_IMU_LOOP_HZ
#define F_SAVE_CSV_HZ 100
File dataFile = NULL;
volatile bool writeToFile = false;

void initDataLog(){
  if (SD.begin(BUILTIN_SDCARD)) {
    int i = 0;
    char fileName[16];
    
    sprintf(fileName, FILENAME_PATTERN, i);
    while (SD.exists(fileName)){
      i++;   
      sprintf(fileName, FILENAME_PATTERN, i);
    }
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println(CSV_HEADER);
      writeToFile = true;
    } else {
      SerialPrintLn("No file could be opened");
    }
  } else {
     SerialPrintLn("Card failed, or not present");
  }
}

void saveCSV(){
  for(int i = 0; i < CSV_LEN - 1; i++) {
    dataFile.print(csv_buffer[i]);
    dataFile.print(",");
  }
  dataFile.print(csv_buffer[CSV_LEN - 1]);
}

void setup() {
  #ifdef DEBUG_SERIAL
    Serial.begin(9600);
  #endif

  #ifdef DEBUG_SD
    initDataLog();
  #endif

}

void loop() {
  // put your main code here, to run repeatedly:

}
