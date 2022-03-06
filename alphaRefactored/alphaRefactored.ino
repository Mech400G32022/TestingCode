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

//Hardware
Adafruit_BNO055 platformImu = Adafruit_BNO055(1, BNO055_ADDRESS_A, &Wire1);
Adafruit_BNO055 baseImu = Adafruit_BNO055(2, BNO055_ADDRESS_A, &Wire);


PololuDcMotor motor1 = PololuDcMotor(INA1, INB1, M1_PWM);
Encoder enc1(ENCA_1, ENCB_1);
PololuDcMotor motor2 = PololuDcMotor(INA2, INB2, M2_PWM);

#define SPOOL_PLATE_RATIO 14.14

#define F_MC_LOOP_HZ 1000
IntervalTimer motorControlTimer;

#define F_IMU_LOOP_HZ 100
IntervalTimer positionControlTimer;

#define IMU_CALIB_SIZE (sizeof(adafruit_bno055_offsets_t) + sizeof(long))

#define DEBUG_SERIAL
#define FATAL(error) Serial.println(error);

//If we don't want serial all calls will be removed by preprocessor
#ifdef DEBUG_SERIAL
  #define SerialPrintLn(m) Serial.println(m)
#else
  #define SerialPrintLn(m)
#endif

#define DEBUG_SD
#define FILENAME_PATTERN "motor_imu%d.csv"
#define CSV_HEADER "time (ms),enc pos,y1,y2"

//Define the index of the data to write to the file
//Ensure that the CSV_HEADER and CSV_LEN of the buffer match.
#define CSV_TIME  0
#define CSV_ENC_POS    1
#define CSV_Y1    2
#define CSV_Y2    3

#define CSV_LEN   4
volatile double csv_buffer[CSV_LEN];

//If the buffer has no length we shouldn't be writing to file.
//Save computation by removing need for bounds checking
#if CSV_LEN <= 0
  #undef DEBUG_SD
#endif


//We are willing to accept losing the last 10 readings from the IMU on power down.
#define FLUSH_PERIOD_MS 10 * 1000 / F_IMU_LOOP_HZ
#define F_SAVE_CSV_HZ 50
IntervalTimer saveCSVTimer;

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
  dataFile.println(csv_buffer[CSV_LEN - 1]);
  //dataFile.flush();
}

void initImu(Adafruit_BNO055 imu) {
   FATAL("Before begin");

   if(!imu.begin()) {
    FATAL("No BNO055 detected Check your wiring or I2C ADDR!");
    delay(200);
    //while(1);
  }

  FATAL("After begin");
  sensor_t sensor;
  imu.getSensor(&sensor);


  int eeAddress = sensor.sensor_id * IMU_CALIB_SIZE;
  long bnoID;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;

  /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  if (bnoID != sensor.sensor_id) {
      SerialPrintLn("\nNo Calibration Data for this sensor exists in EEPROM");
  } else {
      SerialPrintLn("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);
      SerialPrintLn("\n\nRestoring Calibration data to the BNO055...");
      imu.setSensorOffsets(calibrationData);
  }
    
    delay(200);
   /* Crystal must be configured AFTER loading calibration data into BNO055. */
    imu.setExtCrystalUse(true);
}

void setup() {
  #ifdef DEBUG_SERIAL
    Serial.begin(9600);
  #endif




  //The motor specific
  motor1.deadzone = 1200;
  motor2.deadzone = 1200;

  SerialPrintLn("Hello");
  initImu(baseImu);
  SerialPrintLn("Hello2");
  initImu(platformImu);
  SerialPrintLn("Hello3");

  #ifdef DEBUG_SD
    initDataLog();
    if (!saveCSVTimer.begin(saveCSV, 1000000/F_SAVE_CSV_HZ)) {
      FATAL("Hardware timers unavailable");
    }
    motorControlTimer.priority(150);

  #endif

/*
  if (!motorControlTimer.begin(motorLoop, 1000)) {
    FATAL("Hardware timers unavailable");
  }
  motorControlTimer.priority(149);*/

  if (!positionControlTimer.begin(positionLoop, 10000)){
    FATAL("Hardware Timers unavailable");
  }
  positionControlTimer.priority(148);

}

void motorLoop() {
  
}

void positionLoop() {
  imu::Vector<3> euler1 = platformImu.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro1 = platformImu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler2 = baseImu.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro2 = baseImu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  
  noInterrupts();
  csv_buffer[CSV_TIME] = millis();
  csv_buffer[CSV_Y1] = euler1.y();
  csv_buffer[CSV_Y2] = euler2.y();
  interrupts();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() > 4000) {
    motor1.setPower(0);
    dataFile.flush();
  } else if (millis() > 2000) {
    motor1.setPower(2000);
  }
  csv_buffer[CSV_ENC_POS] = (double)enc1.read();
}
