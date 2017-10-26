/*=============================================================================================
    LIBRARY INCLUDES
---------------------------------------------------------------------------------------------*/
#include <Wire.h>
#include <LSM303.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <XBee.h>
#include "SatellitePins.h"
/*===========================================================================================
    FUNCTION PROTOTYPES
--------------------------------------------------------------------------------------------*/
LSM303 compass;
SoftwareSerial xBeeSerial(XBEE_RX, XBEE_TX); // RX, TX
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
sensor_t sensor;

void InitialiseSensors(void);
void StepMotor(int MotorPin, int MotorDirectionPin, int Direction, int Speed);
void TrackSat(int AzimuthAngle,int TimeForMove,int ElevationAngle,int Direction);
void convertCSVtoInt(int *arr, int arrSize, String csvStr);
String getXbeeData();
float GetElevation(float XAccel,float YAccel,float ZAccel);
bool CalibrateAzimuth(float Azimuth, bool Calibrated);
void MoveToAzimuth(int Angle, int Elevation);
/*=============================================================================================
    VARIABLE DECLARATION
----------------------------------------------------------------------------------------------*/
int expansion_output = 0b11001000;
float Azimuth, Elevation;
bool ElevationCalibrated = false, AzimuthCalibrated = false;
bool WaitingForOrders = true; 
String GOCommand;
/*=============================================================================================
    SETUP LOOP
----------------------------------------------------------------------------------------------*/
void setup() {
  InitialiseSensors();
  Serial.println("Sensors Initialised");
}
/*==============================================================================================
       *MAIN PROGRAM LOOP MAIN PROGRAM LOOP MAIN PROGRAM LOOP*    
-----------------------------------------------------------------------------------------------*/

/*==============================================================================================
CALIBRATION ROUTINE
-----------------------------------------------------------------------------------------------*/
void loop() {
  sensors_event_t event;
//BEGIN CALIBRATION ROUTINE OF ELEVATION
  while (ElevationCalibrated == false){
    accel.getEvent(&event);
    Elevation = GetElevation(event.acceleration.x,event.acceleration.y,event.acceleration.z);
    while (Elevation > 5){
      StepMotor(MOTOR_B_STEP, MOTOR_B_DIR,1,10);
      accel.getEvent(&event);
      Elevation = GetElevation(event.acceleration.x,event.acceleration.y,event.acceleration.z);
    }
    while (Elevation < 0){
      StepMotor(MOTOR_B_STEP, MOTOR_B_DIR,0,10);
      accel.getEvent(&event);
      Elevation = GetElevation(event.acceleration.x,event.acceleration.y,event.acceleration.z);
    }
    ElevationCalibrated = true;
  }
//BEGIN CALIBRATION ROUTINE OF AZIMUTH
  while (AzimuthCalibrated == false){
  compass.read();
  Azimuth = compass.heading();//map(compass.heading()+7.0,+7,367,0,360);
  AzimuthCalibrated = CalibrateAzimuth(Azimuth,AzimuthCalibrated);
  }
///////////////////////////////////////////////////////////////////////////////////////////////
//  SEARCHING AND WAITING FOR DATA
//////////////////////////////////////////////////////////////////////////////////////////////
int DataValues[4]; 
while(WaitingForOrders == true){
  String Data = getXbeeData();
  if (Data.length() > 0){
    Serial.println(Data);
    convertCSVtoInt(DataValues, 4, Data);
    //TEST LINE TEST LINE TEST LINE
        for (int i = 0; i < 4; i++){
          Serial.println(DataValues[i]);
        }
        WaitingForOrders = false;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////
//    DATA RECIEVED, MOVE TO RISE AZIMUTH AND NOW WAIT FOR SATELLITE TRANSIT
//////////////////////////////////////////////////////////////////////////////////////////////
  compass.read();
  Serial.print(compass.heading());
  MoveToAzimuth(DataValues[0]);
  bool Recieved = true;
  do{
    GOCommand = getXbeeData();
    Serial.println(GOCommand.length());
    if (GOCommand.length() == 2){
      Recieved = false;
      Serial.println("Recieved the go command");
    }
    else if(GOCommand.length() > 2){
      WaitingForOrders = true;
      AzimuthCalibrated = false;
      Recieved = false;
    }
  }while (Recieved);

///////////////////////////////////////////////////////////////////////////////////////////////
//    SATELLITE PASSING OVER HEAD BEGIN TRACKING
//////////////////////////////////////////////////////////////////////////////////////////////
  GOCommand = "";
  int DistanceToMove = (DataValues[0]-DataValues[2]);
  if (DistanceToMove > 0){
    TrackSat(abs(DistanceToMove),DataValues[3],DataValues[1],1);
  }
  else{
    TrackSat(abs(DistanceToMove),DataValues[3],DataValues[1],0);
  }
///////////////////////////////////////////////////////////////////////////////////////////////
//    SATELLITE OVER HORIZON, LOOP AND RECALIBRATE
//////////////////////////////////////////////////////////////////////////////////////////////
  WaitingForOrders = true;
  AzimuthCalibrated = false;
  ElevationCalibrated = false;
}
                  //*END OF MAIN PROGRAM LOOP*\\
//********************************************************************************************\\
//********************************************************************************************\\

/*=============================================================================================
                      //* FUNCTION DECLARATIONS *\\
-----------------------------------------------------------------------------------------------*/
//          A List Of Custom Functions Used in the main loop
/*=============================================================================================
      "InitialiseSensors"   *Function for Initialising Sensors - Only used at startup
-----------------------------------------------------------------------------------------------*/
void InitialiseSensors(void){
  Wire.begin();
  Wire.beginTransmission(0x20);
  Wire.write(expansion_output);
  Wire.endTransmission();
  Serial.begin(9600);
  xBeeSerial.begin(9600);
  xbee.begin(xBeeSerial);
  //MAGNETOMOTER/COMPASS INITIALISATION
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-496, -504, -123};
  compass.m_max = (LSM303::vector<int16_t>){+706, +549, +855};
  //ACCELEROMETER INITIALISATION
  if(!accel.begin()){
    Serial.println("No Acceleromter Sensor Detected");
    while(1);
  }
  accel.getSensor(&sensor);
}
/*=============================================================================================*/

/*=============================================================================================
    "GetElevation"  *Function for getting Elevation - Used as a part of the calibration routine
-----------------------------------------------------------------------------------------------*/
float GetElevation(float XAccel,float YAccel,float ZAccel){
  float XReading = XAccel/2.04;
  float YReading = YAccel/2.04;
  float ZReading = ZAccel/2.04;
  float Elevation = -(atan(YReading/(sqrt(pow(XReading,2.0) + pow(ZReading,2.0)))) * 180/PI) + 1.50;
  return Elevation;
  }
/*==============================================================================================*/

/*===============================================================================================
    Function for Calibrating Azimuth Motor - Used as a part of the calibration routine
-------------------------------------------------------------------------------------------------*/
  bool CalibrateAzimuth(float Azimuth, bool Calibrated){
    if (Azimuth > 180.0 && Azimuth < 358.0){
    StepMotor(MOTOR_A_STEP, MOTOR_A_DIR,0,10);
  }
  if (Azimuth > 0.0 && Azimuth <= 180.0){
    StepMotor(MOTOR_A_STEP, MOTOR_A_DIR,1,10);
  }
  if (Azimuth >= 358.0 || Azimuth <= 1){
    Serial.println("Pointing North");
    Calibrated = true;
  }
  return Calibrated;
  }
 /*============================================================================================*/
 
/*==============================================================================================
    "StepMotor"   *Function for Step Motor - Used For Stepping the Motors (GENERAL PURPOSE)
-----------------------------------------------------------------------------------------------*/
void StepMotor(int MotorPin, int MotorDirectionPin, int Direction, int Speed){
  if (Direction == 0){
    digitalWrite(MotorDirectionPin,Direction);
  }
  else if (Direction == 1){
    digitalWrite(MotorDirectionPin,Direction);
  }
  digitalWrite(MotorPin,HIGH);
  delayMicroseconds(500);
  digitalWrite(MotorPin, LOW);
  delay(Speed);
  }
  /*========================================================================================*/

 /*===========================================================================================
   "TrackSat"   *Fucntion for Tracking - Used for tracking satelittes through a transit
----------------------------------------------------------------------------------------------*/
void TrackSat(int AzimuthAngle,int TimeForMove,int ElevationAngle, int Direction){
  unsigned long TimeNow;
  unsigned long AZIPreviousTime = 0;
  unsigned long ELEVPreviousTime = 0;
  int AzimuthStepsCounter = 0;
  int ElevationStepsCounter = 0;
  float StepsToBeTakenAzi = round((float)AzimuthAngle*3200.0/360.0);
  float StepsToBeTakenElev = 2*round((float)ElevationAngle*3200.0/360.0);
  float AZISpeed = StepsToBeTakenAzi/TimeForMove;
  float ELEVSpeed = StepsToBeTakenElev/TimeForMove;
  float AZIPeriod = 1.0/AZISpeed*1000;
  float ELEVPeriod = 1.0/ELEVSpeed*1000;
  do{
    TimeNow = millis();
      if (TimeNow - AZIPreviousTime >= AZIPeriod){
        if (AzimuthStepsCounter <= StepsToBeTakenAzi){
        StepMotor(MOTOR_A_STEP, MOTOR_A_DIR,Direction,50);
        AzimuthStepsCounter++;
        AZIPreviousTime = millis();
        }
      }
      TimeNow = millis();
      if (TimeNow - ELEVPreviousTime >= ELEVPeriod){
        //Serial.println(ElevationStepsCounter);
        if (ElevationStepsCounter <= round((float)ElevationAngle*3200.0/360.0)){
          StepMotor(MOTOR_B_STEP, MOTOR_B_DIR,0,50);
          ELEVPreviousTime = millis();
        }
        else{
          StepMotor(MOTOR_B_STEP, MOTOR_B_DIR,1,50);
        }
        ElevationStepsCounter++;
        ELEVPreviousTime = millis();
      }
    }while(AzimuthStepsCounter < StepsToBeTakenAzi);
    ElevationStepsCounter = 0;
    AzimuthStepsCounter = 0;
  return;
}
  /*=============================================================================================*/


 /*===============================================================================================
   Fucntion for Moving to Azimuth - Used to preposition tracker in prep for rise
--------------------------------------------------------------------------------------------------*/
void MoveToAzimuth(int Angle){
  int AzimuthStepsCounter = 0;
  float StepsToBeTakenAzi = round((float)Angle*3200.0/360.0);
  float StepsToBeTakenElev = round((float)Elevation*3200.0/360.0);
  do{
    StepMotor(MOTOR_A_STEP, MOTOR_A_DIR,0,10);
    AzimuthStepsCounter++;
  }while(AzimuthStepsCounter <= StepsToBeTakenAzi);
  return;
}
  /*============================================================================================*/


 /*=============================================================================================
   "getXbeeData"    *Fucntion for Getting Data - Used for Recieving from transmitting Xbee
------------------------------------------------------------------------------------------------*/
String getXbeeData() {
  String outputString = "";
    xbee.readPacket();
    if (xbee.getResponse().isAvailable()) {  
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
        xbee.getResponse().getZBRxResponse(rx);           
        int i;
        for (i = 0; i < rx.getDataLength(); i++){
          outputString += char(rx.getData(i));
        }    
      } else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
        xbee.getResponse().getModemStatusResponse(msr);
      }
    }
    return outputString;
}
/*============================================================================================*/

/*=============================================================================================
   Fucntion for Parsing Data Recieved - Used for Extracting data from packet
-----------------------------------------------------------------------------------------------*/
void convertCSVtoInt(int *arr, int arrSize, String csvStr){
  // Takes a pointer to an array, the size of the array and a string containing the data
      String subStr = csvStr;
      arr[0] = subStr.toInt();
      for (int i = 1; i < arrSize; i++){
        subStr = subStr.substring(subStr.indexOf(',')+1);
        arr[i] = subStr.toInt();
      }
}
/*============================================================================================*/

