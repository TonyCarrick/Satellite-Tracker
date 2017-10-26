//The following program is apart of a satellite tracking system.  It takes data that is sent via
//a Raspberry Pi and uses this data to control the movement of two stepper motors and an aerial
//In order to track satellites across the sky.
//Code written by Tony Carrick, latest compile on 25/10/2017
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
    FUNCTION PROTOTYPES - See function descriptions below main program loop
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
    SETUP LOOP - Calls functions that initialises sensors (Setting up Xbee comms etc.)
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
//BEGIN CALIBRATION ROUTINE OF ELEVATION  - While the antenna is not level, adjust servo motors until it is level.
  while (ElevationCalibrated == false){
    accel.getEvent(&event);            //Get the acceleromter values
    Elevation = GetElevation(event.acceleration.x,event.acceleration.y,event.acceleration.z); //Feed accel reading into Elevation Calibration function
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
    ElevationCalibrated = true;       //The device is now calibrated
  }
//BEGIN CALIBRATION ROUTINE OF AZIMUTH - Move the antenna until the acceleromter reads True North
  while (AzimuthCalibrated == false){
  compass.read();         //Measure the compass reading
  Azimuth = compass.heading();  //Store the resulting heading
  AzimuthCalibrated = CalibrateAzimuth(Azimuth,AzimuthCalibrated); //Call the Azimuth Calibration function until calibrated
  }
///////////////////////////////////////////////////////////////////////////////////////////////
//  SEARCHING AND WAITING FOR DATA - The code loops and searches for incoming data (waiting for orders).  It stays here until data recieved.
//////////////////////////////////////////////////////////////////////////////////////////////
int DataValues[4]; 
while(WaitingForOrders == true){            
  String Data = getXbeeData();
  if (Data.length() > 0){
    convertCSVtoInt(DataValues, 4, Data);      // Once data recieved, extract it from packet using "convertCSV to int
        for (int i = 0; i < 4; i++){
          Serial.println(DataValues[i]);
        }
        WaitingForOrders = false;             //Flag triggers when packet recieved and causes a loop exit so main program continues
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////
//    DATA RECIEVED, MOVE TO RISE AZIMUTH AND NOW WAIT FOR SATELLITE TRANSIT
//////////////////////////////////////////////////////////////////////////////////////////////
//Now that the data has been recieved, move to the rise position in anticipation of the satellite rise and wait for it to appear.
  compass.read();
  Serial.print(compass.heading());
  MoveToAzimuth(DataValues[0]);         //Move to the rise position
  bool Recieved = true;
  do{                                     //In a similar way to above, loop while waiting for a "GO" command 
    GOCommand = getXbeeData();
    Serial.println(GOCommand.length());
    if (GOCommand.length() == 2){
      Recieved = false;
    }
    else if(GOCommand.length() > 2){    //If a "CANCEL" command is recieved, restart the program go back to the calibrate sequence above
      WaitingForOrders = true;
      AzimuthCalibrated = false;
      Recieved = false;
    }
  }while (Recieved);                    //End the waiting for "GO" command loop

///////////////////////////////////////////////////////////////////////////////////////////////
//    SATELLITE PASSING OVER HEAD BEGIN TRACKING - At this stage, GO command would have been received, a will track satellite
//////////////////////////////////////////////////////////////////////////////////////////////
  GOCommand = "";                         //Clear "GO" command
  int DistanceToMove = (DataValues[0]-DataValues[2]);   //Find the total distance of the move from the current coordinates, the setting coordinates
  //Diretion handling for the movement
  if (DistanceToMove > 0){
    TrackSat(abs(DistanceToMove),DataValues[3],DataValues[1],1);      //Satellite Tracking completed within this function
  }
  else{
    TrackSat(abs(DistanceToMove),DataValues[3],DataValues[1],0);      //Same as above, with different direction argument
  }
///////////////////////////////////////////////////////////////////////////////////////////////
//    SATELLITE OVER HORIZON, LOOP AND RECALIBRATE - Reset all flags to uncalibrated, making system calibrate again, and loop.
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
  //Magnetometer needs below functions to ensure it scales local magnetic field to True North (Magnetic declination of Townsville is +7.40 degrees
  compass.m_min = (LSM303::vector<int16_t>){-496, -504, -123};      //This sets the minimum values for 3 axes of magnetometer
  compass.m_max = (LSM303::vector<int16_t>){+706, +549, +855};      //This sets the maximum values for 3 axes of magnetometer
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
  float XReading = XAccel/2.04;   //2.04 value is the Gain of the accelerometer. (calculated and helps scaling in a similar
  float YReading = YAccel/2.04;   //way to the magnetometer
  float ZReading = ZAccel/2.04;
  float Elevation = -(atan(YReading/(sqrt(pow(XReading,2.0) + pow(ZReading,2.0)))) * 180/PI) + 1.50;  //Work out the Elevation angle
  return Elevation;
  }
/*==============================================================================================*/

/*===============================================================================================
    Function for Calibrating Azimuth Motor - Used as a part of the calibration routine
-------------------------------------------------------------------------------------------------*/
  bool CalibrateAzimuth(float Azimuth, bool Calibrated){
    //The following is direction handling based on current position
    if (Azimuth > 180.0 && Azimuth < 358.0){
    StepMotor(MOTOR_A_STEP, MOTOR_A_DIR,0,10);
  }
  if (Azimuth > 0.0 && Azimuth <= 180.0){
    StepMotor(MOTOR_A_STEP, MOTOR_A_DIR,1,10);
  }
  if (Azimuth >= 358.0 || Azimuth <= 1){    //Display pointing north when within 3 degrees of North
    //Serial.println("Pointing North");     //UNCOMMENT THIS LINE IF TRYING TO MONITOR VIA SERIAL
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
  digitalWrite(MotorPin,HIGH);          //Motor takes one step per detected rising edge.
  delayMicroseconds(500);
  digitalWrite(MotorPin, LOW);
  delay(Speed);                         //This delay sets the speed of the movement
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
  float StepsToBeTakenAzi = round((float)AzimuthAngle*3200.0/360.0);      //Calculated Steps to move the required distance
  float StepsToBeTakenElev = 2*round((float)ElevationAngle*3200.0/360.0);
  float AZISpeed = StepsToBeTakenAzi/TimeForMove;                         //Calculate the amount of steps per second
  float ELEVSpeed = StepsToBeTakenElev/TimeForMove;
  float AZIPeriod = 1.0/AZISpeed*1000;                                    //Set Period
  float ELEVPeriod = 1.0/ELEVSpeed*1000;
  do{//The following code steps every 'x' amount of seconds as determined by the above calculations
    TimeNow = millis();
      if (TimeNow - AZIPreviousTime >= AZIPeriod){
        if (AzimuthStepsCounter <= StepsToBeTakenAzi){
        StepMotor(MOTOR_A_STEP, MOTOR_A_DIR,Direction,50);
        AzimuthStepsCounter++;
        AZIPreviousTime = millis();
        }
      }
      //The below does the same as above, only for the elevation
      TimeNow = millis();
      if (TimeNow - ELEVPreviousTime >= ELEVPeriod){
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
    ElevationStepsCounter = 0;    //Reset Steps counter and return to the main program
    AzimuthStepsCounter = 0;
  return;
}
  /*=============================================================================================*/


 /*===============================================================================================
   Fucntion for Moving to Azimuth - Used to preposition tracker in prep for rise
--------------------------------------------------------------------------------------------------*/
//These calculations are the same as above, only without the time dependency (i.e. moves at set speed)
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
  String outputString = "";     //Set variable to store the data
    xbee.readPacket();
    if (xbee.getResponse().isAvailable()) {  
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {  //If a packet is recieved
        xbee.getResponse().getZBRxResponse(rx);           
        int i;
        for (i = 0; i < rx.getDataLength(); i++){
          outputString += char(rx.getData(i));                //parse data into the output string variable
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
  // Takes a pointer to an array, the size of the array and a string containing the data (coverting bytes to human readable values
      String subStr = csvStr;
      arr[0] = subStr.toInt();
      for (int i = 1; i < arrSize; i++){
        subStr = subStr.substring(subStr.indexOf(',')+1);
        arr[i] = subStr.toInt();
      }
}
/*============================================================================================*/

