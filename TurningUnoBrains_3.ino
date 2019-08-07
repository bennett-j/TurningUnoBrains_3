#include "DRV8825.h"
#include "Adafruit_VL6180X.h"
#include "ArduinoJson.h"
#include "Adafruit_BNO055.h"


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)


// define pins
const int dirPin = 4;
const int stepPin = 5;
const int driveEndLim = 6;
const int idleEndLim = 7;
const int laserPin = 8;
const int enablePin = 3;

// define stepper parameters
const int motorSteps = 400;
const int microsteps = 1;
int rpm = 90;
int accel = 8000;

// define other parameters
const int ortInterval = 100;
int lastOrtRead = millis(); //stores last time an orientation reading was taken
int currentPos = 0; 

/*** CREATE HARDWARE OBJECTS ***/
// stepper motor 
DRV8825 stepper(motorSteps, dirPin, stepPin, enablePin);

// TOF distance sensor
Adafruit_VL6180X vl = Adafruit_VL6180X();

// orientation sensor
Adafruit_BNO055 bno = Adafruit_BNO055();

/*
 *  =========
 *    SETUP
 *  =========
 */
void setup() {

  /*** INIT SERIAL ***/
  // initialise serial communication with control device
  initSerial();
  Serial.println("<Serial Communication Verified>");


  /*** SETUP DISTANCE SENSOR ***/
  // check for VL6180X distance sensor (vl)
  if (!vl.begin()) {
    Serial.println("<Failed to find sensor. Please restart.>");
    while(true);  //cannot continue so halting exection here
  }
  Serial.println("<Distance Sensor Found>");


  /*** SETUP ORIENTATION SENSOR ***/
  // chekc for sensor (bno)
  if(!bno.begin()) {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(true);
  }
  delay(1000); //really?
  bno.setExtCrystalUse(true); //what's this for?

  Serial.println("<Orientation Sensor Found>");
  

  /*** SETUP GENERAL ***/
  //set switches as pull-up inputs. NB switch connected to ground and pin will be high when switch open
  pinMode(driveEndLim,INPUT_PULLUP); 
  pinMode(idleEndLim,INPUT_PULLUP);

  //set laser pin as output
  pinMode(laserPin, OUTPUT);

  //begin motor: rpm, microsteps set to 1 for full step
  stepper.begin(rpm, microsteps);
  
  //print Uno setup complete
  Serial.println("<Uno Setup Complete>");
}

/*
 *  ========
 *    LOOP
 *  ========
 */


bool opInProg = false;
bool ortEnabled = false;
bool doScanRead = false;
bool doScanMove = false;
bool doMoveAction = false;

int scanStart_stp;
int scanEnd_stp;
int scanRes_stp;
//int scanMovesRemaining;
int stepsRemaining;
int moveDir;
char op;

void loop() {

  char jsonLabel[4]; // for parsing at other end
  int L_ndx = 0; // label index
  bool newData = false; // to know whether to send json

  char cmd;

  // allocate memory
  StaticJsonDocument<200> jData;

  /*  
   *  if steps to move, then move
   *  else, if in a scan and ready for reading, do that
   *  else, if in a scan and not time for reading but time for move, do that
   *  else, if none of these, then not in an operation so can check for new cmd char
   *  
   *  finally, if time, get an orientation reading  
   */

  /* MOVE ACTION */
  if (stepsRemaining > 0) {
    // if steps to move, then move
    
    //get stepper to do next action
    //unsigned wait_time_micros = stepper.nextAction();
    stepper.nextAction();
    
    // decrement steps remaining
    stepsRemaining--;

    // update current position according to direction (+/- 1)
    currentPos = currentPos + moveDir;
  }
  /* SCAN READING */
  else if (doScanRead) {
    //get (single) distance reading
    float distance = getDistRead(); //error if -1  

    // add values to document
    jData["position"] = currentPos;
    jData["distance"] = distance;

    // add 's' to json label
    jsonLabel[L_ndx++] = 's';

    // there is data to send
    newData = true;

    // set flags
    doScanRead = false;
    doScanMove = true;

    Serial.println("Done Reading");
  }
  /* SCAN MOVEMENT */
  else if (doScanMove) {
    ///### want an else so doesn't do both in same loop ###  
    // move a step
    chariot_move(scanRes_stp);
    
    // set flags
    doScanMove = false;
    if ((currentPos + scanRes_stp) < scanEnd_stp) {
      // if next step less than end, then continue
      doScanRead = true;
    }
    else {
      // else finish scan
      Serial.println("<Scan Complete>");
    }
  }
  /* CMD MANAGEMENT */
  else {
    //get command character to invoke corresponding function
    cmd = recvCommandChar();
    
    switch (cmd){
    // note curly brackets are needed in a case if declaration happens within
    
    /*** SCAN PROCEDURE ***/    
      case 'S': //scan
        {
        opInProg = true;
        op = 'S';
        // <{"start":50, "end":250, "resolution":5}>
        /* get additional info: start, end, resolution */
        // read phrase from serial
        char str[64];
        recvPhrase(str);
              
        // allocate JSON document
        StaticJsonDocument<200> instruction;
  
        // deserialise transmission
        DeserializationError err = deserializeJson(instruction, str);
  
        // test if parsing succeeds
        if (err) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(err.c_str());
          return; //this will exit case
        }
        
        // fetch values
        scanStart_stp = instruction["start"];
        scanEnd_stp = instruction["end"];
        scanRes_stp = instruction["resolution"];
  
        /* prepare hardware */
        //move chariot to start position
        //chariot_goto(scanStart_stp);
  
        //###review below line###
        stepper.setSpeedProfile(stepper.LINEAR_SPEED, accel, accel);


        /*movement*/
        chariot_startMove(scanStart_stp - currentPos);        
        doScanRead = true;
        

        
        }
        break; 
  
      /* ENABLE ORIENTATION */
      case 'Q': {
        ortEnabled = true;
        break;
      }

      /* DISABLE ORIENTATION */
      case 'W': {
        ortEnabled = false;
        break;
      }
        
    /*** HOME CHARIOT ***/
      case 'H': 
        prcHomeMotor();
        Serial.println("<Home Chariot Complete>");
        break; 
  
    /*** MOVE CHARIOT BY ***/
      case 'M':
        {
        opInProg = true;
        op = 'M';
         
        /*movement*/
        chariot_startMove(200); 
        
      

         
        /* recieve distance to move */
        // read phrase from serial
        /*char str[32];
        recvPhrase(str);
              
        // allocate JSON document
        StaticJsonDocument<50> instruction;
  
        // deserialise transmission
        DeserializationError err = deserializeJson(instruction, str);
  
        // test if parsing succeeds
        if (err) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(err.c_str());
          return; //this will exit case
        }
        
        // fetch values
        const int toMove_stp = instruction["stepsToMove"];
  
        //move
        smove(toMove_stp);
  
        //send completion confirmation
        Serial.println("<Chariot Moved>");
  
        sendPosition();
  
        Serial.println("<Move Complete>");*/  
        }
        break;
  
    /*** LASER: LIGHT ***/
      case 'L':
        digitalWrite(laserPin, HIGH);
        break;  
  
    /*** LASER: DARK ***/
      case 'D':
        digitalWrite(laserPin, LOW);
        break;
  
    /*** GET RANGE ***/
      case 'R':
        {
        // move to the right until limit switch engaged
        // use smove so current position is updated
        stepper.setRPM(90);
        while(digitalRead(idleEndLim) == LOW){
          chariot_move(2);
          //delay(50);
        }
        stepper.setRPM(rpm);
        
        // send current position
        sendPosition();
  
        // go home
        chariot_goto(0); 
        }
        break;
  
    /*** UPDATE PARAMETERS ***/
      case 'U':
        {
        //code
        }
        break;
        
    } //end switch
  }

    
  
  /* ORIENTATION READING */
  int currentMillis = millis();
  if (ortEnabled && (currentMillis - lastOrtRead >= ortInterval)) {
    
    // store current millis
    lastOrtRead = currentMillis;
    
    // get orientation from sensor
    imu::Quaternion quat = bno.getQuat();
          
    // add values to document
    jData["qW"] = quat.w();
    jData["qX"] = quat.x();
    jData["qY"] = quat.y();
    jData["qZ"] = quat.z();

    // add 'o' to json label
    jsonLabel[L_ndx] = 'o';
    L_ndx++;

    // there is data to send
    newData = true;
  }


  /* SEND DATA OVER SERIAL */
  if (newData) {

    // terminate label
    jsonLabel[L_ndx] = '\0';
    // add label to document
    jData["label"] = jsonLabel;

    // print json document to serial  
    Serial.print('<');
    serializeJson(jData, Serial);
    Serial.println('>');  
  }
} //loop


/*  
 *  ====================
 *  FUNCTION DEFINITIONS
 *  ====================
 */

//updated
void initSerial() {
  //function to begin and verify serial communication
  
  Serial.begin(250000);

  //wait for serial port to open
  while (!Serial) {
    delay(1);
  }
  
  //verify serial communication
  //print for other device to receive
  Serial.print("<u>");
 
  //wait to receive text from other device
  char r = 'x';  //initialise to something other than expected
  while (r != 'r'){
    r = Serial.read();
  }
  Serial.println();
}

void prcHomeMotor(){

  stepper.setRPM(90);  //rpm set experimentally so steps complete in loop timing

  //loop to rapidly take carrige to end and stop when there
  while(digitalRead(driveEndLim) == LOW){
    stepper.move(-2);
  }
  delay(200);

  //carrige backs off 10 steps and approaches slowly for accurate reading
  stepper.move(16);
  while(digitalRead(driveEndLim) == LOW){
    stepper.move(-1);
    delay(50);
  }

  //move away from the switch
  stepper.move(25);  
  //set current position to 0
  currentPos = 0;

  //set rpm back to standard
  stepper.setRPM(rpm);
}



//new
void chariot_startMove(int steps) {
  stepper.startMove(steps);

  // params
  stepsRemaining = abs(steps);
  moveDir = steps/stepsRemaining;

  
}



//new
void chariot_move(int steps){
  stepper.move(steps);
  currentPos += steps;
}

//new
void chariot_goto(int pos) {
  chariot_move(pos - currentPos);
}

int mm2step(int mm){
  int steps;
  steps = mm * 2;
  return steps;
}

int readInt() {
  while (true) {
    if (Serial.available() > 0) {
      char incoming[] = {'\0', '\0', '\0', '\0', '\0'};
      Serial.readBytesUntil(10, incoming, 5); //(until char, buffer, length)
      int integer = strtol(incoming, NULL, 10); //char string, ret null if error, base 10
      return integer;
    }
    delay(1);
  }
}

//new
float getDistRead() {
  //takes 10 measurements, discards first two, returns mean of next 8.
  
  //get sensor status
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE){
    
    //declare variables
    uint8_t measurements[10];
    int sum = 0;
    
    //get measurements
    for (int j=0; j<10; j++) {
      measurements[j] = vl.readRange();
    }

    //sum 2nd to 10th measurement
    for (int j=2; j<10; j++) {
      sum = sum + measurements[j];
    }

    //calculate and return mean
    float mean = sum/8.0;
    return mean;
  }
  else {
    //return -1 if sensor returns error
    //Serial.println("Sensor Error");
    return -1;
  }
}
          

//new
char recvCommandChar() {
  //function reads buffer until it finds '!' then returns the following character
  char alertChar = '!';
  char rc;
  
  if (Serial.available() > 0) {
    rc = Serial.read();

    if (rc == alertChar) {
      while(Serial.available() < 1);
      rc = Serial.read();
      return rc;
    }
  }
  
}

//new
char recvPhrase(char *str) {
  char startChar = '<';
  char endChar = '>';
  char rc;
  bool recvd = false;
  bool recvInProg = false;
  
  while (!recvd) {
    if (Serial.available() > 0) {
      rc = Serial.read();

      if (recvInProg == true) {
        if (rc != endChar) {
          //assign rc and postincrement
          *str++ = rc;
        }
        else {
          //end of transmission reached so terminate str
          *str = '\0';
          recvd = true;
        }
      }
      if (rc == startChar) {
        //start of transmission identified
        recvInProg = true;
      }
    }
  }
}

//new
void sendPosition() {
  /* generate and print JSON with data */
  // allocate memory
  StaticJsonDocument<50> toSend;

  // add values to document
  toSend["position"] = currentPos;
  
  // generate and send to serial port
  Serial.print('<');
  serializeJson(toSend, Serial);
  Serial.println('>');
}
