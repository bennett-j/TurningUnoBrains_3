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
  prcInitSerial();
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

 // declare movement parameters
  int stepsRemaining;
  int moveDir;
  
void loop() {

  /*
   * DECLARATIONS
   */
  // declare flags
  static bool ortEnabled = false;  
  static bool doScanRead = false;
  static bool doScanMove = false;

  // declare scan parameters
  static int scanStart_stp;
  static int scanEnd_stp;
  static int scanRes_stp;

  

  // declare label data
  char jsonLabel[4]; // for parsing at other end
  int L_ndx = 0; // label index
  bool newData = false; // to know whether to send json

  // allocate memory for json document
  StaticJsonDocument<200> jData;


  /*
   * ---------------------  OPERATIONS  ----------------------
   * will complete one of the following on each loop iteration
   * if none executed then can check if a new command received
   */
  
  /* MOVE ACTION */
  if (stepsRemaining > 0) {
    // get stepper to do next action
    stepper.nextAction();
    
    // decrement steps remaining
    stepsRemaining--;

    // update current position according to direction (+/- 1)
    currentPos += moveDir;

    if (stepsRemaining == 0 && !doScanRead) {
      Serial.println(currentPos);
      Serial.println("<Chariot Move Complete>");
      stepper.stop(); // because I'm paranoid
    }
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
  }
  
  /* SCAN MOVEMENT */
  else if (doScanMove) {
    // evaluate if next step would go beyond end point
    // note set to be end exclusive
    if ((currentPos + scanRes_stp) < scanEnd_stp) {
      // if next step not beyond end point...
      
      // move number of steps to next reading location
      chariot_move(scanRes_stp);
      
      // and set flag to take a reading at that point
      doScanRead = true;
    }
    else {
      // else scan finished
      Serial.println("<Scan Complete>");
    }
    // set move flag
    doScanMove = false;
  }
  
  /* CMD MANAGEMENT */
  else {
    //get command character to invoke corresponding function
    char cmd = recvCommandChar();
    
    switch (cmd){
    // note curly brackets required where declaration happens within a case
    
      /* SCAN PROCEDURE */    
      case 'S': {
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
        //###review below line###
        stepper.setSpeedProfile(stepper.LINEAR_SPEED, accel, accel);

        // initiate move to start location
        chariot_startMoveTo(scanStart_stp);        
        
        // set scan flag
        doScanRead = true;
        break;
      }      
      
      /* MOVE CHARIOT BY */
      case 'M': {
         // <{"posTo":200}>
        /* recieve distance to move */
        // read phrase from serial
        char str[32];
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
        
        // fetch values and start move
        int moveTo_stp = instruction["posTo"];
        chariot_startMoveTo(moveTo_stp); 
        break;
      }
         
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
        
      /* LASER: GO LIGHT */
      case 'L':
        digitalWrite(laserPin, HIGH);
        break;  
  
      /* LASER: GO DARK */
      case 'D':
        digitalWrite(laserPin, LOW);
        break;
  
      /* HOME CHARIOT */
      case 'H': 
        // move to home limit
        prcMoveToLimit(-1, driveEndLim);
        
        // set position to 0
        currentPos = 0;
        Serial.println(currentPos);
        Serial.println("<Home Chariot Complete>");
        break; 
      
      /* GET RANGE */
      case 'R':
        // move to idle end limit
        prcMoveToLimit(+1, idleEndLim);

        /*stepper.startMove(200);
        
        while (digitalRead(idleEndLim) == LOW) {
          stepper.nextAction();
          currentPos++;
          //Serial.println(currentPos);
        }      
        */
        // current position set in procedure
        Serial.println("<Range Find Complete>");
        Serial.print("Range is: "); Serial.println(currentPos);
        break;
        
        // go home
        //chariot_goto(0);
        break;
      case 'G':
        Serial.print("Current Position: "); Serial.println(currentPos);
  
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
    jsonLabel[L_ndx++] = 'o';
    
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
void prcInitSerial() {
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


//new
void prcMoveToLimit(int dir, int pin) {
  long stuff = dir * 20000;
  stepper.startMove(stuff);
  Serial.print("stuff: "); Serial.println(stuff);
  Serial.print("dir: "); Serial.println(dir);
  Serial.print("motorSteps: "); Serial.println(motorSteps);

  while (digitalRead(pin) == LOW) {
    stepper.nextAction();
    currentPos += dir;
    Serial.println(currentPos);
  }
  stepper.stop(); // clear remaining steps
}


void chariot_startMoveTo(int pos) {
  chariot_startMove(pos - currentPos);
}


//new
void chariot_startMove(int steps) {
  stepper.startMove(steps);
  
  // params
  stepsRemaining = abs(steps);
  moveDir = steps/stepsRemaining;
}


//new
void chariot_goto(int pos) {
  chariot_move(pos - currentPos);
}


//new
void chariot_move(int steps){
  stepper.move(steps);
  currentPos += steps;
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
