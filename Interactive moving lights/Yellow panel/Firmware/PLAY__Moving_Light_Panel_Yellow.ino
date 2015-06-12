/************************************************************************************************************
 Yellow control panel for "Interactive moving lights" project for KANEKO's PLAY exhibition
 
 Author: Jason Webb
 Author website: http://jason-webb.info
 
 Hardware requirements:
 - (1) Teensy LC - http://www.pjrc.com/teensy/teensyLC.html
 - (5) Momentary pushbuttons wired to switch digital pins defined below to ground
 - (10) Toggle switches wired to switch digital pins defined below to ground
 - (1) Pan/tilt servo mechanism - http://www.robotshop.com/en/lynxmotion-pan-and-tilt-kit-aluminium2.html
 
 Description:
  Monitors all switch/button states and translates interactions into X and Y movements 
  of a 10W RGB LED floodlight attached to the tilt arm of a pan/tilt servo mechanism. 
  
  - Pan and tilt servos are directly controlled with momentary pushbuttons
  - Center pushbutton returns pan and tilt servos to home position
  - Quirky animations and movements triggered by toggles
  
  If no interactions have been detected in a specific period of time (defined by 
  servoSleepThreshold), the servos will be detached to both reduce internal mechanical
  wear and reduce heat.
  
 License:
  Sketch and all related files released publicly online are under the Creative Commons 
  Attribution-NonCommercial-ShareAlike 4.0 International.    

**************************************************************************************************************/

#include <Servo.h>

// Pin assignments ----------------------------------------------------
#define  buttonLeftPin   5
#define  buttonRightPin  6
#define  buttonUpPin     7
#define  buttonDownPin   8
#define  buttonCenterPin 4

#define  lowerToggleLeftPin    18
#define  lowerToggleRightPin   17
#define  lowerToggleUpPin      19
#define  lowerToggleDownPin    20
#define  lowerToggleCenterPin  21

#define  upperToggleLeftPin    2
#define  upperToggleRightPin   3
#define  upperToggleUpPin      1
#define  upperToggleDownPin    0
#define  upperToggleCenterPin  9

#define  panServoPin   23
#define  tiltServoPin  22

// Program constants --------------------------------------------------------
#define  buttonCheckInterval  50  // number of clock cycles to wait before checking all inputs again - poor man's smoothing filter
#define  toggleCheckInterval  50

#define  toggleActionState  0  // state that triggers an action when "changed" flag is flipped. 0 = up, 1 = down

// Count number of clock cycles until next update
int buttonCheckCounter = 0;
int toggleCheckCounter = 0;

// Current states of all buttons and toggles
int buttonLeftState, buttonRightState, buttonUpState, buttonDownState, buttonCenterState;
int lowerToggleLeftState, lowerToggleRightState, lowerToggleUpState, lowerToggleDownState, lowerToggleCenterState;
int upperToggleLeftState, upperToggleRightState, upperToggleUpState, upperToggleDownState, upperToggleCenterState;

// Previous states of all toggles
int lowerToggleLeftPreviousState, lowerToggleRightPreviousState, lowerToggleUpPreviousState, lowerToggleDownPreviousState, lowerToggleCenterPreviousState;
int upperToggleLeftPreviousState, upperToggleRightPreviousState, upperToggleUpPreviousState, upperToggleDownPreviousState, upperToggleCenterPreviousState;

// Flags to indicate when toggle states have changed
boolean lowerToggleLeftChanged = false, lowerToggleRightChanged = false, lowerToggleUpChanged = false, lowerToggleDownChanged = false, lowerToggleCenterChanged = false;
boolean upperToggleLeftChanged = false, upperToggleRightChanged = false, upperToggleUpChanged = false, upperToggleDownChanged = false, upperToggleCenterChanged = false;

// Pan/tilt servos
Servo panServo, tiltServo;
int panPosition, tiltPosition;
int servoDelay = 10;

// Pan servo variables
int panSavedPosition;
int panHomePosition = 110;
int panMinPosition = 0;
int panMaxPosition = 180;

// Tilt servo variables
int tiltSavedPosition;
int tiltHomePosition = 120;
int tiltMinPosition = 100;
int tiltMaxPosition = 180;

// Non-blocking delay counter for servo updates to prevent excessive jittering 
int servoUpdateCounter = 0;
int servoUpdateInterval = 50;

// Non-blocking delay counter for servo updates
int servoSleepCounter = 0;
int servoSleepThreshold = 1000000;
boolean servosAsleep = false;

boolean debug = false;

void setup() {
  // Set up all input pins
  pinMode(buttonLeftPin, INPUT_PULLUP);
  pinMode(buttonRightPin, INPUT_PULLUP);
  pinMode(buttonUpPin, INPUT_PULLUP);
  pinMode(buttonDownPin, INPUT_PULLUP);
  pinMode(buttonCenterPin, INPUT_PULLUP);
  
  pinMode(lowerToggleLeftPin, INPUT_PULLUP);
  pinMode(lowerToggleRightPin, INPUT_PULLUP);
  pinMode(lowerToggleUpPin, INPUT_PULLUP);
  pinMode(lowerToggleDownPin, INPUT_PULLUP);
  pinMode(lowerToggleCenterPin, INPUT_PULLUP);
  
  pinMode(upperToggleLeftPin, INPUT_PULLUP);
  pinMode(upperToggleRightPin, INPUT_PULLUP);
  pinMode(upperToggleUpPin, INPUT_PULLUP);
  pinMode(upperToggleDownPin, INPUT_PULLUP);
  pinMode(upperToggleCenterPin, INPUT_PULLUP);
  
  // Set up pan/tilt servos
  panServo.attach(panServoPin);
  tiltServo.attach(tiltServoPin);
  
  // Move servos to home position
  panServo.write(panHomePosition);
  tiltServo.write(tiltHomePosition);
  
  // Set current position of servos to home position
  panPosition = panHomePosition;
  tiltPosition = tiltHomePosition;

  // Set up a Serial connection for debugging info
  Serial.begin(9600);
}

void loop() {
  // Read states of all buttons and toggles ------------------------------------------
  buttonLeftState = digitalRead(buttonLeftPin);
  buttonRightState = digitalRead(buttonRightPin);
  buttonUpState = digitalRead(buttonUpPin);
  buttonDownState = digitalRead(buttonDownPin);
  buttonCenterState = digitalRead(buttonCenterPin);
  
  lowerToggleLeftState = digitalRead(lowerToggleLeftPin);
  lowerToggleRightState = digitalRead(lowerToggleRightPin);
  lowerToggleUpState = digitalRead(lowerToggleUpPin);
  lowerToggleDownState = digitalRead(lowerToggleDownPin);
  lowerToggleCenterState = digitalRead(lowerToggleCenterPin);
  
  upperToggleLeftState = digitalRead(upperToggleLeftPin);
  upperToggleRightState = digitalRead(upperToggleRightPin);
  upperToggleUpState = digitalRead(upperToggleUpPin);
  upperToggleDownState = digitalRead(upperToggleDownPin);
  upperToggleCenterState = digitalRead(upperToggleCenterPin);

  // Servo sleep mode ----------------------------------------------------------------
  // - put servos to "sleep" when no buttons have been pressed for a while
  if(buttonLeftState && buttonRightState && buttonUpState && buttonDownState && buttonCenterState &&
     !lowerToggleLeftChanged && !lowerToggleRightChanged && !lowerToggleUpChanged && !lowerToggleDownChanged) {
       
    if(!servosAsleep) {
      if(servoSleepCounter >= servoSleepThreshold) {
        Serial.println("Putting servos to sleep");
        
        // Move servos to safe positions before detaching them to prevent mechanical catching
        tiltServo.write(tiltHomePosition);
        delay(1000);
        panServo.write(panHomePosition);
        delay(1000);
        
        // Disable PWM signals to servos, causing them to go "limp"
        panServo.detach();
        tiltServo.detach();
        
        servosAsleep = true;
      } else {
        servoSleepCounter++;
      }
    }
  } else {
    servoSleepCounter = 0;
    
    // Wake up servos
    if(servosAsleep) {
      Serial.println("Waking up servos");
      
      // Restart PWM signals to servos
      panServo.attach(panServoPin);
      tiltServo.attach(tiltServoPin);
      
      panServo.write(panHomePosition);
      tiltServo.write(tiltHomePosition);
      
      delay(servoDelay);
      
      servosAsleep = false;
    }
  }

  // Check for state changes in toggles -----------------------------------------------------
  if(toggleCheckCounter >= toggleCheckInterval) {
    // Detect changes in toggles -----------------------------------------------------
    if(lowerToggleLeftState != lowerToggleLeftPreviousState)      lowerToggleLeftChanged = true;
    if(lowerToggleRightState != lowerToggleRightPreviousState)    lowerToggleRightChanged = true;
    if(lowerToggleUpState != lowerToggleUpPreviousState)          lowerToggleUpChanged = true;
    if(lowerToggleDownState != lowerToggleDownPreviousState)      lowerToggleDownChanged = true;
    if(lowerToggleCenterState != lowerToggleCenterPreviousState)  lowerToggleCenterChanged = true;
    
    if(upperToggleLeftState != upperToggleLeftPreviousState)      upperToggleLeftChanged = true;
    if(upperToggleRightState != upperToggleRightPreviousState)    upperToggleRightChanged = true;
    if(upperToggleUpState != upperToggleUpPreviousState)          upperToggleUpChanged = true;
    if(upperToggleDownState != upperToggleDownPreviousState)      upperToggleDownChanged = true;
    if(upperToggleCenterState != upperToggleCenterPreviousState)  upperToggleCenterChanged = true;
      
    // Process changes in LOWER toggle states into servo motions -------------------------------------
    if(lowerToggleLeftChanged && lowerToggleLeftState == toggleActionState) {
      panPosition -= 20;
      lowerToggleLeftChanged = false;
    }
    
    if(lowerToggleRightChanged && lowerToggleRightState == toggleActionState) {
      panPosition += 20;
      lowerToggleRightChanged = false;
    }
    
    if(lowerToggleUpChanged && lowerToggleUpState == toggleActionState) {
      tiltPosition += 20;
      lowerToggleUpChanged = false;
    }
    
    if(lowerToggleDownChanged && lowerToggleDownState == toggleActionState) {
      tiltPosition -= 20;
      lowerToggleDownChanged = false;
    }
    
    if(lowerToggleCenterChanged && lowerToggleCenterState == toggleActionState) {
      tiltPosition += random(10,30);
      panPosition -= random(10,30);
      lowerToggleCenterChanged = false;
    }
    
    // Process changes in UPPER toggle states into servo motions -------------------------------------
    if(upperToggleLeftChanged && upperToggleLeftState == toggleActionState) {
      panSavedPosition = panPosition;
      
      // shake left/right 3 times
      for(int i=0; i<3; i++) {
        // move left a bit
        for(int x=0; x<10; x++) {
          panServo.write( constrain(panPosition - x, panMinPosition, panMaxPosition) );
          delay(servoDelay);
        }
        
        // move back to center
        for(int x=10; x>0; x--) {
          panServo.write( constrain(panPosition - x, panMinPosition, panMaxPosition) );
          delay(servoDelay);
        }
        
        // move right a bit
        for(int x=0; x<10; x++) {
          panServo.write( constrain(panPosition + x, panMinPosition, panMaxPosition) );
          delay(servoDelay);
        }
        
        // move back to center
        for(int x=10; x>0; x--) {
          panServo.write( constrain(panPosition + x, panMinPosition, panMaxPosition) );
          delay(servoDelay);
        }
      }
      
      panPosition = panSavedPosition;
      panServo.write(panPosition);
      delay(servoDelay);
      
      upperToggleLeftChanged = false;
    }
    
    if(upperToggleRightChanged && upperToggleRightState == toggleActionState) {
      tiltSavedPosition = tiltPosition;
      
      // shake up/down 3 times
      for(int i=0; i<3; i++) {
        // move up a bit
        for(int y=0; y<10; y++) {
          tiltServo.write( constrain(tiltPosition - y, tiltMinPosition, tiltMaxPosition) );
          delay(servoDelay);
        }
        
        // move back to center
        for(int y=10; y>0; y--) {
          tiltServo.write( constrain(tiltPosition - y, tiltMinPosition, tiltMaxPosition) );
          delay(servoDelay);
        }
        
        // move down a bit
        for(int y=0; y<10; y++) {
          tiltServo.write( constrain(tiltPosition + y, tiltMinPosition, tiltMaxPosition) );
          delay(servoDelay);
        }
        
        // move back to center
        for(int y=10; y>0; y--) {
          tiltServo.write( constrain(tiltPosition + y, tiltMinPosition, tiltMaxPosition) );
          delay(servoDelay);
        }
      }
      
      tiltPosition = tiltSavedPosition;
      tiltServo.write(tiltPosition);
      delay(servoDelay);
      
      upperToggleRightChanged = false;
    }
    
    if(upperToggleUpChanged && upperToggleUpState == toggleActionState) {
      panPosition += random(-15,15);
      tiltPosition += random(-15,15);
      
      upperToggleUpChanged = false;
    }
    
    if(upperToggleDownChanged && upperToggleDownState == toggleActionState) {
      tiltPosition -= 20;
      upperToggleDownChanged = false;
    }
    
    if(upperToggleCenterChanged && upperToggleCenterState == toggleActionState) {
      tiltPosition += random(10,30);
      panPosition -= random(10,30);
      upperToggleCenterChanged = false;
    }
    
    // Save current button states to previous buffer -------------------------------------------
    lowerToggleLeftPreviousState = lowerToggleLeftState;
    lowerToggleRightPreviousState = lowerToggleRightState;
    lowerToggleUpPreviousState = lowerToggleUpState;
    lowerToggleDownPreviousState = lowerToggleDownState;
    lowerToggleCenterPreviousState = lowerToggleCenterState;
    
    upperToggleLeftPreviousState = upperToggleLeftState;
    upperToggleRightPreviousState = upperToggleRightState;
    upperToggleUpPreviousState = upperToggleUpState;
    upperToggleDownPreviousState = upperToggleDownState;
    upperToggleCenterPreviousState = upperToggleCenterState;
    
    toggleCheckCounter = 0;
  } else {
    toggleCheckCounter++;
  }
 
  // Use momentary buttons to update servo positions ---------------------------------------- 
  if(buttonCheckCounter >= buttonCheckInterval) {
    if(!buttonLeftState)
      if(panPosition > panMinPosition)
        panPosition--;
    
    if(!buttonRightState)
      if(panPosition < panMaxPosition)
        panPosition++;
    
    if(!buttonDownState)
      if(tiltPosition > tiltMinPosition)
        tiltPosition--;
        
    if(!buttonUpState)
      if(tiltPosition < tiltMaxPosition)
        tiltPosition++;
        
    buttonCheckCounter = 0;
  } else {
    buttonCheckCounter++;
  }
  
  // Use center momentary button to return servos to home -------------------------
  if(!buttonCenterState) {
    panPosition = panHomePosition;
    tiltPosition = tiltHomePosition;
  }
  
  // Update the servos with non-blocking delay ------------------------------------
  if(servoUpdateCounter == servoUpdateInterval) {
    panPosition = constrain(panPosition, panMinPosition, panMaxPosition);
    tiltPosition = constrain(tiltPosition, tiltMinPosition, tiltMaxPosition);
    
    panServo.write(panPosition);
    tiltServo.write(tiltPosition);

    delay(servoDelay);
    
    servoUpdateCounter = 0;
  } else {
    servoUpdateCounter++;
  }
  
  // Output useful information to Serial ------------------------------------------------
  if(debug) {
    // Momentary Button states
//    Serial.print("BL=");
//    Serial.print(buttonLeftState);
//    Serial.print(" BR=");
//    Serial.print(buttonRightState);
//    Serial.print(" BU=");
//    Serial.print(buttonUpState);
//    Serial.print(" BD=");
//    Serial.print(buttonDownState);
//    Serial.print(" BC=");
//    Serial.print(buttonCenterState);
//    Serial.println();

    // Lower toggle switch states
    Serial.print("LTL=");
    Serial.print(lowerToggleLeftState);
    Serial.print(" LTR=");
    Serial.print(lowerToggleRightState);
    Serial.print(" LTU=");
    Serial.print(lowerToggleUpState);
    Serial.print(" LTD=");
    Serial.print(lowerToggleDownState);
    Serial.print(" LTC=");
    Serial.print(lowerToggleCenterState);    
    Serial.println();

    // Upper toggle switch states
//    Serial.print("UTL=");
//    Serial.print(upperToggleLeftState);
//    Serial.print(" UTR=");
//    Serial.print(upperToggleRightState);
//    Serial.print(" UTU=");
//    Serial.print(upperToggleUpState);
//    Serial.print(" UTD=");
//    Serial.print(upperToggleDownState);
//    Serial.print(" UTC=");
//    Serial.print(upperToggleCenterState);    
//    Serial.println();
  }

}
