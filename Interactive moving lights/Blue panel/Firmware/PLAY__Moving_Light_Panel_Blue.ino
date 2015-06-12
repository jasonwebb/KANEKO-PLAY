/************************************************************************************************************
 Blue control panel for "Interactive moving lights" project for KANEKO's PLAY exhibition
 
 Author: Jason Webb
 Author website: http://jason-webb.info
 
 Hardware requirements:
 - (1) Teensy LC - http://www.pjrc.com/teensy/teensyLC.html
 - (3) 60mm colored dome buttons wired to switch pins 
       defined below to common ground - https://www.adafruit.com/search?q=60mm+arcade+button
 - (1) 100mm colored dome button wired to switch pin 
       defined below to ground - https://www.adafruit.com/search?q=100mm+arcade+button&b=1
 - (1) Small arcade joystick with each directional switch
       wired to switch pins defined below to common ground - https://www.adafruit.com/products/480
 - (1) Pan/tilt servo mechanism - http://www.robotshop.com/en/lynxmotion-pan-and-tilt-kit-aluminium2.html
 
 Description:
  Monitors all switch/button states and translates interactions into X and Y movements 
  of a 10W RGB LED floodlight attached to the tilt arm of a pan/tilt servo mechanism. 
  If no interactions have been detected in a specific period of time (defined by 
  servoSleepThreshold), the servos will be detached to both reduce internal mechanical
  wear and reduce heat.
  
 License:
  Sketch and all related files released publicly online are under the Creative Commons 
  Attribution-NonCommercial-ShareAlike 4.0 International.    

**************************************************************************************************************/

#include <Servo.h>

// Pin assignments
#define  redPin     9
#define  greenPin   10
#define  bluePin    11
#define  yellowPin  8

#define  leftPin   16
#define  rightPin  15
#define  upPin     17
#define  downPin   18

#define  panServoPin   1
#define  tiltServoPin  0

#define  joystickCheckInterval  50

// Count clock cycles until next input check
int joystickCheckCounter = 0;

// States of all buttons and switches
int redState, greenState, blueState, yellowState;
int leftState, rightState, upState, downState;

// Pan/tilt servos
Servo panServo, tiltServo;
int panPosition, tiltPosition;
int servoDelay = 15;

// Pan servo variables
int panHomePosition = 100;  // center of axis
int panSavedPosition;
int panMinPosition = 50;
int panMaxPosition = 180;

// Tilt servo variables
int tiltHomePosition = 115;
int tiltSavedPosition;
int tiltMinPosition = 110;
int tiltMaxPosition = 180;

// Non-blocking delay counter for servo updates to prevent excessive jittering 
int servoUpdateCounter = 0;
int servoUpdateInterval = 50;

// Servo sleep mode
int servoSleepCounter = 0;
int servoSleepThreshold = 1000000;  // number of clock cycles to wait before putting servos to sleep. Through experimentation this number comes out to be about 3-5 minutes.
boolean servosAsleep = false;

// Output useful information to Serial
boolean debug = true;

void setup() {
  // Set up arcade dome buttons
  pinMode(redPin, INPUT_PULLUP);
  pinMode(greenPin, INPUT_PULLUP);
  pinMode(bluePin, INPUT_PULLUP);
  pinMode(yellowPin, INPUT_PULLUP);
  
  // Set up joystick
  pinMode(leftPin, INPUT_PULLUP);
  pinMode(rightPin, INPUT_PULLUP);
  pinMode(upPin, INPUT_PULLUP);
  pinMode(downPin, INPUT_PULLUP);
  
  // Set up pan/tilt servos
  panServo.attach(panServoPin);
  tiltServo.attach(tiltServoPin);
  
  // Move servos to home position
  panServo.write(panHomePosition);
  tiltServo.write(tiltHomePosition);
  
  // Set current position of servos to home position
  panPosition = panHomePosition;
  tiltPosition = tiltHomePosition;

  // Initialize Serial connection now so that debug can flip without reboot
  Serial.begin(9600);
  
  // Wait for servos
  delay(servoDelay);
}

void loop() {
  // Get states of all buttons
  redState = digitalRead(redPin);
  greenState = digitalRead(greenPin);
  blueState = digitalRead(bluePin);
  yellowState = digitalRead(yellowPin);
  
  // Get states of all joystick directional switches
  leftState = digitalRead(leftPin);
  rightState = digitalRead(rightPin);
  upState = digitalRead(upPin);
  downState = digitalRead(downPin);
  
  // Servo sleep mode ----------------------------------------------------------------
  // - put servos to "sleep" when no buttons have been pressed for a while
  if(leftState && rightState && upState && downState && redState && greenState && blueState && yellowState) {
    if(!servosAsleep) {
      if(servoSleepCounter >= servoSleepThreshold) {
        Serial.println("Putting servos to sleep");
        
        // Move tilt and pan servos to safe positions before detaching them to prevent mechanical jams
        tiltServo.write(tiltHomePosition);
        delay(1000);
        panServo.write(panHomePosition);
        delay(1000);
        
        // Turn off PWM signals to servos, causing them to go "limp"
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
      
      // Restart PWM signals to servos to "wake" them
      panServo.attach(panServoPin);
      tiltServo.attach(tiltServoPin);
      
      panServo.write(panHomePosition);
      tiltServo.write(tiltHomePosition);
      
      delay(servoDelay);
      
      servosAsleep = false;
    }
  }

  // Use joystick to update servo positions ---------------------------------------- 
  if(joystickCheckCounter >= joystickCheckInterval && !servosAsleep) {
    if(!leftState)
      if(panPosition > panMinPosition)
        panPosition--;
    
    if(!rightState)
      if(panPosition < panMaxPosition)
        panPosition++;
    
    if(!downState)
      if(tiltPosition > tiltMinPosition)
        tiltPosition--;
        
    if(!upState)
      if(tiltPosition < tiltMaxPosition)
        tiltPosition++;
        
    joystickCheckCounter = 0;
  } else {
    joystickCheckCounter++;
  }
      
  // Use yellow button to return servos to home -------------------------------------
  if(!yellowState && !servosAsleep) {
    panPosition = panHomePosition;
    tiltPosition = tiltHomePosition;
  }
  
  // Use red button to shake side to side ------------------------------------
  if(!redState && !servosAsleep) {
    panSavedPosition = panPosition;
    
    // shake left/right 3 times
    for(int i=0; i<3; i++) {
      // move left a bit
      for(int x=0; x<15; x++) {
        panServo.write( constrain(panPosition - x, panMinPosition, panMaxPosition) );
        delay(servoDelay);
      }
      
      // move back to center
      for(int x=15; x>0; x--) {
        panServo.write( constrain(panPosition - x, panMinPosition, panMaxPosition) );
        delay(servoDelay);
      }
      
      // move right a bit
      for(int x=0; x<15; x++) {
        panServo.write( constrain(panPosition + x, panMinPosition, panMaxPosition) );
        delay(servoDelay);
      }
      
      // move back to center
      for(int x=15; x>0; x--) {
        panServo.write( constrain(panPosition + x, panMinPosition, panMaxPosition) );
        delay(servoDelay);
      }
    }
    
    // Return servo to original position
    panPosition = panSavedPosition;
    panServo.write(panPosition);
    delay(servoDelay);
  }
  
  // Use green button to shake up and down -----------------------------------------
  if(!greenState && !servosAsleep) {
    tiltSavedPosition = tiltPosition;
    
    // shake up/down 3 times
    for(int i=0; i<3; i++) {
      // move up a bit
      for(int y=0; y<15; y++) {
        tiltServo.write( constrain(tiltPosition - y, tiltMinPosition, tiltMaxPosition) );
        delay(servoDelay);
      }
      
      // move back to center
      for(int y=15; y>0; y--) {
        tiltServo.write( constrain(tiltPosition - y, tiltMinPosition, tiltMaxPosition) );
        delay(servoDelay);
      }
      
      // move down a bit
      for(int y=0; y<15; y++) {
        tiltServo.write( constrain(tiltPosition + y, tiltMinPosition, tiltMaxPosition) );
        delay(servoDelay);
      }
      
      // move back to center
      for(int y=15; y>0; y--) {
        tiltServo.write( constrain(tiltPosition + y, tiltMinPosition, tiltMaxPosition) );
        delay(servoDelay);
      }
    }
    
    tiltPosition = tiltSavedPosition;
    tiltServo.write(tiltPosition);
    delay(servoDelay);
  }
  
  // Use blue button to move in circle -----------------------------------------
  if(!blueState && servosAsleep) {
    panPosition += random(-15,15);
    tiltPosition += random(-15,15);
  }

  // Update the servos with non-blocking delay ------------------------------------
  if(servoUpdateCounter == servoUpdateInterval && !servosAsleep) {
    panPosition = constrain(panPosition, panMinPosition, panMaxPosition);
    tiltPosition = constrain(tiltPosition, tiltMinPosition, tiltMaxPosition);
    
    panServo.write(panPosition);
    tiltServo.write(tiltPosition);

    delay(servoDelay);
    
    servoUpdateCounter = 0;
  } else {
    servoUpdateCounter++;
  }

  // Output useful information to Serial ----------------------------------------
  if(debug) {
//    // Output colored dome button states
//    Serial.print("R=");
//    Serial.print(redState);
//    Serial.print(" G=");
//    Serial.print(greenState);
//    Serial.print(" B=");
//    Serial.print(blueState);
//    Serial.print(" Y=");
//    Serial.print(yellowState);
//    
    // Output  direction switch states
//    Serial.print(" --- ");
//    Serial.print("L=");
//    Serial.print(leftState);
//    Serial.print(" R=");
//    Serial.print(rightState);
//    Serial.print(" U=");
//    Serial.print(upState);
//    Serial.print(" D=");
//    Serial.println(downState);

//    Serial.println(tiltPosition);
  }

}
