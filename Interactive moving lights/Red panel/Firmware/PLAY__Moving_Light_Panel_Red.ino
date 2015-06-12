/************************************************************************************************************
 Red control panel for "Interactive moving lights" project for KANEKO's PLAY exhibition
 
 Author: Jason Webb
 Author website: http://jason-webb.info
 
 Hardware requirements:
 - (1) Teensy LC - http://www.pjrc.com/teensy/teensyLC.html
 - (3) Piezo transducers wired to ground and analog input pins defined below, along with a 
       1M resistor between analog pin and ground - http://www.digikey.com/product-detail/en/CEB-20D64/102-1126-ND/412385
 - (1) Circular soft potentiometer wired as described in product description - https://www.adafruit.com/product/1069
 - (2) Momentary push buttons wired to switch digital pins defined below to common ground
 - (1) Pan/tilt servo mechanism - http://www.robotshop.com/en/lynxmotion-pan-and-tilt-kit-aluminium2.html
 
 Description:
  Monitors all switch/button states and translates interactions into X and Y movements 
  of a 10W RGB LED floodlight attached to the tilt arm of a pan/tilt servo mechanism. 
  
  - Pan servo is controlled by the two momentary push buttons
  - Tilt servo is controlled by circular softpot
  - Quirky animations are triggered by hitting the three piezo transducers
  
  If no interactions have been detected in a specific period of time (defined by 
  servoSleepThreshold), the servos will be detached to both reduce internal mechanical
  wear and reduce heat.
  
 License:
  Sketch and all related files released publicly online are under the Creative Commons 
  Attribution-NonCommercial-ShareAlike 4.0 International.    

**************************************************************************************************************/

#include <Servo.h>

// Pin assignments
#define  softpotPin      A0
#define  piezo1Pin       A6
#define  piezo2Pin       A8
#define  piezo3Pin       A9
#define  buttonLeftPin   A2
#define  buttonRightPin  A1
#define  panServoPin     0
#define  tiltServoPin    1

#define  bufferSize           30    // number of readings to take for each average
#define  triggerThreshold     550   // minimum value of piezo averages to count as a trigger
#define  triggerCooldownTime  2000  // number of clock cycles to wait before triggering again

#define  softpotCheckInterval  10  // number of clock cycles to wait before check buttons - poor man's smoothing filter
#define  inputCheckInterval    50

// Count clock cycles until next input check
int softpotCheckCounter = 0;
int inputCheckCounter = 0;

// Rolling average variables
int buffer1[bufferSize], buffer2[bufferSize], buffer3[bufferSize];
int total1 = 0, total2 = 0, total3 = 0;
int average1 = 0, average2 = 0, average3 = 0;

int bufferIndex = 0;

// Non-blocking cooldown counter
int cooldownCounter1 = 0;
int cooldownCounter2 = 0;
int cooldownCounter3 = 0;

// Button states
int buttonLeftState, buttonRightState;
int buttonCheckInterval = 50;

// Softpot variables
int softpotPosition;
int softpotMinPosition = 330;
int softpotMaxPosition = 660;
int softpotBuffer[bufferSize];
int softpotTotal = 0;
int softpotAverage = 0;

// Pan/tilt servos
Servo panServo, tiltServo;
int panPosition, tiltPosition;
int servoDelay = 15;

// Pan servo variables
int panHomePosition = 90;
int panSavedPosition;
int panMinPosition = 80;
int panMaxPosition = 140;

// Tilt servo variables
int tiltHomePosition = 115;
int tiltSavedPosition;
int tiltMinPosition = 100;
int tiltMaxPosition = 180;

// Non-blocking delay counter for servo updates
int servoUpdateCounter = 0;
int servoUpdateInterval = 50;

// Servo sleep mode
int servoSleepCounter = 0;
int servoSleepThreshold = 1000000;  // number of clock cycles to wait before putting servos to sleep. Through experimentation this number comes out to be about 3-5 minutes.
boolean servosAsleep = false;

boolean debug = true;

void setup() {
  // Set up pins
  pinMode(softpotPin, INPUT);
  pinMode(piezo1Pin, INPUT);
  pinMode(piezo2Pin, INPUT);
  pinMode(piezo3Pin, INPUT);
  pinMode(buttonLeftPin, INPUT_PULLUP);
  pinMode(buttonRightPin, INPUT_PULLUP);
  
  // Initialize buffers to be empty
  for(int i=0; i<bufferSize; i++) {
    buffer1[i] = 0;
    buffer2[i] = 0;
    buffer3[i] = 0;
  }
  
  // Set up pan/tilt servos
  panServo.attach(panServoPin);
  tiltServo.attach(tiltServoPin);
  
  // Move servos to home position
  panServo.write(panHomePosition);
  tiltServo.write(tiltHomePosition);
  
  // Set current position of servos to home position
  panPosition = panHomePosition;
  tiltPosition = tiltHomePosition;
  
  // Wait for servos to move
  delay(servoDelay);
  
  // Set up serial connection
  Serial.begin(9600);
}

void loop() {
  readSensors();

  // Get states of buttons -------------------------------------------------------
  buttonLeftState = digitalRead(buttonLeftPin);
  buttonRightState = digitalRead(buttonRightPin);

  // Servo sleep mode ----------------------------------------------------------------
  // - put servos to "sleep" when no buttons have been pressed for a while
  if(buttonLeftState && buttonRightState && (softpotAverage < softpotMinPosition || softpotAverage > softpotMaxPosition)) {
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

  // Use buttons to update pan servo --------------------------------------------- 
  if(inputCheckCounter >= inputCheckInterval) {
    if(!buttonLeftState)
      if(panPosition > panMinPosition)
        panPosition--;
    
    if(!buttonRightState)
      if(panPosition < panMaxPosition)
        panPosition++;      
    inputCheckCounter = 0;
  } else {
    inputCheckCounter++;
  }

  // Use softpot position to control tilt servo --------------------------------------------------
  if(softpotCheckCounter >= softpotCheckInterval) {
    if(softpotAverage > softpotMinPosition && softpotAverage < softpotMaxPosition)
      tiltPosition = map(softpotAverage, softpotMinPosition, softpotMaxPosition, tiltMinPosition, tiltMaxPosition);
    
    softpotCheckCounter = 0;
  } else {
    softpotCheckCounter++;
  }

  // Piezo 1 cooldown --------------------------------------------------------------
  if(cooldownCounter1 == 0) {
    // Piezo event triggered
    if(average1 > triggerThreshold) {
      // PIEZO 1 TRIGGER ******************************
      Serial.println("[1] triggered");
      
      panSavedPosition = panPosition;
      
      // shake left/right 3 times
      for(int i=0; i<3; i++) {
        // move left a bit
        for(int x=0; x<10; x++) {
          panServo.write( constrain(panPosition - x*2, panMinPosition, panMaxPosition) );
          delay(servoDelay);
        }
        
        // move back to center
        for(int x=10; x>0; x--) {
          panServo.write( constrain(panPosition - x*2, panMinPosition, panMaxPosition) );
          delay(servoDelay);
        }
        
        // move right a bit
        for(int x=0; x<10; x++) {
          panServo.write( constrain(panPosition + x*2, panMinPosition, panMaxPosition) );
          delay(servoDelay);
        }
        
        // move back to center
        for(int x=10; x>0; x--) {
          panServo.write( constrain(panPosition + x*2, panMinPosition, panMaxPosition) );
          delay(servoDelay);
        }
      }
      
      panPosition = panSavedPosition;
      panServo.write(panPosition);
      delay(servoDelay);
      
      // Reset cooldown counter 
      cooldownCounter1 = triggerCooldownTime;
    }
  } else {
    cooldownCounter1--;
  }
  
  // Piezo 2 cooldown --------------------------------------------------------------
  if(cooldownCounter2 == 0) {
    // Piezo event triggered
    if(average2 > triggerThreshold) {
      // PIEZO 2 TRIGGER ******************************
      Serial.println("[2] triggered");
      
      tiltSavedPosition = tiltPosition;
      
      // shake up/down 3 times
      for(int i=0; i<3; i++) {
        // move up a bit
        for(int y=0; y<10; y++) {
          tiltServo.write( constrain(tiltPosition - y*2, tiltMinPosition, tiltMaxPosition) );
          delay(servoDelay);
        }
        
        // move back to center
        for(int y=10; y>0; y--) {
          tiltServo.write( constrain(tiltPosition - y*2, tiltMinPosition, tiltMaxPosition) );
          delay(servoDelay);
        }
        
        // move down a bit
        for(int y=0; y<10; y++) {
          tiltServo.write( constrain(tiltPosition + y*2, tiltMinPosition, tiltMaxPosition) );
          delay(servoDelay);
        }
        
        // move back to center
        for(int y=10; y>0; y--) {
          tiltServo.write( constrain(tiltPosition + y*2, tiltMinPosition, tiltMaxPosition) );
          delay(servoDelay);
        }
      }
      
      tiltPosition = tiltSavedPosition;
      tiltServo.write(tiltPosition);
      delay(servoDelay);
      
      // Reset cooldown counter 
      cooldownCounter2 = triggerCooldownTime;
    }
  } else {
    cooldownCounter2--;
  }
  
  // Piezo 3 cooldown --------------------------------------------------------------
  if(cooldownCounter3 == 0) {
    // Piezo event triggered
    if(average3 > triggerThreshold) {
      // PIEZO 3 TRIGGER ******************************
      Serial.println("[3] triggered");
      
      panPosition += random(30,60) * (int)random(-1,1);
      tiltPosition += random(30,60) * (int)random(-1,1);
        
      panPosition = constrain(panPosition, panMinPosition, panMaxPosition);
      tiltPosition = constrain(tiltPosition, tiltMinPosition, tiltMaxPosition);
        
      panServo.write(panPosition);
      tiltServo.write(tiltPosition);   
  
      delay(1000);
      
      // Reset cooldown counter 
      cooldownCounter3 = triggerCooldownTime;
    }
  } else {
    cooldownCounter3--;
  }
  
  // Update the servos with non-blocking delay ------------------------------------------
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

  // Output useful information to serial ------------------------------------------------- 
  if(debug) {
//    // Piezo readings
//    Serial.print("1=");
//    Serial.print(average1);
//    Serial.print(", 2=");
//    Serial.print(average2);
//    Serial.print(", 3=");
//    Serial.print(average3);

    // Softpot reading
//    Serial.println(softpotAverage);
    
    // Button states
//    Serial.print("L=");
//    Serial.print(buttonLeftState);
//    Serial.print(", R=");
//    Serial.println(buttonRightState);

    Serial.println();
  }

}

/**********************************************************
 Obtain and smooth raw data from sensors, then calculate
 rolling average for processing
***********************************************************/
void readSensors() {
  // Remove previous buffer entries
  total1 -= buffer1[bufferIndex];
  total2 -= buffer2[bufferIndex];
  total3 -= buffer3[bufferIndex];
  softpotTotal -= softpotBuffer[bufferIndex];
  
  // Obtain new readings from sensors
  buffer1[bufferIndex] = analogRead(piezo1Pin);
  buffer2[bufferIndex] = analogRead(piezo2Pin);
  buffer3[bufferIndex] = analogRead(piezo3Pin);
  softpotBuffer[bufferIndex] = analogRead(softpotPin);
  
  // Append new readings to totals
  total1 += buffer1[bufferIndex];
  total2 += buffer2[bufferIndex];
  total3 += buffer3[bufferIndex];
  softpotTotal += softpotBuffer[bufferIndex];
  
  // Increment and wrap buffer index
  bufferIndex++;
  
  if(bufferIndex >= bufferSize)
    bufferIndex = 0;
  
  // Finding rolling average for each sensor  
  average1 = total1 / bufferSize;
  average2 = total2 / bufferSize;
  average3 = total3 / bufferSize;
  softpotAverage = softpotTotal / bufferSize;
}
