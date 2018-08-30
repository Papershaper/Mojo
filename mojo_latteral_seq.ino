/*************************************************** 
  Mojo Latteral Sequence Walk
  
  Using the PWM library from Adafruit for the PCA 9685
  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//need to convert these to the specifics of the motor being controlled.
// each side will have different values, so need SERVOMIN/SERVOMAX for each servo
#define SERVOMIN  650 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2350 // this is the 'maximum' pulse length count (out of 4096)
#define FREQUENCY 60 //  Analog servos run at ~60 Hz updates  (or 50?)

//INFO only now
//int LFleg = 0;
//int RFleg = 1;
//int LHleg = 2;
//int RHleg = 3;
//int LFknee = 4;
//int RFknee = 5;
//int LHknee = 6;
//int RHknee = 7;

int standPos[8]={120,60,60,120,90,90,90,90};
int startPos[8]={100,50,80,130,90,90,90,90};
int angle[8]={100,50,80,130,90,90,90,90};  //used to hold the incrementing angle data

//immutable increment value for each servo x segment
int seq[8][6]={
  {1,1,1,1,-2,-2},
  {-1,2,2,-1,-1,-1},
  {-2,-2,1,1,1,1},
  {-1,-1,-1,2,2,-1},
  {0,0,0,0,9,-9},
  {0,-9,9,0,0,0},
  {-9,9,0,0,0,0},
  {0,0,0,9,-9,0}
};


void setup() {
  Serial.begin(9600);
  Serial.println("Mojo Lateral Sequence - Start");
  pinMode(LED_BUILTIN, OUTPUT);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  delay(500);  //pause before starting
  stand();      //set everything to inital starting position, energize servos
  delay(500);  //pause before starting
}

int anglePulse(int angle) {
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

void stand() {
  Serial.println("Stand!");
  for(int x=0; x<=7; x++){
    pwm.setPWM(x, 0, anglePulse(standPos[x]));
  }
}
void start(){
  for(int x=0; x<=7; x++){
    pwm.setPWM(x, 0, anglePulse(startPos[x]));
  }
}

void lateral_walk(){
  // Lateral Sequence Walk
  Serial.println("Slow latreal Walk");
  int stepDelay = 10; //milliseconds
  int stepCount = 9;  // 10 steps per 6 segments
  //Starting positions (RFleg, LHleg and LFleg, RHleg)

  //cycle: segments 6 X 10 steps X 8 servos
  for(int seg=0; seg<=5; seg++){
    for(int qStep=0; qStep<=stepCount; qStep++){
      for(int x=0; x<=7; x++){
        angle[x]=angle[x]+seq[x][seg];
        pwm.setPWM(x, 0, anglePulse(angle[x]));
      } 
      delay(stepDelay);     
    }
  }
 
}
void loop() {
  delay(200);                       
  digitalWrite(LED_BUILTIN, LOW);
  //stand();
  //start();
  delay(10);
  lateral_walk();  //walk 
  //Brinlk
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
}



