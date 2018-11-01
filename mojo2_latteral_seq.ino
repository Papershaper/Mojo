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
//{LF, RF, LH, RH, LFK, RFK, LHK,RHK}
// backup  60,90,100,110
// 40 degree Front 50-90/130-90 Hind 60-100/120-80

const int buttonPin = 2; // State change Pin
int state = 0; //stand=0 walk=1
int buttonState = 0;
int lastState = 0; //stateful :)
//int servoTest = 0;

int standPos[8]={90,90,90,90,90,90,90,90};
int startPos[8]={60,90,100,110,90,90,90,90};
int angle[8]={60,90,100,110,90,90,90,90};  //used to hold the incrementing angle data

//immutable increment value for each servo x segment
int seq[8][6]={
  {1,1,1,1,-2,-2},
  {-1,2,2,-1,-1,-1},
  {-2,-2,1,1,1,1},
  {-1,-1,-1,2,2,-1},
  {90,90,90,90,170,170},
  {90,10,10,90,90,90},
  {10,10,90,90,90,90},
  {90,90,90,170,170,90}
};


void setup() {
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);
  Serial.println("Mojo Lateral Sequence - Start");
  pinMode(LED_BUILTIN, OUTPUT);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  //stand();      //set everything to inital starting position, energize servos
  delay(1000);  //pause before starting
}

int anglePulse(int angle) {
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

void stand() {
  //Serial.println("Stand!");
  for(int x=0; x<=7; x++){
    pwm.setPWM(x, 0, anglePulse(standPos[x]));
  }
}
void start(){
  for(int x=0; x<=7; x++){
    pwm.setPWM(x, 0, anglePulse(startPos[x]));
  }
}
void sweep(int servo){
  for(int x=90; x<=120; x++){
    pwm.setPWM(servo, 0, anglePulse(x));
    delay(20);
  }
  for(int x=90; x>=60; x--){
    pwm.setPWM(servo, 0, anglePulse(x));
    delay(20);
  }
}
void knee_test(int servo){
  Serial.print("Servo Test: ");
  Serial.println(servo);
  int angle=90;
  int stepAngle=9;
  for(int x=0; x<=9; x++){
    pwm.setPWM(servo, 0, anglePulse(angle));
    angle=angle+stepAngle;
    delay(100);
    Serial.println(angle);
  }
  for(int x=0; x<=9; x++){
    pwm.setPWM(servo, 0, anglePulse(angle));
    angle=angle-stepAngle;
    delay(100);
    Serial.println(angle);
  }

}
void lateral_walk(int speedDelay){
  // Lateral Sequence Walk
  //Serial.println("Slow latreal Walk - sequence");
  //int stepDelay = 10; //milliseconds   <=STEP DELAY
  int stepCount = 9;  // 10 steps per 6 segments
  //Starting positions (RFleg, LHleg and LFleg, RHleg)

  //cycle: segments 6 X 10 steps X 8 servos
  for(int seg=0; seg<=5; seg++){
    for(int qStep=0; qStep<=stepCount; qStep++){
      for(int x=0; x<=7; x++){
        if(x<4) {
          angle[x]=angle[x]+seq[x][seg];
        }else{
          angle[x]=seq[x][seg];
        }
        pwm.setPWM(x, 0, anglePulse(angle[x]));
      } 
      delay(speedDelay);     
    }
  }
 
}
void loop() {
  buttonState = digitalRead(buttonPin);
  //check-change state
  if (buttonState != lastState) {
    //change state
    if (buttonState == HIGH){
      if(state == 0) {
        state = 1; //walk
        start(); //prep-positions
      } else {
        state = 0; //stand
      }
    }
    delay(50); //for contact bounce
  }
  lastState = buttonState;
  //MOVE COMMANDS
  if (state == 0) {                     
    digitalWrite(LED_BUILTIN, LOW);
    stand();
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    lateral_walk(15);  //walk  speed should be between 5-toofast and 10-15-normal? or 15
//    knee_test(servoTest++);
//    if (servoTest >=8 ) {
//      servoTest = 0;
//    }
  }
 
}



