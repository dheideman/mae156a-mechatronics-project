/*
Closed Loop Challenge
By Daniel Heideman and Stuart Sonatina
MAE 156A - 2017-03-02
Details:
  1. Raise mass from bottom to top (-175 degrees).
  2. Swing mass around +355 degrees to strike pendulum at top (+180 degrees).
  3. Follow-through and come to rest at bottom again (360 degrees)

*/
#include <DiscreteFilter.h>

// Pin Definitions
#define SENSOR_PIN  0 // What is this?
#define DIR_PIN     8 //2
#define PWM_PIN     9 //3
#define TA_PIN      4 // pin to connect to TA arduino
#define START_PIN  A5 // place momentary switch to ground

// Encoder Definitions
#define ENC_PIN_A   2 //18
#define ENC_PIN_B   3 //19
#define CPR      48.0 // counts per revolution:

// Gear Ratio and transition angles
#define GEAR_RATIO      4.0
#define THETA_TOP    -175.0   // degrees
#define THETA_IMPACT  180.0   // degrees
#define THETA_STOP    360.0   // degrees

// Stop motor after a period of time (in case of mishaps)
#define CUTOFF_TIME 5000 // (ms)

// Delays and periods
#define SERIAL_WRITE_PERIOD 100   // ms
#define TA_DELAY            5000  // ms Wait for TA arduino
#define SAMPLE_PERIOD       100   // ms

// Variable Declarations
float setpoint;   // desired position
float K_p = 0.5;  // proportional control const
float K_i = 0;    // integral control const
float K_d = 0;    // derivative control const

long encoderCount = 0;        // Current encoder position
unsigned long beginTime = 0;  // Time when loop() starts
int motorSpeed  = 0;          // Percent
int isStopped   = 1;
int stopWriting = 0;

// Runtime (timer) Variables
unsigned long serialWriteRunTime = 0;
unsigned long sampleRunTime = 0;

// Define state struct type
typedef struct S_t
{
  float theta[2];     // radians
  float theta_dot[2]; // rad/s
  float t[2];         // seconds
  float u[2];         // output to motor

} S_t;

// Create state structure
S_t S;

// Create controller
//createPIDController(K_p, K_i, K_d, SAMPLE_PERIOD)

///////////
// Setup //
///////////
void setup() {
  // Initialize pins
  pinMode(SENSOR_PIN,INPUT);
  pinMode(ENC_PIN_A,INPUT);
  pinMode(ENC_PIN_B,INPUT);
  pinMode(START_PIN, INPUT_PULLUP);
  //digitalWrite(START_PIN, INPUT_PULLUP);  // set pullup on analog pin 5
  pinMode(DIR_PIN,OUTPUT);
  pinMode(PWM_PIN,OUTPUT);
  pinMode(TA_PIN, OUTPUT);

  // Initialize interrupts for encoder readings
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), handleEncoderB, CHANGE);

  // Start serial connection
  Serial.begin(250000);

  // Wait for input from user
  Serial.println("Press start button");
  while (isStopped)
  {
    isStopped = digitalRead(START_PIN);
    delay(10);
  }

  // Tell the TAs that we're ready...
  digitalWrite(TA_PIN,LOW);
  delay(TA_DELAY);
  digitalWrite(TA_PIN,HIGH);

  // Reset encoder value
  encoderCount = 0;

  // Set zero time to use as offset
  beginTime = millis();
}

//////////
// Loop //
//////////
void loop()
{
  // Sample position every SAMPLE_PERIOD
  if(millis() >= sampleRunTime && !isStopped)
  {
    sampleRunTime = millis() + SAMPLE_PERIOD;
    S.t[0] = float(micros())/1000000.0;
    S.theta[0] = countsToRadians(encoderCount);
    S.theta_dot[0] = calculateVelocity(&S,S.t[0],S.theta[0]);
  }
  /*switch(stage) // Still need to define stage
  {
    case 1: // 1. Raise mass from bottom to top (-175 degrees).
    {
      setpoint = -175;
      break;
    }
    case 2: // 2. Swing mass around +355 degrees to strike pendulum at top (+180 degrees).
    {
      while(S.theta[0] <= deg2rad(180)) {setMotor(100);}
      break;
    }
    case 3: // 3. Follow-through and come to rest at bottom again (360 degrees)
    {
      setpoint = 0;
      break;
    }
  }
*/
  // Print at defined intervals
  if(millis() >= serialWriteRunTime && !stopWriting)
  {
    serialWriteRunTime = millis() + SERIAL_WRITE_PERIOD;

    // Write to serial port
    Serial.print(millis()-TA_DELAY-beginTime);
    Serial.print(",");
    Serial.print(encoderCount);
    Serial.print(",");
    Serial.println(S.theta_dot[0]);
    return;
  }

  // Stop time (time at 360 +/- 1.5 degrees and velocity <= 10 deg/s)
  if(S.theta[0] >= THETA_STOP - 1.5 &&
    S.theta[0] <= THETA_STOP + 1.5 &&
    S.theta_dot[0] <= deg2rad(10) &&
    !stopWriting)
  {
    stopWriting = 1;
    Serial.print("Time to Hit: ~");
    Serial.print(millis()-TA_DELAY-beginTime);
    Serial.print(" ms\n");
  }

  // Only run/compile cutoff bit if the cutoff time is greater than 0
  #if (CUTOFF_TIME > 0)
  // Stop motor if time has gone too long
  if(millis() >= CUTOFF_TIME+TA_DELAY+beginTime && isStopped == 0)
  {
    stopMotor();
  }
  #endif
}

//////////////////////////////
// Custom defined functions //
//////////////////////////////

/*******************************************************************************
* float rad2deg(float radians)
*
* Exactly what you think it does
*******************************************************************************/
float rad2deg(float radians)
{
  return radians * 180.0/PI;
}

/*******************************************************************************
* float deg2rad(float degrees)
*
* Exactly what you think it does
*******************************************************************************/
float deg2rad(float degrees)
{
  return degrees * PI/180.0;
}

/*******************************************************************************
* void handleEncoderA()
*
* Encoder interrupt handler
*******************************************************************************/
void handleEncoderA()
{
  // Standard read
  int encA = digitalRead(ENC_PIN_A);
  int encB = digitalRead(ENC_PIN_B);

  encoderCount += 2*(encA ^ encB) - 1;
}

/*******************************************************************************
* void handleEncoderB()
*
* Encoder interrupt handler
*******************************************************************************/
void handleEncoderB()
{
  // Standard read
  int encA = digitalRead(ENC_PIN_A);
  int encB = digitalRead(ENC_PIN_B);

  encoderCount -= 2*(encA ^ encB) - 1;
}

/*******************************************************************************
* float countsToRadians(long counts)
*
* Convert encoder counts to radians
*******************************************************************************/
float countsToRadians(long counts)
{
  return float(counts)*2*PI/CPR;
}

/*******************************************************************************
* float countsToDegrees(long counts)
*
* Convert encoder counts to degrees
*******************************************************************************/
float countsToDegrees(long counts)
{
  return float(counts)*360.0/CPR;
}

/*******************************************************************************
* void stopMotor()
*
* Stop the motor, set flag "isStopped"
*******************************************************************************/
void stopMotor()
{
  stopWriting = 1;
  isStopped = 1;
  digitalWrite(PWM_PIN,0);
}

/*******************************************************************************
* float setMotor()
*
* Set the motor to a speed and direction
* Has saturation protection
*******************************************************************************/
float setMotor(float motorSpeed)
{
  // Set motor direction
  if (motorSpeed>0){digitalWrite(DIR_PIN,LOW);}
  else{digitalWrite(DIR_PIN,HIGH);}

  // Saturation protection
  if (motorSpeed>100){motorSpeed=100.0;}
  if (motorSpeed<-100){motorSpeed=-100.0;}

  // convert 0-100% to 0-255 duty cycle
  int motorDuty = map(abs(motorSpeed),0,100,0,255);

  // send PWM to motor
  analogWrite(PWM_PIN,motorDuty);

  // Return the actual controller output with saturation protection
  return motorSpeed;
}

/*******************************************************************************
* float calculateVelocity(S_t* S, float t, float x)
*
* Calculate the velocity for a S_t struct
* It's best to use time in seconds
*******************************************************************************/
float calculateVelocity(S_t* S, float t, float theta)
{
  // Update values
  S->theta_dot[1] = S->theta_dot[0];
  S->theta[1]     = S->theta[0];
  S->theta[0]     = theta;
  S->t[1]         = S->t[0];
  S->t[0]         = t;

  float d_theta = S->theta[0] - S->theta[1];
  float dt      = S->t[0] - S->t[1];

  S->theta_dot[0] = d_theta/dt;
  return S->theta_dot[0];
}
