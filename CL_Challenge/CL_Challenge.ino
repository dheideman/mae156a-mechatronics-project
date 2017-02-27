/*
Closed Loop Challenge
By Daniel Heideman and Stuart Sonatina
MAE 156A - 2017-03-02
Details:
  1. Raise mass from bottom to 170-180 degrees.
  2. Swing mass around 360 to strike pendulum
  3. Follow-through and come to rest at 0 degrees (bottom) again

*/
//#include <DiscreteFilter.h>

// Pin Definitions
#define SENSOR_PIN  0 // What is this?
#define DIR_PIN     8 //2
#define PWM_PIN     9 //3
#define TA_PIN      4 // pin to connect to TA arduino

// Encoder Definitions
#define ENC_PIN_A   2 //18
#define ENC_PIN_B   3 //19
#define CPR      48.0  // counts per revolution:

// Gear Ratio and Stop Angle
#define GEAR_RATIO  4
#define THETA_STOP  315    // degrees
#define THETA_BUFF  45     // degrees

// TA Delay
#define TA_DELAY    5000

// Start Delay
#define START_DELAY 10

// Write Cutoff Time (ms)
#define CUTOFF_TIME 1000

// Delays
#define SERIAL_WRITE_DELAY 10

// Variable Declarations
long encodercount = 0;

int motorSpeed  = 100;   // Percent
int isStopped   = 0;
int stopWriting = 0;

// Runtime (timer) Variables
unsigned long SerialWriteRuntime = 0;

// Velocity struct
typedef struct velstruct_t
{
  float x[2]; // radians
  float t[2]; // seconds
  float v[2]; // rad/s
};
velstruct_t velstruct;

///////////
// Setup //
///////////
void setup() {
  // Initialize pins
  pinMode(SENSOR_PIN,INPUT);
  pinMode(ENC_PIN_A,INPUT);
  pinMode(ENC_PIN_B,INPUT);

  pinMode(DIR_PIN,OUTPUT);
  pinMode(PWM_PIN,OUTPUT);
  pinMode(TA_PIN, OUTPUT);

  // Initialize interrupts for encoder readings
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), handleEncoderB, CHANGE);

  // delay a little to keep velocity calculator from going infinite
  delay(START_DELAY);

  // Tell the TAs that we're ready...
  digitalWrite(TA_PIN,LOW);
  delay(TA_DELAY);
  digitalWrite(TA_PIN,HIGH);

  // Reset encoder value
  encodercount = 0;

  // Start motor
  setMotor(motorSpeed);

  // Start serial connection
  Serial.begin(250000);
}

//////////
// Loop //
//////////
void loop()
 {
  // Take a lot of readings
  if(millis() >= SerialWriteRuntime && !stopWriting)
  {
    SerialWriteRuntime = millis() + SERIAL_WRITE_DELAY;

    // Calculate velocity
    float t = float(micros())/1000000.0;
    float x = countsToRadians(encodercount);
    float v = calculateVelocity(&velstruct,t,x);

    // Write to serial port
    Serial.print(millis()-TA_DELAY);
    Serial.print(",");
    Serial.print(encodercount);
    Serial.print(",");
    Serial.println(velstruct.v[0]);
    return;
  }

  // Only run/compile cutoff bit if the stop angle is greater than 0
  #if (THETA_STOP > 0)
  // Stop time
  if(countsToDegrees(encodercount)/GEAR_RATIO >= THETA_STOP && !stopWriting)
  {
    stopWriting = 1;
    Serial.print("Time to Hit: ~");
    Serial.print(millis()-TA_DELAY-START_DELAY);
    Serial.print(" ms\n");
  }

  if(countsToDegrees(encodercount)/GEAR_RATIO >= (THETA_STOP + THETA_BUFF) && isStopped == 0)
  {
    stopMotor();
  }

  #endif

  // Only run/compile cutoff bit if the cutoff time is greater than 0
  #if (CUTOFF_TIME > 0)
  // Stop time
  if(millis() >= CUTOFF_TIME+TA_DELAY+START_DELAY && isStopped == 0)
  {
    stopMotor();
  }
  #endif
}

//////////////////////////////
// Custom defined functions //
//////////////////////////////

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

  encodercount += 2*(encA ^ encB) - 1;
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

  encodercount -= 2*(encA ^ encB) - 1;
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
* void setMotor()
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
* float calculateVelocity(velstruct_t* velstruct, float t, float x)
*
* Calculate the velocity for a velstruct_t struct
* It's best to use time in seconds
*******************************************************************************/
float calculateVelocity(velstruct_t* velstruct, float t, float x)
{
  // Update values
  velstruct->v[1] = velstruct->v[0];
  velstruct->x[1] = velstruct->x[0];
  velstruct->x[0] = x;
  velstruct->t[1] = velstruct->t[0];
  velstruct->t[0] = t;

  float dx = velstruct->x[0] - velstruct->x[1];
  float dt = velstruct->t[0] - velstruct->t[1];

  velstruct->v[0] = dx/dt;
  return velstruct->v[0];

}
