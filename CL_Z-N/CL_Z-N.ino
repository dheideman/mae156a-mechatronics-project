/*
Closed Loop Challenge
By Daniel Heideman and Stuart Sonatina
MAE 156A - 2017-03-02

Code for testing basic controller
*/

// Pin Definitions
#define POT_PIN     0
#define DIR_PIN     8 //
#define PWM_PIN     9 //
#define TA_PIN      4 // pin to connect to TA arduino
#define START_PIN  A5 // place momentary switch to ground

// Potentiometer Definitions
#define POT_MIN     0
#define POT_MAX     340 // max angle readable by pot (degrees)
#define POT_OFFSET  0   // offset of mass down position from 0 (degrees)

// Pot Buffer Zone
#define POT_BUFFER    5               // buffer zone size around dead zone (degrees)
#define POT_BUFF_POS  POT_MAX - POT_BUFFER    // Effective boundary of pot angles (+)
#define POT_BUFF_NEG  POT_BUFFER              // Effective boundary of pot angles (-)

// Encoder Definitions
#define ENC_PIN_A   2 //
#define ENC_PIN_B   3 //
#define CPR     201.6 // 48.0 * 4.2 counts per revolution

// Stop motor after a period of time (in case of mishaps)
#define CUTOFF_TIME 5000 // (ms)

// Delays and periods
#define SERIAL_WRITE_PERIOD 40   // ms
#define SAMPLE_PERIOD       10   // ms

// Variable Declarations
float setpoint = 0; // desired position
float error[2] = {0,0};    // position error
float de = 0;
float dt = 0;
float K_p = 600;     // proportional control const
float K_i = 0;      // integral control const
float K_d = 0;      // derivative control const

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
  float theta[2] = {0,0};     // radians
  float theta_dot[2] = {0,0}; // rad/s
  float t[2] = {0,0};         // seconds
  float u[2] = {0,0};         // output to motor
} S_t;

// Create state structure
S_t S;

///////////
// Setup //
///////////
void setup() {
  // Initialize pins
  pinMode(SENSOR_PIN,INPUT);
  pinMode(ENC_PIN_A,INPUT);
  pinMode(ENC_PIN_B,INPUT);
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(DIR_PIN,OUTPUT);
  pinMode(PWM_PIN,OUTPUT);
  pinMode(TA_PIN, OUTPUT);

  // Initialize interrupts for encoder readings
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), handleEncoderB, CHANGE);

  // Start serial connection
  Serial.begin(250000);

  S.t[0] = millis();

  // Wait for input from user
  Serial.println();
  Serial.println("Press start button");
  while (isStopped)
  {
    isStopped = digitalRead(START_PIN);
    delay(10);
  }

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
  setpoint = deg2rad(180);
  while(!isStopped)
  {
    // Sample position every SAMPLE_PERIOD
    if(millis() >= sampleRunTime)
    {
      sampleRunTime = millis() + SAMPLE_PERIOD;
      S.t[1]  = S.t[0];
      S.t[0]  = float(micros())/1000000.0;
      dt = S.t[0]-S.t[1];

      S.theta[1]      = S.theta[0];
      S.theta[0]      = countsToRadians(encoderCount);
      //S.theta_dot[0] = (S.theta[0] - S.theta[1])/S.t[0] - S.t[1];

      // Get difference of position and setpoint
      error[1] = error[0];
      error[0] = setpoint - S.theta[0];
      de = error[0]-error[1];

      // Controller
      S.u[0] = K_p*error[0];// + K_d*de/dt;
      S.u[1] = setMotor(S.u[0]);
    }

    // Print at defined intervals
    if(millis() >= serialWriteRunTime && !stopWriting)
    {
      serialWriteRunTime = millis() + SERIAL_WRITE_PERIOD;

      // Write to serial port
      Serial.print("\t");
      Serial.print(rad2deg(S.theta[0]));
      Serial.print(",\t");
      Serial.print(rad2deg(error[0]));
      Serial.print(",\t");
      Serial.print(S.u[0]);
      Serial.print(",\t");
      Serial.println(dt);
      return;
    }

    // Stop motor if time has gone too long
    if(millis()>=CUTOFF_TIME+beginTime)
    {
      Serial.println("CUTOFF_TIME reached");
      stopMotor();
    }
  }

  // Stop motor if isStopped is triggered.
  stopMotor();
  Serial.println("Motor Stopped");
  delay(5000);
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
* float readPotRadians(int pin)
*
* Read pot angle in radians
*******************************************************************************/
float readPotRadians(int pin)
{
  // Standard read
  float rawangle = map(constrain(analogRead(pin),0,1023),0,1023,POT_MIN,POT_MAX);
  return deg2rad( rawangle - POT_OFFSET );
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
