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
#define DEG_PER_CNT -0.3636
#define POT_OFFSET  144

//// Pot Buffer Zone
//#define POT_BUFFER    5               // buffer zone size around dead zone (degrees)
//#define POT_BUFF_POS  POT_MAX - POT_BUFFER    // Effective boundary of pot angles (+)
//#define POT_BUFF_NEG  POT_BUFFER              // Effective boundary of pot angles (-)

// Encoder Definitions
#define ENC_PIN_A   2 //
#define ENC_PIN_B   3 //
#define CPR     201.6 // 48.0 * 4.2 counts per revolution

// Stop motor after a period of time (in case of mishaps)
#define CUTOFF_TIME 5000 // (ms)

// Delays and periods
#define SERIAL_WRITE_PERIOD 10   // ms
#define SAMPLE_PERIOD       10   // ms

// Variable Declarations
float K_p = 300;        // proportional control const
float K_i = 1800;       // integral control const
float K_d = 20;//10.58;      // derivative control const

long encoderCount = 0;        // Current encoder position
unsigned long beginTime = 0;  // Time when loop() starts
float maxangle = 0;
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

// Define PID controller struct type
typedef struct PID_t
{
  // Constants
  float kp = 0;
  float ki = 0;
  float kd = 0;

  // Setpoint/Target
  float setpoint = 0;         // radians

  // Saturation value
  float errorsat = 0;

  // Error storage
  float error[2] = {0,0};     // radians
  float errorsum = 0;
  float t[2] = {0,0};         // seconds
} PID_t;

// Create state structure
S_t S;

// Create PID controller structure
PID_t PID;

///////////
// Setup //
///////////
void setup() {
  // Initialize pins
  pinMode(POT_PIN,INPUT);
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
  //Serial.println(rad2deg(readPotRadians(POT_PIN)));

  S.t[0] = millis();

  // Wait for input from user
  Serial.println();
  Serial.println("Press start button");
  while (isStopped)
  {
    isStopped = digitalRead(START_PIN);
    //Serial.println(analogRead(POT_PIN));
    //Serial.println(rad2deg(readPotRadians(POT_PIN)));  // measure pot angle count
    delay(10);
  }

  // Initialize PID controller
  PID.kp = K_p;
  PID.ki = K_i;
  PID.kd = K_d;
  PID.errorsat = 100/PID.ki;

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
  PID.setpoint = deg2rad(-172);
  while(!isStopped)
  {
    // Sample position every SAMPLE_PERIOD
    if(millis() >= sampleRunTime)
    {
      sampleRunTime = millis() + SAMPLE_PERIOD;
      S.t[1]  = S.t[0];
      S.t[0]  = float(micros())/1000000.0;

      PID.t[1]  = PID.t[0];
      PID.t[0]  = float(micros())/1000000.0;
      float dt = S.t[0]-S.t[1];

      // Save old angle value, read in new one.
      S.theta[1]      = S.theta[0];
      S.theta[0]      = readPotRadians(POT_PIN);
      //S.theta_dot[0] = (S.theta[0] - S.theta[1])/S.t[0] - S.t[1];

      // Check if this was the maximum angle
      if( S.theta[0] < maxangle ) maxangle = S.theta[0];
      
      // Get difference of position and setpoint
      PID.error[1] = PID.error[0];
      PID.error[0] = PID.setpoint - S.theta[0];
      float de = PID.error[0]-PID.error[1];

      // Calculate integral gain
      if (abs(PID.kp * PID.error[0]) >= 150)
        PID.errorsum = 0;
      else if ( PID.errorsat > 0 )
        PID.errorsum = constrain(PID.errorsum + PID.error[0] * dt,
                                -1*PID.errorsat,PID.errorsat);
      else
        PID.errorsum += PID.error[0];
      

      // Controller
      S.u[0] = PID.kp*PID.error[0] + PID.ki*PID.errorsum + PID.kd*de/dt;
      S.u[1] = setMotor(S.u[0]);
    }

    // Print at defined intervals
    if(millis() >= serialWriteRunTime && !stopWriting)
    {
      serialWriteRunTime = millis() + SERIAL_WRITE_PERIOD;

      // Write to serial port
      Serial.print(millis());
      Serial.print(",");
//      Serial.print("\t");
      Serial.print(rad2deg(S.theta[0]));
//      Serial.print(",\t");
//      Serial.print(rad2deg(PID.error[0]));
//      Serial.print(",\t");
//      Serial.print(S.u[0]);
//      Serial.print(",\t");
//      //Serial.println(dt);
//      Serial.println(S.t[0]-S.t[1]);
      Serial.print("\n");
      return;
    }

    // Stop motor if time has gone too long
    if(millis()>=CUTOFF_TIME+beginTime)
    {
      Serial.println("CUTOFF_TIME reached");
      Serial.print("Max angle attained: ");
      Serial.println(rad2deg(maxangle));
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
  return deg2rad(DEG_PER_CNT*(analogRead(pin) - POT_OFFSET));
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
