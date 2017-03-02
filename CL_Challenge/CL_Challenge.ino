/*
Closed Loop Challenge
By Daniel Heideman and Stuart Sonatina
MAE 156A - 2017-03-02
Details:
  1. Raise mass from bottom to top (-175 degrees).
  2. Swing mass around +355 degrees to strike pendulum at top (+180 degrees).
  3. Follow-through and come to rest at bottom again (360 degrees)

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

// Stop motor after a period of time (in case of mishaps)
#define CUTOFF_TIME 10000 // (ms)

// Delays and periods
#define SERIAL_WRITE_PERIOD 100   // ms
#define TA_DELAY            2000  // ms Wait for TA arduino
#define CONTROLLER_PERIOD   10    // ms
#define POSITION_HOLD_TIME  1000  // ms

// PID Constants: Lifting
#define KP_LIFT   300
#define KI_LIFT   1800
#define KD_LIFT   20

// PID Constants: Stopping
#define KP_STOP   300
#define KI_STOP   1800
#define KD_STOP   20

// Variable Declarations

// Transition angles
float thetaTop;    // degrees
float thetaImpact; // degrees
float thetaStop;    // degrees

// Angle hold time;
unsigned long timeatpos = 0;
int posholdcount = 0;

long encoderCount = 0;        // Current encoder position
unsigned long beginTime = 0;  // Time when loop() starts
int motorSpeed  = 0;          // Percent
int stopWriting = 0;

// Runtime (timer) Variables
unsigned long serialWriteRunTime = 0;
unsigned long controllerRunTime = 0;

// Define state struct type
typedef struct S_t
{
  float theta[2];     // radians
  float theta_dot[2]; // rad/s
  float t[2];         // seconds
  float u[2];         // output to motor
  int   state;        // state of wheel
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

  // Enabled?
  int enabled = 0;
} PID_t;

// Create state structure
S_t S;

// Create PID controller structure
PID_t PID;

///////////
// Setup //
///////////
void setup() {
  // Transition angles
  thetaTop    = deg2rad(-172);  // degrees
  thetaImpact = deg2rad(-180); // degrees
  thetaStop   = deg2rad(0); // degrees

  // Initialize pins
  pinMode(POT_PIN,INPUT);
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(DIR_PIN,OUTPUT);
  pinMode(PWM_PIN,OUTPUT);
  pinMode(TA_PIN, OUTPUT);

  // Start serial connection
  Serial.begin(250000);

  // Set state to 0 to begin program
  S.state = 0;
}

//////////
// Loop //
//////////
void loop()
{
  // Depending on state, perform action
  switch(S.state)
  {
    case 0: // 0. Wait for button press.
    {
      // Make sure motor is stopped
      stopMotor();
      
      // Wait for input from user
      Serial.println("Press start button");
      while (digitalRead(START_PIN)) delay(10);
    
//        Serial.println("Tell the TAs that we're ready...");
//        digitalWrite(TA_PIN,LOW);
//        delay(TA_DELAY);
//        digitalWrite(TA_PIN,HIGH);
    
      // First state: get to -175 degrees
      S.state = 1;
      Serial.println("Moving to State 1");
    
      // Reset encoder value
      encoderCount = 0;
      
      // Set zero time to use as offset
      beginTime = millis();
      
      break;
    }
    case 1: // 1. Initialize controller for lift.
    {
      // Initialize PID controller for lift
      PID.kp = KP_LIFT;
      PID.ki = KI_LIFT;
      PID.kd = KD_LIFT;
      PID.errorsat = 100/PID.kp;
      PID.enabled = 1;
      
      PID.setpoint = thetaTop;

      // Initialization is done, so immediately go to state 2
      S.state = 2;
      Serial.println("Moving to State 2");
      break;
    }
    case 2: // 2. Raise mass from bottom to top (-172 degrees).
    {
      if(abs(PID.error[0]) <= deg2rad(2))
      {
        // You've reached the top!
        timeatpos = millis();
        Serial.println("thetaTop reached");
        Serial.println("Moving to State 3");
        S.state = 3;
      }
      break;
    }
    case 3: // 3. Wait at top for prescribed amount of time
    {
      // Make sure we're still at the top
      if(abs(PID.error[0]) > deg2rad(2)) 
      {
        // if not, go back to state 2
        S.state = 2;
        Serial.println("Moving back to State 2");
      }
      // If we've been here long enough, then we can proceed.
      else if( millis() - timeatpos > POSITION_HOLD_TIME )
      {
        S.state = 4;
        Serial.println("Moving to State 4");
      }
      break;
    }
    case 4: // 4. Start to swing mass around +355 degrees to strike pendulum at top.
    {
      // Run free for a while.  100% ALL THE WAY!  But make sure PID is disabled first
      PID.enabled = 0;
      setMotor(100);

      // Wait a little to let the wheel go past the strike zone
      delay(100);

      // Prep work done - continue to next state
      S.state = 5;
      Serial.println("Moving to State 5");
      break;
    }
    case 5: // 5. Wait for mass to strike pendulum at top (-180 degrees).
    {
      if(S.theta[0] <= thetaImpact)
      {
        Serial.println("Impact");
        S.state = 6;
        Serial.println("Moving to State 6");
      }
      break;
    }
    case 6: // 6. Initialize controller for follow-through
    {
      // Initialize PID controller for lift
      PID.kp = KP_LIFT;
      PID.ki = KI_LIFT;
      PID.kd = KD_LIFT;
      PID.errorsat = 100/PID.kp;
      PID.enabled = 1;
      
      PID.setpoint = thetaStop;
      
      // Initialization is done, so immediately go to state 7
      S.state = 7;
      Serial.println("Moving to State 7");
      break;
    }
    case 7: // 7. Wait for controller to reach bottom (0 degrees)
    {
      if(abs(PID.error[0]) <= deg2rad(1.5))
      {
        // You've reached the end!
        Serial.println("thetaStop reached");
        timeatpos = millis();
        S.state = 8;
        Serial.println("Moving to State 8");
      }
      break;
    }
    case 8: // 8. Wait for controller to stabilize at bottom (0 degrees)
    {
      // Make sure we're hanging out at the bottom
      if(abs(PID.error[0]) > deg2rad(1.5)) 
      {
        // if not, go back to state 7
        S.state = 7;
        Serial.println("Moving back to State 7");
      }
      // If we've been here long enough, then we can proceed.
      else if( millis() - timeatpos > POSITION_HOLD_TIME )
      {
        // You've reached the end!
        Serial.println("Stopping Motor");
        stopMotor();
        S.state = 0;
        Serial.print("Execution time: ");
        Serial.print(timeatpos - beginTime);
        Serial.println(" ms");
        Serial.println("Moving to State 0");
      }
      break;
    }
    default:
    {
      S.state = 0;
      Serial.println("Moving to State 0");
    }
  }

  // Run controller every CONTROLLER_PERIOD
  if(millis() >= controllerRunTime)
  {
    // Set time to run controller the next time
    controllerRunTime = millis() + CONTROLLER_PERIOD;

    // Start running controller
    PID.t[1]  = PID.t[0];
    PID.t[0]  = float(micros())/1000000.0;
    float dt = PID.t[0]-PID.t[1];

    // Save old angle value, read in new one.
    S.theta[1]      = S.theta[0];
    S.theta[0]      = readPotRadians(POT_PIN);
    
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
      
    // Only run if enabled
    if(PID.enabled)
    {
      // Controller
      S.u[0] = PID.kp*PID.error[0] + PID.ki*PID.errorsum + PID.kd*de/dt;
      S.u[1] = setMotor(S.u[0]);
    }
  }

  // Print at defined intervals
  if(millis() >= serialWriteRunTime && !stopWriting)
  {
    serialWriteRunTime = millis() + SERIAL_WRITE_PERIOD;

    // Write to serial port
    Serial.print("\t");
    Serial.print(rad2deg(S.theta[0]));
    Serial.print(",\t");
    Serial.print(rad2deg(PID.error[0]));
    Serial.print(",\t");
    Serial.println(S.u[0]);
    return;
  }

//  // Stop time (time at thetaStop +/- 1.5 degrees and velocity <= 10 deg/s)
//  if(S.theta[0] >= thetaStop - 1.5 &&
//    S.theta[0] <= thetaStop + 1.5 &&
//    S.theta_dot[0] <= deg2rad(10))
//  {
//    Serial.print("Theta reached. Total Time: ~");
//    Serial.print(millis()-beginTime);
//    Serial.print(" ms\n");
//  }

  // Stop motor if time has gone too long
  if(millis()>=CUTOFF_TIME+beginTime)
  {
    Serial.println("CUTOFF_TIME reached");
    stopMotor();
    S.state = 0;
    Serial.println("Moving to State 0");
  }
  
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
* void stopMotor()
*
* Stop the motor, set flag "isStopped"
*******************************************************************************/
void stopMotor()
{
  stopWriting = 1;
  setMotor(0);
  PID.enabled = 0;
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
