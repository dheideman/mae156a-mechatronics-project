// Pin Definitions
#define SENSOR_PIN 0
#define DIR_PIN    2
#define PWM_PIN    3
#define LED_PIN    13

// Encoder Definitions
#define ENC_A 18
#define ENC_B 19
#define CPR   48.0  // counts per revolution:

// Write Cutoff Time (ms)
#define CUTOFF_TIME 500

// Delays
#define SERIAL_WRITE_DELAY 10

// Struct typedefs

// Velocity calculation struct
struct velocitycalc_d
{
  unsigned long times[2];
  float positions[2];
  float velocity;
};

// Variable Declarations
long encodercount = 0;
unsigned long SerialWriteRuntime = 0;

int pwmSpeed = 255;
int motorDirection = 0;
int isStopped = 0;

velocitycalc_d velcalc;

///////////
// Setup //
///////////
void setup() {
  // Initialize pins
  pinMode(SENSOR_PIN,INPUT);
  pinMode(ENC_A,INPUT);
  pinMode(ENC_B,INPUT);

  pinMode(DIR_PIN,OUTPUT);
  pinMode(PWM_PIN,OUTPUT);
  pinMode(LED_PIN,OUTPUT);
  
  // Initialize the structs and shit
  initializeVelocityCalc(&velcalc);

  // Initialize interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), handleEncoderB, CHANGE);

  // Start motor
  digitalWrite(DIR_PIN,motorDirection);
  analogWrite(PWM_PIN,pwmSpeed);
  
  // Start serial connection
  Serial.begin(250000);
  Serial.println("Time (s), Velocity (rad/s)");
}

//////////
// Loop //
//////////
void loop() {
  // Take a lot of readings
  if(millis() >= SerialWriteRuntime && isStopped == 0)
  {
    SerialWriteRuntime = millis() + SERIAL_WRITE_DELAY;
    float theta = countsToRadians(encodercount);
    float velocity = calculateVelocity(&velcalc, theta, micros());
    Serial.print(micros()/1000000.0,4);
    Serial.print(",");
    Serial.println(velocity);
    return;
  }
  
  // Stop time
  if(millis() >= CUTOFF_TIME && isStopped == 0)
  {
    isStopped = 1;
    digitalWrite(PWM_PIN,0);
  }
  
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
  int encA = digitalRead(ENC_A);
  int encB = digitalRead(ENC_B);
  
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
  int encA = digitalRead(ENC_A);
  int encB = digitalRead(ENC_B);

  encodercount -= 2*(encA ^ encB) - 1;
}

/*******************************************************************************
* float calculateVelocity(velocitycalc_d* velcalc_ptr, float pos, unsigned long
* newtime)
* 
* Calculate velocity of motor from encoder values.  Give time in microseconds.
*******************************************************************************/
float calculateVelocity(velocitycalc_d* velcalc_ptr, float pos, unsigned long newtime)
{
  velcalc_ptr->positions[1] = velcalc_ptr->positions[0];
  velcalc_ptr->positions[0] = pos;
  velcalc_ptr->times[1] = velcalc_ptr->times[0];
  velcalc_ptr->times[0] = newtime;
  
  float da = velcalc_ptr->positions[0]-velcalc_ptr->positions[1];
  float dt = (velcalc_ptr->times[0]-velcalc_ptr->times[1])/1000000.0;
  
  velcalc_ptr->velocity = da/dt;
  
  return velcalc_ptr->velocity;
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

/////////////////////////////////////
// Struct initialization functions //
/////////////////////////////////////

/*******************************************************************************
* void initializeVelocityCalc(velocitycalc_d* velcalc_ptr)
* 
* Set all values of a velocity calculation struct to 0
*******************************************************************************/
void initializeVelocityCalc(velocitycalc_d* velcalc_ptr)
{
  for(int i=0; i<2; i++)
  {
    velcalc_ptr->times[i] = 0;
    velcalc_ptr->positions[i] = 0;
  }
  velcalc_ptr->velocity = 0.0;
}

