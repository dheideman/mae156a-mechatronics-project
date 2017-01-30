// Pin Definitions
#define SENSOR_PIN 0
#define DIR_PIN    2
#define PWM_PIN    3
#define LED_PIN    13

// Encoder Definitions
#define ENC_A 18
#define ENC_B 19
#define CPR   48.0  // counts per revolution:

// Low Pass Filter Constants
#define LP_DELAY   100
#define LP_GAIN    1.0
#define LP_TAU     0.2

// Delays
#define SERIAL_WRITE_DELAY 10

// Struct typedefs

// 1st order filter struct
struct filter1_d
{
  // Basic stuff
  float dt;
  float gain;
  float saturation;
  unsigned long currentStep;
  int initialized;
  
  // Filter Values
  float num[2];
  float den[2];
  float inputs[2];
  float outputs[2];
};

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

filter1_d      velfilter;
filter1_d      accelfilter;
velocitycalc_d velcalc;
velocitycalc_d velcalc2;
velocitycalc_d accelcalc;

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
  createLowPassFilter(&velfilter,LP_TAU,LP_DELAY/1000.0,LP_GAIN);
  createLowPassFilter(&accelfilter,LP_TAU,LP_DELAY/1000.0,LP_GAIN);
  initializeVelocityCalc(&velcalc);
  initializeVelocityCalc(&velcalc2);
  initializeVelocityCalc(&accelcalc);

  // Initialize interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), handleEncoderB, CHANGE);

  // Start motor
  digitalWrite(DIR_PIN,motorDirection);
  analogWrite(PWM_PIN,pwmSpeed);
  
  // Start serial connection
  Serial.begin(250000);
  Serial.println("Time (ms), Angle (radians), Velocity (filtered) (rad/s), Velocity (unfiltered) (rad/s), Acceleration (rad/s/s)");
}

//////////
// Loop //
//////////
void loop() {
  // Take a lot of readings
  if(millis() >= SerialWriteRuntime)
  {
    SerialWriteRuntime = millis() + SERIAL_WRITE_DELAY;
    float theta = countsToRadians(encodercount);
    float velocity =  stepFilter1(&velfilter, calculateVelocity(&velcalc, theta, micros()));
    float velocity2 = calculateVelocity(&velcalc2, theta, micros());
    float accel =  stepFilter1(&accelfilter, calculateVelocity(&accelcalc, velocity, micros()));
    Serial.print(millis());
    Serial.print(",");
    Serial.print(theta);
    Serial.print(",");
    Serial.print(velocity);
    Serial.print(",");
    Serial.print(velocity2);
    Serial.print(",");
    Serial.println(accel);
    return;
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
* float stepFilter1(filter1_d* filter_ptr, float input)
* 
* Step through filter, return output
*******************************************************************************/
float stepFilter1(filter1_d* filter_ptr, float input)
{
  unsigned long currentMillis = millis();
  filter_ptr->currentStep++;
  
  // Update inputs/outputs
  filter_ptr->inputs[1] = filter_ptr->inputs[0];
  filter_ptr->inputs[0] = input;
  filter_ptr->outputs[1] = filter_ptr->outputs[0];
  
  // Calculate new output
  float output = 0.0;
  for (int i=0; i<2; i++)
  {
    output += filter_ptr->gain*filter_ptr->num[i]*filter_ptr->inputs[i];
  }
  output -= filter_ptr->den[1]*filter_ptr->outputs[1];
  filter_ptr->outputs[0] = output / filter_ptr->den[0];

  return filter_ptr->outputs[0];
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
* void createLowPassFilter(filter1_d* filter_ptr,float tau,float dt,float gain)
* 
* Create a 1st order low pass filter for time step dt, time constant tau.
*******************************************************************************/
void createLowPassFilter(filter1_d* filter_ptr,float tau,float dt,float gain)
{
  filter_ptr->dt = dt;
  filter_ptr->gain = gain;
  filter_ptr->saturation = -1;
  filter_ptr->currentStep = 0;
  
  filter_ptr->num[0] = dt/tau;
  filter_ptr->num[1] = 0.0;
  filter_ptr->den[0] = 1.0;
  filter_ptr->den[1] = dt/tau - 1.0;
  
  filter_ptr->inputs[0] = 0.0;
  filter_ptr->inputs[1] = 0.0;
  filter_ptr->outputs[0] = 0.0;
  filter_ptr->outputs[1] = 0.0;
  
  filter_ptr->initialized = 1;
}

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

