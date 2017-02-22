PWM_Pin#include <DiscreteFilter.h>

// Pin Definitions
#define SENSOR_PIN 0
#define DIR_PIN    8 //2
#define PWM_PIN    9 //3

// Encoder Definitions
#define ENC_PIN_A 2 //28
#define ENC_PIN_B 3 //19
#define CPR   48.0  // counts per revolution:

// Velocity Low-Pass Filter Constants
#define FILTER_DT   0.005
#define FILTER_TAU  0.05

// Write Cutoff Time (ms)
#define CUTOFF_TIME 1000

// Velocity Drop Threshold
#define VELDROP_THRESHOLD   0.99
#define VELCOUNT_THRESHOLD  3

// Delays
#define SERIAL_WRITE_DELAY 10

// Variable Declarations
long encodercount = 0;

int motorSpeed = 100;   // Percent
int veldropcounter = 0;
int isStopped = 0;

// Runtime (timer) Variables
unsigned long SerialWriteRuntime = 0;
unsigned long FilterRuntime = 0;

// Velocity struct
typedef struct velstruct_t
{
  float x[2];
  float t[2];
  float v[2];
};
velstruct_t velstruct;

// Filter Declaration
DiscreteFilter velfilter;

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

  // Initialize interrupts for encoder readings
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), handleEncoderB, CHANGE);

  // Initialize filter
  velfilter.createFirstOrderLowPassFilter(FILTER_DT,FILTER_TAU);

  // delay a little to keep velocity calculator from going infinite
  delay(10);

  // Start motor
  setMotor(motorSpeed);

  // Start serial connection
  Serial.begin(250000);
  //Serial.println("Time (ms), Encoder Ticks, Angle (deg)");
}

//////////
// Loop //
//////////
void loop()
 {
  // フィルタ第一! (Filter First!)
  // Run the lp filter on the velocity, compare to velocity.  Stop if needed.
  if(millis() >= FilterRuntime && !isStopped)
  {
    FilterRuntime = millis() + FILTER_DT*1000;
    float t = float(micros())/1000000.0;
    float x = countsToRadians(encodercount);
    float v = calculateVelocity(&velstruct,t,x);
    velfilter.step(v);

    // Check if velocity has fallen below threshold
    if ( v < velfilter.getLastOutput()*VELDROP_THRESHOLD )
    //if ( velstruct.v[0] < velstruct.v[1]*VELDROP_THRESHOLD )
    {
      // Increment counter
      veldropcounter++;
      Serial.println(veldropcounter);

      // Check if we've seen enough consecutively low values to stop
      if ( veldropcounter >= VELCOUNT_THRESHOLD )
      {
        Serial.println("Velocity Drop Detected");
        stopMotor();
      }
    }
    else
    {
      // Clear counter if we're above the filtered value again.
      veldropcounter = 0;
    }

    return;
  }

  // Take a lot of readings
  if(millis() >= SerialWriteRuntime && !isStopped)
  {
    SerialWriteRuntime = millis() + SERIAL_WRITE_DELAY;
    Serial.print(millis());
    Serial.print(",");
    Serial.print(velstruct.v[0]);
    Serial.print(",");
    //Serial.println(velstruct.v[1]);
    Serial.println(velfilter.getLastOutput());
    return;
  }

  // Only run/compile cutoff bit if the cutoff time is greater than 0
  #if (CUTOFF_TIME > 0)
  // Stop time
  if(millis() >= CUTOFF_TIME && isStopped == 0)
  {
    stopMotor();
    return;
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
  return float(counts)*180.0/CPR;
}

/*******************************************************************************
* void stopMotor()
*
* Stop the motor, set flag "isStopped"
*******************************************************************************/
void stopMotor()
{
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
