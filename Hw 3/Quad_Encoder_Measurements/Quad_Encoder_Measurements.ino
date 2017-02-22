// Pin Definitions
#define SENSOR_PIN 0
#define DIR_PIN    8 //2
#define PWM_PIN    9 //3
#define LED_PIN    13

// Encoder Definitions
#define ENC_A 2 //28
#define ENC_B 3 //19
#define CPR   48.0  // counts per revolution:

// Write Cutoff Time (ms)
#define CUTOFF_TIME 5000

// Delays
#define SERIAL_WRITE_DELAY 10

// Variable Declarations
long encodercount = 0;
unsigned long SerialWriteRuntime = 0;

int pwmSpeed = 255;
int motorDirection = 0;
int isStopped = 0;

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

  // Initialize interrupts for encoder readings
  attachInterrupt(digitalPinToInterrupt(ENC_A), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), handleEncoderB, CHANGE);

  // Start motor
  digitalWrite(DIR_PIN,motorDirection);
  analogWrite(PWM_PIN,pwmSpeed);
  
  // Start serial connection
  Serial.begin(250000);
  Serial.println("Time (ms), Encoder Ticks, Angle (rad)");
}

//////////
// Loop //
//////////
void loop() {
  // Take a lot of readings
  if(millis() >= SerialWriteRuntime && !isStopped)
  {
    SerialWriteRuntime = millis() + SERIAL_WRITE_DELAY;
    Serial.print(millis());
    Serial.print(",");
    Serial.print(encodercount);
    Serial.print(",");
    Serial.println(countsToRadians(encodercount));
    return;
  }
  
  // Only run/compile cutoff bit if the cutoff time is greater than 0
  #if (CUTOFF_TIME > 0)
  // Stop time
  if(millis() >= CUTOFF_TIME && isStopped == 0)
  {
    isStopped = 1;
    digitalWrite(PWM_PIN,0);
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
* float countsToRadians(long counts)
* 
* Convert encoder counts to radians
*******************************************************************************/
float countsToRadians(long counts)
{
  return float(counts)*2*PI/CPR;
}
