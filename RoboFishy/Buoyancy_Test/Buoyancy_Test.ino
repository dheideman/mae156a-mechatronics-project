#include <Stepper.h>

// Stepper Motor
#define STEPS_PER_REV   2048    // buoyancy motor steps per rev
#define BMOTOR_N_TEETH  10      // teeth, buoyancy motor pinion
#define BRACK_PITCH     0.004   // m/tooth
#define MAX_TRAVEL      0.043   // max travel, m

// Pressure Sensor
#define PSENSOR_PIN       A0
#define PSENSOR_M_PER_ADC 10    // ADC counts per meter
#define PSENSOR_M_OFFSET  0     // Offset to 0 m

// Delays and periods
#define SERIAL_READ_PERIOD  40   // ms

// Create stepper motor object for buoyancy motor
Stepper bMotor(STEPS_PER_REV, 8, 9, 10, 11);
long bmotorposition = 0;

// Runtime (timer) Variables
unsigned long serialReadRunTime = 0;

void setup()
{
  // set the speed at maximum for this motor:
  bMotor.setSpeed(15);

  // assume we're starting at max + position
  bmotorposition = metersToSteps(MAX_TRAVEL/2);
  
  // initialize the serial port:
  Serial.begin(250000);
}

void loop()
{
  // Read from serial port every SERIAL_READ_PERIOD
  if(millis() >= serialReadRunTime)
  {
    float dx;
    serialReadRunTime = millis() + SERIAL_READ_PERIOD;
    if(Serial.available() > 0)
    {
      switch(char(Serial.read()))
      {
        // Move motor to absolute position, mm
        case 'G':
        case 'g':
          dx = Serial.parseFloat();
          setBMotorPosition(dx/1000);
          Serial.print("Move motor to: ");
          Serial.println(dx);
          break;
        // Set current position to 0
        case 'Z':
        case 'z':
          Serial.parseFloat();
          bmotorposition = 0;
          Serial.println("Zeroing Buoyancy Motor");
          break;
        // Move by __ amount (mm)
        case 'M':
        case 'm':
          dx = Serial.parseFloat();
          setBMotorPosition(dx/1000+stepsToMeters(bmotorposition));
          Serial.print("Move motor by: ");
          Serial.println(dx);
          break;
        case 'X':
        case 'x':
          dx = Serial.parseFloat();
          bmotorposition = metersToSteps(MAX_TRAVEL/2);
          Serial.println("Set as upper max");
          break;
        default:
          //Serial.readBytes();
          Serial.print("Current Position: ");
          Serial.println(stepsToMeters(bmotorposition)*1000);
          break;
      }
    }
    
  }

}

/////////////////////////////
// Stepper Motor Functions //
/////////////////////////////

/*******************************************************************************
 * void setBMotorPosition(float x)
 * 
 * set buoyancy motor to position x in meters
 ******************************************************************************/
void setBMotorPosition(float x)
{
  // Make sure we aren't going beyond the ends
  if(x>MAX_TRAVEL/2) x = MAX_TRAVEL/2;
  if(x<-MAX_TRAVEL/2) x = -MAX_TRAVEL/2;
  
  // Calculate number of steps needed to move
  long dstep = metersToSteps(x) - bmotorposition;

  // Move stepper motor
  bMotor.step(-dstep);

  // Update position
  bmotorposition = bmotorposition + dstep;
}

/*******************************************************************************
 * float stepsToMeters(long steps)
 * 
 * convert step value to displacement in meters
 ******************************************************************************/
float stepsToMeters(long steps)
{
  return steps/float(STEPS_PER_REV)*BMOTOR_N_TEETH*BRACK_PITCH;
}

/*******************************************************************************
 * long metersToSteps(float meters)
 * 
 * convert displacement in meters to step value
 ******************************************************************************/
long metersToSteps(float meters)
{
  return long(meters/(BRACK_PITCH*BMOTOR_N_TEETH)*STEPS_PER_REV);
}

///////////////////////////////
// Pressure Sensor Functions //
///////////////////////////////

/*******************************************************************************
 * float readPressureSensorMeters()
 * 
 * read pressure sensor, report in meters
 ******************************************************************************/
float readPressureSensorMeters()
{
  return PSENSOR_M_PER_ADC*analogRead(PSENSOR_PIN) + PSENSOR_M_OFFSET;
}
