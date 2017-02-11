/*
 * Passover.ino
 * MAE 156A
 * Open Loop Challenge Pt 1
 * 
 * Read the absolute angle from a continuous pot
 */
#include <DiscreteFilter.h>

// Define pins
#define POT_PIN   0

// Define pot info
#define POT_MIN    0
#define POT_MAX    340
#define ANALOG_MIN 0
#define ANALOG_MAX 1023
#define THRESHOLD  10

// Define serial constants
#define BAUD_RATE 115200

// Filter time constants (s)
#define FILTER_TAU    0.5
#define FILTER_DT     0.02

// Declare Variables
long          rollovercount = 0;
float         lastangle = 0.0;
unsigned long lasttime = 0;
float         velocity = 0.0;
float         absoluteangle = 0.0;
int           indeadzone = 0;

// Declare filter object
DiscreteFilter lpfilter;  // Low-pass filter

///////////
// Setup //
///////////

void setup()
{
  // Create low-pass filter
  lpfilter.createFirstOrderLowPassFilter(FILTER_DT,FILTER_TAU);
  
  // Initialize serial connection
  Serial.begin(BAUD_RATE);

  // Delay a little to keep velocity from going to infinity.
  delay(10);
  lastangle = readPot(POT_PIN);
}

//////////
// Loop //
//////////

void loop()
{
  // Get current angle
  float potangle = readPot(POT_PIN);

  // Check whethere we're probably in the dead zone
  if ( !indeadzone )
  {
    // Check for overflow
    if ( (lastangle > POT_MAX - THRESHOLD) && (velocity > 0) )
    {
      if ( potangle < POT_MAX - THRESHOLD )
        indeadzone = 1;
    }
    // Check for underflow
    else if ( (lastangle < THRESHOLD) && (velocity < 0) )
    {
      if ( potangle > THRESHOLD )
        indeadzone = -1;
    }
    // Otherwise, carry on
    else
    {
      // Calculate velocity
      velocity = (potangle - lastangle)/( float(millis() - lasttime)/1000.0 );
      absoluteangle = potangle + rollovercount*360;
    }
    lasttime = millis();
    lastangle = potangle;
  }
  // If we're in the dead zone...
  else
  {
    // Positive direction
    if ( (indeadzone == 1) && (potangle > POT_MIN )
        && (potangle < POT_MIN + 2*THRESHOLD) )
    {
      indeadzone = 0;
      rollovercount++;
      lastangle = potangle;
    }
    // Negative direction
    if ( (indeadzone == -1) && (potangle < POT_MAX )
        && (potangle > POT_MAX - 2*THRESHOLD) )
    {
      indeadzone = 0;
      rollovercount--;
      lastangle = potangle;
    }
  }
  float filteredvelocity = lpfilter.step(velocity);
  Serial.print(potangle);
  Serial.print(",");
  Serial.print(absoluteangle);
  Serial.print(",");
  Serial.print(velocity);
  Serial.print(",");
  Serial.println(filteredvelocity);
  delay(1000*FILTER_DT);
}

//////////////////////////////
// Custom defined functions //
//////////////////////////////

/*******************************************************************************
* float readPot(int pin)
* 
* Reads pot, returns angle in degrees
*******************************************************************************/
float readPot(int pin)
{
  return map(analogRead(pin),ANALOG_MIN,ANALOG_MAX,POT_MIN,POT_MAX);
}

