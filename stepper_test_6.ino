#include <Stepper.h>
 
// Gauge Defaults
#define STEPS 600     // Max number of steps for gauge type
#define DEGREES 270   // Max number of degrees in the gauge sweep

// Useful calculations
float stepsperdegree = float(STEPS)/float(DEGREES);

// Set up the Target value
int speedotarget = 0;

// Setup the input string
String inString = "";

// A variable to globally state the clusters mode (0=initialization, 1= runtime)
int clustermode = 0;

class Gauge
{
  // Values
  int sMaxSteps;     // Total number of steps that the stepper motor is capable of (used for initial resets)
  int sStepIncr;     // Number of step increments to take with each step (smoothness of the needle speed)
  int sMaxDegrees;   // Max value for the sweep in degrees, min presumed to be zero
  int sGaugeMin;     // Value associated with 0 degrees
  int sGaugeMax;     // Value associated with MaxDegrees
  int sSweepMode;    // Initial sweep status of the gauge
  int sRngSteps;     // Number of steps at the maximum

  // Stepper object
  Stepper* stepper;  // Tie into the Stepper library object

  // Current status
  int sPosDegrees;    // Current position of the gauge (in stepper degrees)
  int sTarget;     // Latest target request for the gauge (in gauge units)

  // Constructor - create a gauge
  public:
  Gauge(Stepper* stepperobj, int maxsteps, int stepincr, int maxdegrees, int gaugemin, int gaugemax, int stepspeed)
  {
    // Receive a stepper object
    stepper = stepperobj;

    // Set the gauge speed
    stepper->setSpeed(stepspeed);

    // Get all the settings for this gauge
    sMaxSteps = maxsteps;
    sStepIncr = stepincr;
    sMaxDegrees = maxdegrees;
    sGaugeMin = gaugemin;
    sGaugeMax = gaugemax;

    // Initialize
    sPosDegrees = 0;      // Current degree position of the gauge
    sTarget = gaugemin;   // Start the gauge (after initialization) at the minimum gauge value (=0 degrees)
    sSweepMode = 0;       // Initialize for first sweep
    sRngSteps = int(float(sMaxDegrees)*stepsperdegree);
  }

  // Set a new target value for the gauge
  void Update(int target)
  {
    sTarget = target;
  }

  // Make a step in the initialization sweep
  public:
  boolean Sweep()
  {
    boolean sweepdone = false;

    // Just run down the stepper STEPS times to make sure it is in the zero position
    if (sSweepMode == 0){
        sPosDegrees = STEPS;
        sSweepMode = 1;
    } else {
      if ((sPosDegrees - sStepIncr) >= 0) {
        sPosDegrees = sPosDegrees - sStepIncr;
        stepper->step(-sStepIncr);
      } else {
        stepper->step(-sPosDegrees);
        sPosDegrees = 0;    // Zero the position
        sweepdone = true;   // Signal that sweep is completed
      }
    }
    return sweepdone;
  }

  // Make a move towards the target value if necessary
  void Move()
  {
     // Target is in gauge units, so need to interpolate to get the stepper setting to aim for
     float percgauge = float(sTarget - sGaugeMin)/float(sGaugeMax - sGaugeMin);
     int dTarget = int(float(sRngSteps) * percgauge);

     // Range checks
     if (dTarget > sRngSteps){dTarget = sRngSteps;}

     // runtime mode
     if((sPosDegrees + sStepIncr) < dTarget){
        // Move up towards target
        sPosDegrees = sPosDegrees + sStepIncr;
        stepper->step(sStepIncr);
     }
     if((sPosDegrees - sStepIncr) > dTarget){
        // Move down towards target
        sPosDegrees = sPosDegrees - sStepIncr;
        stepper->step(-sStepIncr);
     }
  }

};  // Close Gauge class

// Create the speedometer gauge
Stepper stepper1(STEPS, 4, 5, 6, 7);
Gauge speedo( &stepper1, STEPS, 1, 180, 0, 100, 60);  // 90 degrees, 0-100 range

void setup()
{
  // Initialize the serial port
  Serial.begin(9600);
  Serial.println("Stepper test!");
  
  // Initialize the Speedo Target
  speedotarget = 0;
}

void loop()
{
  // Test if there are any values on the serial port
  if (Serial.available()>0){
    int inChar = Serial.read();
    if (isDigit(inChar)){
      inString += (char)inChar;
    }
    if (inChar == '\n'){
      Serial.println(inString);
      speedotarget = inString.toInt();
      inString = "";
    }
  }

  // Initialize the gauges by sweeping up then down through the max steps
  if (clustermode == 0)    // initialization
  {
      // Instruct the gauge to sweep
      boolean speedosweepdone = false;
      speedosweepdone = speedo.Sweep();

      // Decide whether to switch into runtime mode (all gauges done initial sweep)
      if (speedosweepdone){
        clustermode = 1;    // switch to runtime mode
      }
  }

  // Runtime so update the gauges with any new data and progress movement towards their target
  if (clustermode == 1)   // runtime
  {
      // Update the speedo gauge
      speedo.Update(speedotarget);

      // Get the gauges to Move
      speedo.Move();
  }   
}
