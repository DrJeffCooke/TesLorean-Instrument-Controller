// EV Dash TesLorean
// Jeff Cooke Feb 2018

// TO-DOs
// Remove the Delay() from the loop section, recode to schedule time delays between events

// **** FUNCTIONALITY **** //
// CAN-BUS Shield
// Stepper Motor driver
// LED blinking light controls
// LCD Controller
// Real-time Clock

// INCLUDES ---------------------------------------------------------------------------------

// CAN-BUS Shield
#include <mcp2515.h>
#include <SPI.h>

// Stepper Motor driver
#include <Stepper.h>

// LED blinking light controls
#include <LinkedList.h>

// OLED Controller
#include <String.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <LinkedList.h>

// Real-time Clock
#include <Wire.h>
#include <DS3231.h>

// DEFINITIONS  ---------------------------------------------------------------------------------

#define DEBUG 1

// Stepper Motor driver
#define STEPS 600     // Max number of steps for gauge type
#define DEGREES 270   // Max number of degrees in the gauge sweep
float stepsperdegree = float(STEPS)/float(DEGREES);  // Useful calculations

// Steppper Motor WIRING
#define G1_1 4      // LARGE RIGHT - Speedo
#define G1_2 5
#define G1_3 6
#define G1_4 7
#define G2_1 4      // LARGE LEFT - Torque
#define G2_2 5
#define G2_3 6
#define G2_4 7
#define G3_1 4      // SMALL TOP LEFT - Low Voltage
#define G3_2 5
#define G3_3 6
#define G3_4 7
#define G4_1 4      // SMALL BOT LEFT - Temperature
#define G4_2 5
#define G4_3 6
#define G4_4 7
#define G5_1 4      // SMALL TOP RIGHT - Power
#define G5_2 5
#define G5_3 6
#define G5_4 7
#define G6_1 4      // SMALL BOT RIGHT - Traction Battery
#define G6_2 5
#define G6_3 6
#define G6_4 7

// Gauge display limits
#define HVREGENPOWER 10       // kW of regen power (power going back into system)
#define HVDISCHPOWER 10       // kW of power being discharged (use to drive the car)

// Battery HV Limits
#define BATTHIGHVOLTS 410       // recheck these max and mins for the A123 battery types
#define BATTLOWVOLTS  325

// Power gauge limits
#define MAXHVREGENPOWER 10      // check these against actual performance
#define MINHVDISCHPOWER -10

// LED blinking light controls
#define BEAT_ON_MILLIS 250    // Duration of an on beat
#define BEAT_OFF_MILLIS 125    // Duration of an off beat
#define BEAT_BTW_MILLIS 500    // Duration of an between beat
#define BEAT_GAP_MILLIS 1500    // Duration between codes

// LED warning light WIRING
#define WARNING_LIGHT_1 13          // Assign pins for each warning light
#define WARNING_LIGHT_2 14          // 
#define WARNING_LIGHT_3 15          // 
#define WARNING_LIGHT_4 16          // 
#define WARNING_LIGHT_5 17          // 

// OLED settings
#define I2C_ADDR_LEFT    0x3C   // Display under Range dial
#define I2C_ADDR_RGHT    0x3D   // Display under MPH dial
#define RST_PIN -1
#define DISPLAY_CHARS 16  // Physical characteristics of the display
#define DISPLAY_LINES 2
#define DISPLAY_COUNT 2
#define ARRAY_CHARS (DISPLAY_CHARS + 1)  // Size of arrays that contain 16 chars and a terminating \0 character

// CLASSES  ---------------------------------------------------------------------------------

// Stepper Motor driver
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

// LED blinking light controls
class LEDControl
{
  private:
  // Values
  unsigned long lastEventTime;    // millis reading when last on/off event
  bool onStatus;                  // true if LED is on
  char ledPattern[7];             // 6 numeric code and \0 character
  int posPattern;                 // 0-6, current place in the pattern
  int posCycle;                   // Current cycle in the pattern
  int controlType;                // 0 = off, 1 = pattern, 2 = continuous
  int lcPin;                      // Arduino pin that the LED light is controlled from
  int posLength;                  // Num chars in the pattern 0-6
  
  public:
  // Constructor
  LEDControl(int iPin)
  {
    // Set the pin
    lcPin = iPin;
    
    // Set the millis timer
    lastEventTime = millis();

    // Initial statuses
    controlType = 0;
    onStatus = false;
    posPattern = 0;
    posCycle = 0;
    posLength = 0;
  }

  // Initialize the LED (needs to be called in the setup() routine
  void InitLED()
  {
    // initialize the pin associated with the LED
    pinMode(lcPin, OUTPUT);
  }

  // Turns on the LED continuously
  void TurnOn()
  {
    // Set the status on and the control type
    onStatus = true;
    controlType = 2;

    // Turn the LED on
    digitalWrite(lcPin, HIGH);  
  }

  // Turn off the LED (also quits a pattern)
  void TurnOff()
  {
    // Set the Status off and clear the control type
    onStatus = false;
    controlType = 0;

    // Turn the LED off
    digitalWrite(lcPin, LOW);  
  }

  // Set the Blink pattern of the LED, and set the cycles going
  void SetPattern(char sPattern[])
  {
    // Transfer the pattern into the Class
    posLength = strlen(sPattern);

    // Check length for suitability
    if (posLength > 6){posLength = 6;}

    // Transfer pattern into the class char arracy
    for(int i=0; i<posLength; i++){ledPattern[i]=sPattern[i];}
    ledPattern[posLength] = '\0';

    // TODO : Confirm that the pattern is composed of digits 0-9

    // Start pattern settings
    controlType = 1;
    lastEventTime = millis();
    posPattern = 0;

    // Get the character at the posPosition in the pattern
    char curPattern;
    curPattern = ledPattern[posPattern];
    if (isDigit(curPattern))  // Get the ASCII of the text digit, then -48 ('0'=48 in ASCII)
    {
      posCycle = ((curPattern+0)-48);
      if (posCycle>0) {posCycle--;}
    }
    else {posCycle = 1;}

    // Turn on the LED to start with
    onStatus = true;
    digitalWrite(lcPin, HIGH);

  }

  // Check of the LEDcontrol has hit a transition point (or gone past it)
  void CheckStatus()
  {
    if (controlType == 1)
    {
      // Get the current millis time
      unsigned long curmillis;
      curmillis = millis();

      // Time gap to look for depends on current status
      unsigned long BEAT_MILLIS;
      if (!onStatus && posPattern == (posLength - 1) && posCycle == 0) {BEAT_MILLIS = BEAT_GAP_MILLIS;}
      else
      {
        if (posCycle==0){BEAT_MILLIS = BEAT_BTW_MILLIS;}    // Gap between numbers
        else
        {
          if (onStatus){BEAT_MILLIS = BEAT_ON_MILLIS;}      // LED on
          else {BEAT_MILLIS = BEAT_OFF_MILLIS;}             // LED off
        }
      }

      // Check if sufficent millis has passed since the last event
      if (curmillis > (lastEventTime + BEAT_MILLIS))
      {
        // Update the event time
        lastEventTime = curmillis;

        // Check if in the ON part of cycle
        if (onStatus)
        {
          // Change to the OFF part of the cycle
          onStatus = false;

          // Turn off the LED
          digitalWrite(lcPin, LOW);
        }
        else  // Finishing the OFF cycle
        {
          // Check the cycle
          if (posCycle == 0)
          {
            // Increment to the next position in the pattern
            posPattern++;

            // Check if you need to go back to the start of the pattern
            if (posPattern >= posLength)
            {
              // Restart the pattern
              posPattern = 0;
            }

            // With the next position, set the Cycle appropriately
            // Get the character as the posPosition in the pattern
            char curPattern;
            curPattern = ledPattern[posPattern];

            // Convert to an integer and set posCycle
            if (isDigit(curPattern))  // Get the ASCII of the text digit, then -48 ('0'=48 in ASCII)
            {
              posCycle = ((curPattern+0)-48);
              if (posCycle>0){posCycle--;}
            }
            else {posCycle = 1;}
          }
          else // Not at the end of the cycle
          {
            // move to the next cycle
            posCycle--;
          }
          
          // Change to the ON part of the cycle
          onStatus = true;

          // Turn on the LED
          digitalWrite(lcPin, HIGH);  
        }
      }
    }
  }
};   // Close LEDControl

// LCD Controller
class LCDMessageType
{
    // Allow the values to be directly accessed from outside the class
  public:

    // Values
    int mtTypeCode;   // Defined by the message type creator - just a simple index number
    int mtDisplay;    // Which display does the message appear on, 0 = Left, 1 = Right
    int mtLine;       // 0 = Upper line, 1 = Lower line
    int mtStart;      // Start position, 0-15
    int mtChars;      // Number of characters, 1-16
    int mtJustif;     // Left or Right Justified in segment, 0 = Left, 1 = Right
    int mtPriority;   // Level of priority of the message, 0 = Low, 5 = High
    int mtFrequency;  // Frequency of display of the message, 0 = Always displayed, N = seconds between displays
    int mtLongevity;  // How long to display a message, 0 = Until replaced, N = for N seconds (then expire)
    char mtTemplate[ARRAY_CHARS];   // Template into which the submitted value is placed, replacing '#' character

    // Constructor : Easy set up of the message types to be used
    LCDMessageType(int mTypeCode, int mDisplay, int mLine, int mStart, int mChars, int mJustif, char mTemplate[], int mPriority, int mFrequency, int mLongevity)
    {
      // Set the initial variable values
      mtTypeCode = mTypeCode;
      mtDisplay = mDisplay;
      mtLine = mLine;
      mtStart = mStart;
      mtChars = mChars;
      mtJustif = mJustif;
      mtPriority = mPriority;
      mtFrequency = mFrequency;
      mtLongevity = mLongevity;

      // Transfer the template charaters into the array
      int slen = strnlen(mTemplate, DISPLAY_CHARS);
      if(slen>DISPLAY_CHARS){slen=DISPLAY_CHARS;}
      int p = 0;
      for (p=0;p<slen;p++){mtTemplate[p] = mTemplate[p];}
      mtTemplate[slen] = '\0';
    }
};  // Close LCDMessageType

class LCDMessage
{
    // Allow the values to be directly accessed from outside the class
  public:

    // Values
    int msID;         // Unique ID for the message (so it can be found and removed or updated later
    int msTypeCode;   // TypeCode of the message (refers to LCDMessageType
    char msContent[ARRAY_CHARS]; // Value to the combined with the MessageType
    // unsigned long msTime;  // timestamp in millis, last time the information was posted

    // Constructor
  public:
    LCDMessage(int mID, int mTypeCode, char mContent[])
    {
      // Set the initial variable values
      msID = mID;
      msTypeCode = mTypeCode;

      // Transfer the template characters into the array
      int slen = strnlen(mContent, DISPLAY_CHARS);
      if(slen>DISPLAY_CHARS){slen=DISPLAY_CHARS;}
      int p = 0;
      for (p=0;p<slen;p++){msContent[p] = mContent[p];}
      msContent[slen] = '\0';
    }
};  // Close LCDMessage

// VARIABLES  ---------------------------------------------------------------------------------

// Gauge : Stepper Motor driver
    int clustermode = 0;  // A variable to globally state the clusters mode (0=initialization, 1= runtime)

    int speedotarget = 0;  // Set up the speedo gauge Target value
    int torquetarget = 0;
    
    // HSR Vehicle Speed
    float   DU_vehicleSpeed;
    int16_t DU_vehicleSpeedint;     // in 1/10th MPH
    
    // HSR Torque Output
    float DU_outputTorque;
    int16_t DU_outputTorqueint;     // units???
    
    // HSR Low Voltage
    float DU_LV_Input_Voltage = 0.0;    // volts
    int16_t DU_LV_Input_Voltageint;     // 1/8th volt
    
    // HSR Inlet Temp for instrument cluster
    int DU_inletT;                      //DegC

    // HSR Amps of Regen or Discharged power
    float DU_maxHVChargeRegenCurrent;
    float DU_maxHVDischargeCurrent;

    // HSR kW of Regen or Discharged power
    float DU_maxHVChargeRegenPower;
    float DU_maxHVDischargePower;

    // HSR Battery HV reading
    float DU_HV_Input_Voltage = 0.0;

// LED


// OLED
    // iOutput is a global variable updated by the last call to the getDisplayLine function
    char iOutput[ARRAY_CHARS] = "                ";   // 16 spaces + 1 terminated by a \0

// OBJECTS  ---------------------------------------------------------------------------------

// CAN-BUS Shield
    MCP2515 can0(10);     // CS on digitial pin 10

// Setup all the GAUGES : Stepper Motor driver
    //  Gauge(Stepper* stepperobj, int maxsteps, int stepincr, int maxdegrees, int gaugemin, int gaugemax, int stepspeed)

    // SPEEDO
    Stepper stepper1(STEPS, G1_1, G1_2, G1_3, G1_4);  // Create the LARGE RIGHT speedometer gauge
    Gauge speedo( &stepper1, STEPS, 1, 180, 0, 100, 60);  // 90 degrees, 0-100 range

    // TORQUE
    Stepper stepper2(STEPS, G2_1, G2_2, G2_3, G2_4);  // Create the LARGE LEFT torque gauge
    Gauge torque( &stepper2, STEPS, 1, 180, 0, 100, 60);    

    // LOWVOLTAGE
    Stepper stepper3(STEPS, G3_1, G3_2, G3_3, G3_4);  // Create the SMALL TOP LEFT 12V battery gauge
    Gauge lowvolts( &stepper3, STEPS, 1, 180, 0, 100, 60);    

    // TEMPERATURE
    Stepper stepper4(STEPS, G4_1, G4_2, G4_3, G4_4);  // Create the SMALL BOT LEFT coolant temp gauge
    Gauge temps( &stepper2, STEPS, 1, 180, 0, 100, 60);    

    // POWER
    Stepper stepper5(STEPS, G5_1, G5_2, G5_3, G5_4);  // Create the SMALL TOP RIGHT power gauge
    Gauge power( &stepper5, STEPS, 1, 180, 0, 100, 60);    

    // HV BATTERY
    Stepper stepper6(STEPS, G6_1, G6_2, G6_3, G6_4);  // Create the traction battery capacity gauge
    Gauge battery( &stepper6, STEPS, 1, 180, 0, 100, 60);    

// Setup all the WARNING LIGHTS : LED blinking light controls

    // TODO : Rename variables with more meaningful
    LEDControl warnlight1 = LEDControl(WARNING_LIGHT_1);  // Global Vars
    LEDControl warnlight2 = LEDControl(WARNING_LIGHT_2);  // Global Vars
    LEDControl warnlight3 = LEDControl(WARNING_LIGHT_3);  // Global Vars
    LEDControl warnlight4 = LEDControl(WARNING_LIGHT_4);  // Global Vars
    LEDControl warnlight5 = LEDControl(WARNING_LIGHT_5);  // Global Vars

// OLED Controller
    SSD1306AsciiWire lcdl;
    SSD1306AsciiWire lcdr;

// Set up the global lists
    LinkedList<LCDMessageType*> llMessageTypes = LinkedList<LCDMessageType*>();
    LinkedList<LCDMessage*> llMessages = LinkedList<LCDMessage*>();

// Set up the MessageTypes once in the global context
// Create all the message types to use
// LCDMessageType(int mTypeCode, int mDisplay, int mLine, int mStart, int mChars, int mJustify, String mTemplate, int mPriority, int mFrequency, int mLongevity)

  // Left Display
  // Upper line
  char mtp1[] = "BAT #%";
  LCDMessageType *t1 = new LCDMessageType(1, 0, 0, 0, 8, 0, mtp1, 0, 0, 0);
  
  char mtp2[] = "RNG #";
  LCDMessageType *t2 = new LCDMessageType(2, 0, 0, 9, 7, 1, mtp2, 0, 0, 0);
  
  // Bottom line
  char mtp3[] = "#";
  LCDMessageType *t3 = new LCDMessageType(3, 0, 1, 0, 7, 1, mtp3, 0, 0, 0);

  char mtp4[] = "#";
  LCDMessageType *t4 = new LCDMessageType(4, 0, 1, 12, 4, 1, mtp4, 0, 0, 0);
  
  // Warnings
  char mtp9[] = "Engage Handbrake";
  LCDMessageType *t9 = new LCDMessageType(9, 0, 1, 0, 16, 1, mtp9, 1, 0, 0);

  char mtp10[] = "Door Ajar";
  LCDMessageType *t10 = new LCDMessageType(10, 0, 1, 0, 16, 1, mtp10, 1, 5, 0);

  char mtp11[] = "Fasten Seatbelt";
  LCDMessageType *t11 = new LCDMessageType(11, 0, 1, 0, 16, 1, mtp11, 2, 0, 5);
  
  // Trouble Codes
  char mtp12[] = "12v System Low";
  LCDMessageType *t12 = new LCDMessageType(12, 0, 1, 0, 16, 1, mtp12, 3, 0, 0);

  char mtp13[] = "Hi Coolant Temp";
  LCDMessageType *t13 = new LCDMessageType(13, 0, 1, 0, 16, 1, mtp13, 3, 0, 0);

  char mtp14[] = "Lo Coolant Temp";
  LCDMessageType *t14 = new LCDMessageType(14, 0, 1, 0, 16, 1, mtp14, 3, 10, 60);
  
  // Cruise Control
  char mtp15[] = "Cruise READY";
  LCDMessageType *t15 = new LCDMessageType(15, 0, 1, 0, 16, 1, mtp15, 2, 0, 0);
  
  char mtp16[] = "Cruise OFF";
  LCDMessageType *t16 = new LCDMessageType(16, 0, 1, 0, 16, 1, mtp16, 2, 0, 30);
  
  char mtp17[] = "Cruise # MPH";
  LCDMessageType *t17 = new LCDMessageType(17, 0, 1, 0, 16, 1, mtp17, 2, 0, 0);

  // Right Display
  // Upper line
  char mtp5[] = "#";
  LCDMessageType *t5 = new LCDMessageType(5, 1, 0, 0, 8, 0, mtp5, 0, 0, 0);

  char mtp6[] = "#";
  LCDMessageType *t6 = new LCDMessageType(6, 1, 0, 8, 8, 1, mtp6, 0, 0, 0);

  // Bottom line
  char mtp7[] = "Trip #";
  LCDMessageType *t7 = new LCDMessageType(7, 1, 1, 0, 8, 0, mtp7, 0, 0, 0);

  char mtp8[] = "# MPH";
  LCDMessageType *t8 = new LCDMessageType(8, 1, 1, 8, 8, 1, mtp8, 0, 0, 0);

  // Information
  char mtp18[] = "# Miles/kWh";
  LCDMessageType *t18 = new LCDMessageType(18, 1, 1, 0, 16, 1, mtp18, 1, 0, 0);

  char mtp19[] = "# kWh/Mile";
  LCDMessageType *t19 = new LCDMessageType(19, 1, 1, 0, 16, 1, mtp19, 1, 0, 0);

  char mtp20[] = " Temp #F";
  LCDMessageType *t20 = new LCDMessageType(20, 1, 1, 0, 16, 1, mtp20, 1, 60, 5);

  // Warnings
  char mtp21[] = "Low Battery #%";
  LCDMessageType *t21 = new LCDMessageType(21, 1, 1, 0, 16, 1, mtp21, 2, 0, 0);

  char mtp22[] = "Wheel Slip F-R";
  LCDMessageType *t22 = new LCDMessageType(22, 1, 1, 0, 16, 1, mtp22, 2, 0, 5);

  char mtp23[] = "Wheel Slip L-R";
  LCDMessageType *t23 = new LCDMessageType(23, 1, 1, 0, 16, 1, mtp23, 2, 0, 5);

  // Trouble Codes
  char mtp24[] = "EPS Fault #";
  LCDMessageType *t24 = new LCDMessageType(24, 1, 1, 0, 16, 1, mtp24, 3, 0, 0);

  char mtp25[] = "Bat Fault #";
  LCDMessageType *t25 = new LCDMessageType(25, 1, 1, 0, 16, 1, mtp25, 3, 0, 0);

  char mtp26[] = "DU Fault #";
  LCDMessageType *t26 = new LCDMessageType(26, 1, 1, 0, 16, 1, mtp26, 3, 0, 0);

// Global cycle variable
int cycle;

// Real-time Clock
DS3231  rtc(SDA, SCL);  // Code from the Demo Example of the DS3231 Library

// FUNCTIONS  ---------------------------------------------------------------------------------

// CAN BUS

// Send the frame to the Serial port
void outputCANframe(can_frame & frame){

  // Send frame to the Serial Port
  Serial.print("CAN frame : ");
  Serial.print(frame.can_id,HEX);
  Serial.print(" - ");
  for (int i=0;i<frame.can_dlc;i++){
    Serial.print(frame.data[i],HEX);
    Serial.print(" ");
  }
  Serial.println();  
}

// Stepper Motor driver

// LED blinking light controls

// LCD Controller

// reverse a string in location
char *strrev(char *str)
{
  char *p1, *p2;

  if (! str || ! *str)
    return str;
  for (p1 = str, p2 = str + strlen(str) - 1; p2 > p1; ++p1, --p2)
  {
    *p1 ^= *p2;
    *p2 ^= *p1;
    *p1 ^= *p2;
  }
  return str;
}

void getReplaced(char* combo, const char* source, char wildchar,const char* content, int ichars)
{
  int slen = strnlen(source,DISPLAY_CHARS);
  int clen = strnlen(content,DISPLAY_CHARS);
  int pos = 0;

  for (int p=0;p<slen;p++)
  {
    if (pos<=ichars)
    {
      if (source[p] == wildchar)
      {
        // transfer the whole of the content to the combo
        for (int i=0;i<clen;i++)
        {
          if (pos<=ichars)
          {
            combo[pos] = content[i];
            pos++;
          }
        }
      }
      else    // not the wildcard
      {
        // transfer the source character into the combo
        combo[pos] = source[p];
        pos++;
      }
    }
  }
  combo[pos] = '\0';

//  return combo;
}

// Left justify the source char array in place
bool setLeftJustified(char* source, int numchars)
{
  // Test that the source can carry numchars characters
  if (DISPLAY_CHARS >= numchars)
  {
    // How many characters are there in the source string
    int slen = strnlen(source, DISPLAY_CHARS);
      
    // Test lengths to transfer, only need to modify is < or > numchars
    if (slen < numchars)
    {
      // Concatenate space characters to source
      for (int n=0;n<(numchars-slen);n++)
      {source[slen+n] = ' ';}
      source[numchars] = '\0';
    }
    if (slen > numchars)
    {
      // Just terminate at numchars
      source[numchars] = '\0';
    }

    return true;
  }
  else
  {
    return false;
  }
}

// Right justify the source char array in place
bool setRightJustified(char* source, int numchars)
{
  // Test that the source can carry numchars characters
  if (DISPLAY_CHARS >= numchars)
  {
    // How many characters are there in the source string
    int slen = strnlen(source,DISPLAY_CHARS);
    int i;  // general loop counter
      
    // Test lengths to transfer, only need to modify is < or > numchars
    if (slen < numchars)
    {
      // Reverse string, add spaces, the reverse again
      source = strrev(source);
      for (int n=0;n<(numchars-slen);n++)
      {source[slen+n] = ' ';}
      source[numchars] = '\0';
      source = strrev(source);
    }
    if (slen > numchars)
    {
      // Pull the last numchar characters off the source
      int cshift = slen - numchars;
      for (i=0;i<numchars;i++){source[i]=source[i+cshift];}
      source[numchars] = '\0';
    }

    return true;
  }
  else
  {
    return false;
  }
}

// Request the output text for a given display line (left/right, upper/lower)
void getDisplayLine(int iDisplay, int iLine)
{
  // UPDATES THE iOutput GLOBAL VARIABLE
  for(int j = 0; j < DISPLAY_CHARS; j++ ) {iOutput[j] = ' ';}
  iOutput[DISPLAY_CHARS] = '\0';
  
  // Get the number of messages
  int iNumMsgs;
  iNumMsgs = llMessages.size();

  // Cycle in increasing order of priority 0..3
  for (int iPrio = 0; iPrio < 4; iPrio++)
  {
    // Cycle through the messages
    for (int i = 0; i < iNumMsgs; i++)
    {
      // Fetch the MessageType referenced by this message
      LCDMessage *msg;
      msg = llMessages.get(i);

      LCDMessageType* msgtyp = getMessageType(msg->msTypeCode);

      // Test that the message type was found
      if (msgtyp != NULL)
      {
        // Test if the priority, display, and line are of interest
        if ((msgtyp->mtPriority == iPrio) && (msgtyp->mtDisplay == iDisplay) && (msgtyp->mtLine == iLine))
        {
          int iStart = msgtyp->mtStart;
          int iChars = msgtyp->mtChars;
          int iJustif = msgtyp->mtJustif;
          
          // Place the content into the Template
          char iBase[ARRAY_CHARS];
          for(int b = 0; b < DISPLAY_CHARS; b++ ) {iBase[b] = ' ';}
          iBase[DISPLAY_CHARS] = '\0';
          getReplaced(iBase, msgtyp->mtTemplate,'#',llMessages.get(i)->msContent,iChars);

          // Check for justification
          if (iJustif==1){setRightJustified(iBase,iChars);}   // Right
          if (iJustif==0){setLeftJustified(iBase,iChars);}   // Left

          // Transfer the characters from iBase to the iOutput array
          int iPos;
          for (iPos = 0; iPos < iChars; iPos++)
          {
            if ((iStart + iPos) < DISPLAY_CHARS)
            {
              iOutput[iStart + iPos] = iBase[iPos];
            }
          }
          // Put a string terminator on the end
          iOutput[DISPLAY_CHARS] = '\0';

          // Delete iBase to recover memory
          // delete(iBase);
        }
      }
    }
  }
}

LCDMessageType* getMessageType(int mTypeCode)
{
  int iNumMsgTypes;
  iNumMsgTypes = llMessageTypes.size();

  for (int i = 0; i < iNumMsgTypes; i++)
  {
    if (llMessageTypes.get(i)->mtTypeCode == mTypeCode)
    {
      // Return the pointer to the MessageType object
      return llMessageTypes.get(i);
    }
  }
  return NULL;    // In case the TypeCode provided isn't found
}

// Updates an existing message with new content
void updateMessage(int iID, char* iContent)
{
  // Run down the linked list looking for a matching message id, when found update the content
  int iNumMsgs;
  iNumMsgs = llMessages.size();

  for (int i = 0; i < iNumMsgs; i++)
  {
    if (llMessages.get(i)->msID == iID)
    {
      // Update the content of the message object
      strncpy(llMessages.get(i)->msContent, iContent,DISPLAY_CHARS);
    }
  }
}

// Add a message to the listing
void addMessage(int iID, int iTypeCode, char* iContent)
{
  // Create a new LCDMessage
  LCDMessage* msg = new LCDMessage(iID, iTypeCode, iContent);

  // Add the new Message to the linked list
  llMessages.add(msg);
}

// Remove a message from the listing (note: removes all messages with this ID)
void removeMessage(int iID)
{
  // Run down the linked list looking for a matching message id, when found remove
  int iNumMsgs;
  iNumMsgs = llMessages.size();

  for (int i = 0; i < iNumMsgs; i++)
  {
    if (llMessages.get(i)->msID == iID)
    {
      // Get a reference to the object
      LCDMessage *tempMsg = llMessages.remove(i);
      delete(tempMsg);
    }
  }
}

// Real-time Clock

// SETUP  ---------------------------------------------------------------------------------

void setup()
{
  // Initialize the serial port
  Serial.begin(9600);
  
  // CAN-BUS Shield Receive
  can0.reset();
  can0.setBitrate(CAN_500KBPS);
  can0.setNormalMode();

// Stepper Motor driver
  speedotarget = 0;    // Initialize the Speedo Target

// LED blinking light controls

  // Set up the warning light
  LEDControl warnlight(1);
  warnlight.InitLED();
  
  // Pattern Test
  char myPat[] = "113";
  warnlight.SetPattern(myPat);

// LCD Controller

  // Reset the global cycle counter
  cycle = 0;

// OLED Setup

  #if RST_PIN >= 0
    lcdl.begin(&Adafruit128x32, I2C_ADDR_LEFT, RST_PIN);
    lcdr.begin(&Adafruit128x32, I2C_ADDR_RGHT, RST_PIN);
  #else // RST_PIN >= 0
    lcdl.begin(&Adafruit128x32, I2C_ADDR_LEFT);
    lcdr.begin(&Adafruit128x32, I2C_ADDR_RGHT);
  #endif // RST_PIN >= 0
  // Set Font
  lcdl.setFont(ZevvPeep8x16);
  lcdr.setFont(ZevvPeep8x16);

  // Set the cursor in the home position
  lcdl.home ();
  lcdr.home ();

  // Start-up message
  lcdl.print("TesLorean DMC-EV");
  lcdl.setCursor (0, 1);
  lcdl.print("Start-up...");
  lcdl.home ();
  lcdr.print("By Jeff Cooke");
  lcdr.setCursor (0, 1);
  lcdr.print("Built 2012-18");
  lcdr.home ();

  // Set up the MessageTypes list
  llMessageTypes.add(t1);    // on left : "BAT 100%"
  llMessageTypes.add(t2);     // on right : "RNG 60"
  llMessageTypes.add(t3);    // on left : "100,000 " (mileage counter)
  llMessageTypes.add(t4);     // on right : "INCR", "AVG", or "DECR" (or arrow characters if available)
  llMessageTypes.add(t9);    // on left : "Engage Handbrake"
  llMessageTypes.add(t10);    // on left : "Door Ajar"
  llMessageTypes.add(t11);    // on left : "Fasten Seatbelt"
  llMessageTypes.add(t12);    // on left : "Low Voltage Low"
  llMessageTypes.add(t13);    // on left : "Hi Coolant Temp"
  llMessageTypes.add(t14);    // on left : "Low Coolant Temp"
  llMessageTypes.add(t15);    // on left : "Cruise Ready"
  llMessageTypes.add(t16);    // on left : "Cruise Off"
  llMessageTypes.add(t17);    // on left : "Cruise 40 MPH"
  llMessageTypes.add(t5);    // on left : "PARK/DRIVE/REVERSE/NEUTRAL"
  llMessageTypes.add(t6);    // on right : "08:23 AM"
  llMessageTypes.add(t7);    // on left : "Trip 23"
  llMessageTypes.add(t8);    // on right : "45 MPH"
  llMessageTypes.add(t18);    // center : "3.1 Miles/kWh"
  llMessageTypes.add(t19);    // center : "3.1 kWh/Mile"
  llMessageTypes.add(t20);    // center : "Temp 83*"
  llMessageTypes.add(t21);    // center : "Battery Low 10%"
  llMessageTypes.add(t22);    // center : "Wheel Slip F-R"
  llMessageTypes.add(t23);    // center : "Wheel Slip L-R"
  llMessageTypes.add(t24);    // center : "EPS Fault 101"
  llMessageTypes.add(t25);    // center : "BAT Fault A01"
  llMessageTypes.add(t26);    // center : "DU Fault 101"

  // Add the permanent messages (ones that regularly get updated with new content)
  addMessage(1, 1, (char *)"95");
  addMessage(2, 2, (char *)"65");
  addMessage(3, 3, (char *)"26,234");
  addMessage(4, 4, (char *)"AVG");
  addMessage(5, 5, (char *)"Park");
  addMessage(6, 6, (char *)"08:23AM");
  addMessage(7, 7, (char *)"25");
  addMessage(8, 8, (char *)"88");

// Real-time Clock

  // Initialize the rtc object
  rtc.begin();
  
  // The following lines can be uncommented to set the date and time
  rtc.setDOW(SUNDAY);        // Set Day-of-Week to SUNDAY
  rtc.setTime(9,0,0);     // Set the time to 12:00:00 (24hr format)
  rtc.setDate(12, 17, 2017);   // Set the date to January 1st, 2014

}

// LOOP  ---------------------------------------------------------------------------------

void loop()
{
    
// Gauge Sweeps

  // Test if the gauges are done sweeping
  boolean gaugesweepdone = false;

  // Initialize the gauges by sweeping up then down through the max steps
  if (clustermode == 0)    // initialization
  {
      // Instruct the gauge to sweep
     gaugesweepdone = speedo.Sweep();
     bool gaugedummy;
     gaugedummy = torque.Sweep();
     gaugedummy = lowvolts.Sweep();
     gaugedummy = temps.Sweep();
     gaugedummy = power.Sweep();
     gaugedummy = battery.Sweep();

      // Decide whether to switch into runtime mode (all gauges done initial sweep)
      if (gaugesweepdone)
      {
        clustermode = 1;    // switch to runtime mode
      }
  }
  // On every loop cycle progress gauge movement towards their current value
  if (clustermode == 1)   // runtime
  {
      speedo.Move();
      torque.Move();
      lowvolts.Move();
      temps.Move();
      power.Move();
      battery.Move();
  }

// LED blinking light controls

  // Lights status check
  warnlight1.CheckStatus();
  warnlight2.CheckStatus();
  warnlight3.CheckStatus();
  warnlight4.CheckStatus();
  warnlight5.CheckStatus();

// LCD Controller

  // Update the cycle counter
  cycle++;

  // Display the messages
  lcdl.setCursor(0, 0);
  getDisplayLine(0, 0);
  lcdl.print(iOutput);
  lcdl.setCursor(0, 1);
  getDisplayLine(0, 1);
  lcdl.print(iOutput);
  lcdr.setCursor(0, 0);
  getDisplayLine(1, 0);
  lcdr.print(iOutput);
  lcdr.setCursor(0, 1);
  getDisplayLine(1, 1);
  lcdr.print(iOutput);

  // Display a change in the speedo value
  if (cycle >= 5)
  {
    int speedo = 30 + cycle;
    char speedochars[DISPLAY_CHARS];
    itoa(speedo, speedochars, 10);    // itoa is a standard C++ routine (10 = base 10)
    updateMessage(8, speedochars);
  }
  if (cycle == 10)
  {
    char msg10[] = "Dummy";
    addMessage(10,11,msg10);
  }
  if (cycle == 15)
  {
    removeMessage(10);
  }

// Real-time Clock

  Serial.print(rtc.getDOWStr());      // Send Day-of-Week
  Serial.print(rtc.getDateStr());         // Send date
  Serial.println(rtc.getTimeStr());     // Send time
  Serial.println(rtc.getTemp());       // Get Temp
  
// MAIN CODE - check for incoming CAN frame, text ID and update instrument cluster variables

// CANbus receiver

    // Define CAN Frame variable structure
    struct can_frame incoming;

    // check if CAN frame waiting
    if (can0.readMessage(&incoming) == MCP2515::ERROR_OK)
    {

      // Output FRAME ID and bytes in CSV format
      //Serial.print(incoming.can_id, HEX); // print ID
      //Serial.print(incoming.can_dlc, DEC); // print DLC
      //Serial.print(incoming.data[i],HEX);

      int powerregensetting = 0;
      int powerdischsetting = 0;
      int powersetting = 0;
      int batterysetting = 0;
          
      switch (incoming.can_id)
      {
        case 0x115:  // HSR_speedData

          DU_vehicleSpeedint = (incoming.data[2] << 8) | incoming.data[3]; 
          DU_vehicleSpeed = DU_vehicleSpeedint / 10.0; // MPH
          DU_vehicleSpeedint = (uint16_t)DU_vehicleSpeed;

          // Update the speedo gauge
          speedo.Update(DU_vehicleSpeedint);
          
          break;
          
        case 0x116:  // HSR_torquePowerData

          DU_outputTorqueint = (incoming.data[0] << 8) | incoming.data[1];
          DU_outputTorque = DU_outputTorqueint / 4.0;
          DU_outputTorqueint = (uint16_t)DU_outputTorque;

          torque.Update(DU_outputTorqueint);

          break;
          
        case 0x119:  // HSR_HVLVdata
          DU_LV_Input_Voltage = incoming.data[4] / 8.0; // (Volts)
          DU_LV_Input_Voltageint = incoming.data[4];  //  1/8th of a volt
          
          lowvolts.Update(DU_LV_Input_Voltageint);

          DU_HV_Input_Voltage = ((incoming.data[0] << 8) | incoming.data[1]) / 8.0;  // (Volts)

          batterysetting = (DU_HV_Input_Voltage - BATTLOWVOLTS) / (BATTHIGHVOLTS - BATTLOWVOLTS);
          battery.Update(batterysetting);

          break;
          
        case 0x506:  // HSR_DI_temperature
          DU_inletT       = incoming.data[5] - 40;  // degC

          temps.Update(DU_inletT);

          break;

        case 0x120:  // HSR_powerData
          DU_maxHVChargeRegenPower   = ((incoming.data[4] << 8) | incoming.data[5]) / 10.0; // (kW)
          DU_maxHVDischargePower     = ((incoming.data[6] << 8) | incoming.data[7]) / 10.0; 

          powerregensetting = 50 * (DU_maxHVChargeRegenPower/MAXHVREGENPOWER);
          powerdischsetting = 50 * (DU_maxHVDischargePower/MINHVDISCHPOWER);
          powersetting = 50 - powerdischsetting + powerregensetting;
    
          power.Update(powersetting);

          break;

        default:
          // if nothing else matches, do the default
          break;
      }

    } // End of CAN frame check

}

// END  ---------------------------------------------------------------------------------
