#include <LinkedList.h>

/*
  Class used to control the setting, clearing, and coded blinking of LED instrument cluster lights

 by Jeff Cooke
 Dec 2017
 */

//#define DEBUG 1

#define BEAT_ON_MILLIS 250    // Duration of an on beat
#define BEAT_OFF_MILLIS 125    // Duration of an off beat
#define BEAT_BTW_MILLIS 500    // Duration of an between beat
#define BEAT_GAP_MILLIS 1500    // Duration between codes

#define WARNING_LIGHT 13          // Pin asigned

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
/*
#ifdef DEBUG
  Serial.print("Setup |");
  Serial.print(onStatus);
  Serial.print("|");
  Serial.print(ledPattern);
  Serial.print("|");
  Serial.print(posPattern);
  Serial.print("|");
  Serial.print(posCycle);
  Serial.print("|");
  Serial.print(posLength);
  Serial.println("|");
#endif
*/
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
/*
#ifdef DEBUG
  Serial.print("Loop |");
  Serial.print(onStatus);
  Serial.print("|");
  Serial.print(ledPattern);
  Serial.print("|");
  Serial.print(posPattern);
  Serial.print("|");
  Serial.print(posCycle);
  Serial.print("|");
  Serial.print(lastEventTime);
  Serial.print("|");
  Serial.print(curmillis);
  Serial.print("|");
  Serial.print(BEAT_MILLIS);
  Serial.println("|");
#endif
*/       
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
};

// Global Vars
LEDControl warnlight = LEDControl(WARNING_LIGHT);

// the setup function runs once when you press reset or power the board
void setup()
{

#ifdef DEBUG
  Serial.begin(9600);
#endif

  // Set up the warning light
  warnlight.InitLED();

/*
  // Simple Test
  warnlight.TurnOn();
  delay(BEAT_GAP_MILLIS);
  warnlight.TurnOff();
  delay(BEAT_GAP_MILLIS);
*/

  // Pattern Test
  char myPat[] = "113";
  warnlight.SetPattern(myPat);
}

// the loop function runs over and over again forever
void loop()
{
  // Lights status check
  warnlight.CheckStatus();
}

