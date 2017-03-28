#include <NewPing.h>

#include <Sonar.h>



#define SONARPIN 6

#define AZMPIN 3
#define CLOCKWISE_MS 0.5
#define CNTRCLOCKWISE_MS 1
#define MSPERDEGREE 3.2

#define NUM_OF_BINS (NUM_POSITIONS)
#define TOTAL_RTN_TIME_MS 1200
#define SAMPLETIME_MS 10
#define NUMSAMPLES 5
#define SAMPELSTATE_MS (NUMSAMPLES * SAMPLETIME_MS)
#define ANGLESLICETIME_MS (TOTAL_RTN_TIME_MS/NUM_OF_BINS)
#define RUNNINGTIME_MS (ANGLESLICETIME_MS)

#define MAXANGLE 360 // Used in setting current position limits in naming locations
#define MINANGLE 0

Sonar SONAR1(SONARPIN); 
#define NORMAL_RANGE 10000

unsigned long long directionCounter = 0;
unsigned long long runningTimeMark = 0;
unsigned long long sampleTimeMark = 0;

unsigned long long CWlastPwmEvent = 0;
volatile bool CW_en = false;
bool CW_on = false;

unsigned long long CCWlastPwmEvent = 0;
volatile bool CCW_en = false;
bool CCW_on = false;

enum controlState_t {INIT,RUNNING,SAMPLE,PROCESS,SOUNDALERT,KILLALL,REST};
controlState_t controlState = INIT;




void setup() {
  pinMode(AZMPIN, OUTPUT);
  azmControlInit();
  Serial.begin(9600);
  delay(1000);
  SONAR1.reset();
  SONAR1.setNormalRange(NORMAL_RANGE);



}

// Intializes all the variables
void azmControlInit()
{
  CW_en = false;
  CCW_en = false;

  CWlastPwmEvent = 0;
  CCWlastPwmEvent = 0;
}



void loop() {
  azmControlTick();
}


void debugStatePrint()
{
  static controlState_t lastState = INIT;
  
  if (controlState != lastState)
  {
    lastState = controlState;
    Serial.println("CurrentState Is:");
    Serial.println(controlState);
  }
}

void controlSMtick()
{
  debugStatePrint();
  
  switch (controlState)
  {
    case INIT:
      {
        controlState = RUNNING;
        directionCounter = 0;
        runningTimeMark = millis();
        sampleTimeMark = 0;    
        
      }
      break;
    case RUNNING:
      {   
          enableCW();    
          if ((millis() - runningTimeMark) >= (RUNNINGTIME_MS))
          {
            controlState = SAMPLE;
            sampleTimeMark = millis();
            disableCW();             
          }
      }
       break;
    case SAMPLE:
           
          if ((millis() - sampleTimeMark) >= (SAMPELSTATE_MS))
          {
            controlState = PROCESS;
            runningTimeMark = millis();       
          }
     break;
     case PROCESS:
      {
        controlState = SOUNDALERT;
      }
      break; 
     case SOUNDALERT:
      {
        controlState = KILLALL;
      }
      break; 
     case KILLALL:
      {
        controlState = REST;     
      }
      break; 
     
        case REST:
      {
          
          disableCW();
          controlState = RUNNING; 
      }
      break; 
    

    default:
      {
        controlState = INIT;
      }
      break;
  }
}


  // Enable Clockwise Motion
  void enableCW()
  {
    CW_en = true;
  }

  // Disable Clockwise Motion
  void disableCW()
  {
    CW_en = false;
  }

  // Enable CounterClockwise Motion
  void enableCCW()
  {
    CCW_en = true;
  }

  // Disable Clockwise Motion
  void disableCCW()
  {
    CCW_en = false;
  }


  void tickServos()
  {
    if (CW_en == true)
    {
      tickCW();
    }
    else if (CCW_en == true)
    {
      tickCCW();
    }
  }

  void tickCW()
  {

    // Check to see if an event time has lapsed

    if ((millis() - CWlastPwmEvent) > CLOCKWISE_MS)
    { // Pulse Width has elapsed
      // Serial.println("TickCW Event");
      // Check to see if on or off
      if (CW_on == true) // On
      {
        //Serial.println("TickCW On -> Off");
        // Turn off the pwm
        digitalWrite(AZMPIN, LOW);
        // Report
        CW_on = false;
      }
      else // Off
      {
        // Serial.println("TickCW Off -> On");
        // Turn on the pwm
        digitalWrite(AZMPIN, HIGH);
        // Report
        CW_on = true;
      }
      // Log the last event
      CWlastPwmEvent = millis();
    }
    // No Event, Do nothing
  }

  void tickCCW()
  {
    // Check to see if an event time has lapsed

    if ((millis() - CCWlastPwmEvent) > CNTRCLOCKWISE_MS)
    { // Pulse Width has elapsed

      // Check to see if on or off
      if (CCW_on == true) // On
      {

        // Turn off the pwm
        digitalWrite(AZMPIN, LOW);
        // Report
        CCW_on = false;
      }
      else // Off
      {

        // Turn on the pwm
        digitalWrite(AZMPIN, HIGH);
        // Report
        CCW_on = true;
      }

      // Log the last event
      CCWlastPwmEvent = millis();
    }
    // No Event, Do nothing
  }

  // Ticks the appropriate clockwise
  // or counter clockwise
  void azmControlTick()
  {
    controlSMtick();
    tickServos();
  }
