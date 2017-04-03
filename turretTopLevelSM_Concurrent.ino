#include <NewPing.h>

#include <Sonar.h>



#define SONARPIN 6

#define LEDPIN 10
#define LED_FREQUENCY_HZ 4500
#define LED_HALFPERIOD_US ((1E6 / (LED_FREQUENCY_HZ*2)))

#define AZMPIN 3
#define CLOCKWISE_US 1000
#define CNTRCLOCKWISE_US 4000
#define USPERDEGREE 3200

#define MAXANGLE 90 // Used in setting current position limits in naming locations
#define MINANGLE 0

#define NUM_OF_BINS (3)
#define TOTAL_RTN_TIME_MS 1500
#define SAMPLETIME_MS (10)
#define NUMSAMPLES (5)

#define ANGLESLICETIME_MS (TOTAL_RTN_TIME_MS/NUM_OF_BINS)
#define ANGLESLICE_DEG (MAXANGLE/NUM_OF_BINS)

#define FORWARDTIME_MS (120) // 90 DEGREE 3 BINS
#define REVERSETIME_MS (150) // 90 DEGREE 3 BINS
#define SAMPELSTATE_MS (1000)
#define KILLALLTIME_MS (1000)



Sonar SONAR1(SONARPIN);
#define NORMAL_RANGE 10000

unsigned long long directionCounter = 0;
unsigned long long forwardTimeMark = 0;
unsigned long long reverseTimeMark = 0;
unsigned long long sampleTimeMark = 0;
unsigned long long killAllTimeMark = 0;
bool incORdec = true;

unsigned int locationCounter = 0;

unsigned long long CWlastPwmEvent = 0;
volatile bool CW_en = false;
bool CW_on = false;

unsigned long long CCWlastPwmEvent = 0;
volatile bool CCW_en = false;
bool CCW_on = false;

unsigned long long LEDlastPwmEvent = 0;
volatile bool LED_en = false;
bool LED_on = false;
/*
   We need to get the switch player values
   One full set up circle
   Rotate
   Sample
   If there is a hit, (already at angle)
   Sound Alarm, Fire 5 seconds
   Check Again,
   Either Continue firing or start search again


*/
enum controlState_t {INIT, CALIBRATION, FORWARD, REVERSE, SAMPLE, SOUNDALERT, KILLALL, REST};
controlState_t controlState = INIT;




void setup() {
  pinMode(AZMPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  initTurret();
  Serial.begin(9600);
  delay(1000);
  SONAR1.reset();
  SONAR1.setNormalRange(NORMAL_RANGE);



}

// Intializes all the variables
void initTurret()
{
  CW_en = false;
  CCW_en = false;
  LED_en = false;

  CWlastPwmEvent = 0;
  CCWlastPwmEvent = 0;
  LEDlastPwmEvent = 0;
}



void loop() {
  allTicks();
}


void debugStatePrint()
{
  static controlState_t lastState = INIT;

  if (controlState != lastState)
  {
    lastState = controlState;
    Serial.println("CurrentState Is:");
    switch (controlState)
    {
      case INIT:
        Serial.println("INIT");
        break;

      case CALIBRATION:
        Serial.println("REST");
        break;

      case FORWARD:
        Serial.println("FORWARD");
        break;

      case REVERSE:
        Serial.println("REVERSE");
        break;

      case SAMPLE:
        Serial.println("SAMPLE");
        break;

      case SOUNDALERT:
        Serial.println("SOUNDALERT");
        break;

      case KILLALL:
        Serial.println("KILLALL");
        break;

      case REST:
        Serial.println("REST");
        break;

      default:
        Serial.println("default");
    }
  }
}

void controlSMtick()
{
  //debugStatePrint();

  switch (controlState)
  {
    case INIT:
      {
        directionCounter = 0;
        forwardTimeMark = millis();
        sampleTimeMark = 0;
        locationCounter = 0;

        controlState = FORWARD;
        enableCW();
        incORdec = true;
      }
      break;
    case FORWARD:
      {

        if ((millis() - forwardTimeMark) >= (FORWARDTIME_MS))
        {
          controlState = SAMPLE;
          sampleTimeMark = millis();
          disableCW();
          locationCounter++;
          
        }
      }
      break;
    case REVERSE:
      {
        if ((millis() - reverseTimeMark) >= (REVERSETIME_MS))
        {
          controlState = SAMPLE;
          sampleTimeMark = millis();
          disableCCW();
          locationCounter--;
        }
      }
      break;
    case SAMPLE:
      if(SONAR1.hitDetected(locationCounter)){
        controlState = SOUNDALERT;
        break;
      }
      else{
        if (incORdec)
        {
          if ((locationCounter == NUM_OF_BINS))
          {
            controlState = REVERSE;
            reverseTimeMark = millis();
            enableCCW();
            incORdec = false;
          } else
          {
            controlState = FORWARD;
            forwardTimeMark = millis();
            enableCW();
            incORdec = true;
          }
        }
        else
        {
          if ((locationCounter == 0))
          {
            controlState = FORWARD;
            forwardTimeMark = millis();
            enableCW();
            incORdec = true;
          } else
          {
            controlState = REVERSE;
            reverseTimeMark = millis();
            enableCCW();
            incORdec = false;
          }
        }
      }
      break;
    case SOUNDALERT:
      {
        enableLED();
        controlState = KILLALL;
        killAllTimeMark = millis();
      }
      break;
    case KILLALL:
      {
        if ((millis() - killAllTimeMark) >= (KILLALLTIME_MS))
        {
          controlState = INIT;
          forwardTimeMark = millis();
        }

      }
      break;

    case REST:
      {

        disableCW();
        disableLED();
        controlState = FORWARD;
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

// Disable CounterClockwise Motion
void disableCCW()
{
  CCW_en = false;
}

// Enable LED Strobe
void enableLED()
{
  LED_en = true;
}

// Disable LED Strobe
void disableLED()
{
  LED_en = false;
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

  if ((micros() - CWlastPwmEvent) > CLOCKWISE_US)
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
    CWlastPwmEvent = micros();
  }
  // No Event, Do nothing
}

void tickCCW()
{
 
  // Check to see if an event time has lapsed

  if ((micros() - CCWlastPwmEvent) > CNTRCLOCKWISE_US)
  {
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
    CCWlastPwmEvent = micros();
  }
  // No Event, Do nothing
}

void tickLED()
{
  // Check to see if an event time has lapsed

  if ((micros() - LEDlastPwmEvent) > LED_HALFPERIOD_US)
  { // Pulse Width has elapsed

    // Check to see if on or off
    if (LED_on == true) // On
    {

      // Turn off the pwm
      digitalWrite(LEDPIN, LOW);
      // Report
      LED_on = false;
    }
    else // Off
    {

      // Turn on the pwm
      digitalWrite(LEDPIN, HIGH);
      // Report
      LED_on = true;
    }

    // Log the last event
    LEDlastPwmEvent = micros();
  }
  // No Event, Do nothing
}







///////////////////////////////////////////
// ANGLE CODE
///////////////////////////////////////////

#define MSPERDEGREE 3.2
signed int currentAngle = 0;

void goToDegree(signed int azm)
{
  //Serial.println("Target Angle");
  //Serial.println(azm);

  // Get the difference
  signed int difference = azm - getCurrentAngle();

  //Serial.println("Difference");
  //Serial.println(difference);

  // If difference is positive,


  if (difference > 0)
  {



    // Rotate  CCW
    if (difference < 180)
    {


      rotateDegCCW(difference);
    } else
    {

      rotateDegCW(360 - difference);
    }
  }

  if (difference < 0)
  {
    // Rotate  CCW
    if (abs(difference) < 180)
    {

      rotateDegCW(abs(difference));
    } else
    {


      rotateDegCCW(360 - abs(difference));
    }
  }
}


// Rotates the given number of degrees
void rotateDegCW(unsigned int azmDeg)
{

  unsigned long runTime = azmDeg * MSPERDEGREE;
  setCWdriveTime(runTime);
  enableDriveCW();
  signed int negDelta = -1 * azmDeg;

  updateAngle(negDelta);


}

// Rotates the given number of degrees
void rotateDegCCW(unsigned int azmDeg)
{

  unsigned long runTime = azmDeg * MSPERDEGREE;
  setCCWdriveTime(runTime);
  enableDriveCCW();
  updateAngle(azmDeg);


}

// Updates the current location of the servo.
// Handles the rollover degrees
// positive deltaDeg is CW
// negative deltaDeg is CCW
void updateAngle(signed int deltaDeg)
{
  currentAngle = currentAngle + deltaDeg;

  // Handle Rollover
  if (currentAngle > MAXANGLE)
  {
    currentAngle = (currentAngle - MAXANGLE) + MINANGLE;
  }
  else if (currentAngle < MINANGLE)
  {
    currentAngle = MAXANGLE - abs(MINANGLE - currentAngle);
  }

}

// Returns the current angular position
signed int getCurrentAngle()
{
  return currentAngle;
}

///////////////////////////////////////////
// DRIVE CODE
///////////////////////////////////////////

unsigned long long CWlastDriveEvent = 0;
unsigned long long CWdriveTime = 0;
volatile bool CWdrive_en = false;

unsigned long long CCWlastDriveEvent = 0;
unsigned long long CCWdriveTime = 0;
volatile bool CCWdrive_en = false;

// Drives the turning on and off of the servos according to
// pre-set driveTime and preset drive_en if
// they are enabled with CWdrive_en and CCWdrive_en_en

void tickDrive()
{
  // ClockWise

  // Check to see if an event time has lapsed
  if (CWdrive_en == true)
  {

    // Check if starting disabled
    if (CW_en == false)
    {

      // Enable
      enableCW();
      // Log time
      CWlastDriveEvent = millis();
    }
    // Keep ticks enabled until the drive time has elapsed
    else if ((millis() - CWlastDriveEvent) >  CWdriveTime)
    {
      // Disable
      disableCW();
      // Turn off
      CWdrive_en = false;
      // Log time
      CWlastDriveEvent = millis();

    }
    // No Event, Do nothing
  } else

    // CounterClockWise

    // Check to see if an event time has lapsed
    if (CCWdrive_en == true)
    {
      // Check if starting disabled
      if (CCW_en == false)
      {

        // Enable
        enableCCW();
        // Log time
        CCWlastDriveEvent = millis();
      }
      // Keep ticks enabled until the drive time has elapsed
      else if ((millis() - CCWlastDriveEvent) >  CCWdriveTime)
      {
        // Disable
        disableCCW();
        // Turn off
        CCWdrive_en = false;
        // Log time
        CCWlastDriveEvent = millis();

      }
      // No Event, Do nothing
    }
}

// Sets the drive run time
void setCWdriveTime(unsigned long duration)
{
  CWdriveTime = duration;
}

// Enable drive
void enableDriveCW()
{
  CWdrive_en = true;
}

// Disable drive
void disableDriveCW()
{
  CWdrive_en = false;
}

// Sets the drive run time
void setCCWdriveTime(unsigned long duration)
{
  CCWdriveTime = duration;
}


// Enable drive
void enableDriveCCW()
{

  CCWdrive_en = true;
}

// Disable drive
void disableDriveCCW()
{
  CCWdrive_en = false;
}



///////////////////////////////////////////
// TICK ALL CODE
///////////////////////////////////////////


// Ticks the appropriate clockwise
// or counter clockwise
void allTicks()
{
  controlSMtick();
  tickServos();
  tickLED();
}
