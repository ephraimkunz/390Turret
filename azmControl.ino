#define AZMPIN 3
#define CLOCKWISE_MS 0.5
#define CNTRCLOCKWISE_MS 1
#define MSPERDEGREE 3.2


unsigned long long CWlastPwmEvent = 0;
unsigned long long CWlastDriveEvent = 0;
unsigned long long CWdriveTime = 0;
volatile bool CWdrive_en = false;
volatile bool CW_en = false;
bool CW_on = false;

unsigned long long CCWlastPwmEvent = 0;
unsigned long long CCWlastDriveEvent = 0;
unsigned long long CCWdriveTime = 0;
volatile bool CCWdrive_en = false;
volatile bool CCW_en = false;
bool CCW_on = false;

enum controlState_t {INIT,DIRECT,RUNNING,SAMPLE,OVER};
controlState_t controlState = INIT;


#define MAXANGLE 360 // Used in setting current position limits in naming locations
#define MINANGLE 0
signed int currentAngle = 0;

void setup() {
  pinMode(AZMPIN, OUTPUT);
  azmControlInit();
  Serial.begin(9600);
  delay(1000);



}

// Intializes all the variables
void azmControlInit()
{
  CW_en = false;
  CCW_en = false;

  CWdrive_en = false;
  CCWdrive_en = false;

  CWlastPwmEvent = 0;
  CWlastDriveEvent = 0;

  CCWlastPwmEvent = 0;
  CCWlastDriveEvent = 0;

  CWdriveTime = 0;
  CCWdriveTime = 0;

  currentAngle = 0;
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


#define NUM_OF_BINS 12
#define TOTAL_RTN_TIME_MS 1200
#define SAMPLETIME_MS 300
#define ANGLESLICETIME_MS (TOTAL_RTN_TIME_MS/NUM_OF_BINS)
#define RUNNINGTIME_MS (ANGLESLICETIME_MS)

unsigned long long directionCounter = 0;
unsigned long long runningTimeMark = 0;
unsigned long long sampleTimeMark = 0;




void controlSMtick()
{
  debugStatePrint();
  
  switch (controlState)
  {
    case INIT:
      {
        controlState = DIRECT;
        setCWdriveTime(ANGLESLICETIME_MS);
        directionCounter = 0;
        runningTimeMark = 0;
        sampleTimeMark = 0;    
  
      }
      break;
    case DIRECT:
      {     
        if (directionCounter == NUM_OF_BINS)
        {
          controlState = OVER;
         }
        else
        {
          controlState = RUNNING;
          runningTimeMark = millis();
          directionCounter++;
          enableDriveCW();
          
        }
      }
      break;
    case RUNNING:
      {          
          if ((millis() - runningTimeMark) >= (RUNNINGTIME_MS))
          {
            controlState = SAMPLE;
            sampleTimeMark = millis();            
          }
      }
       break;
    case SAMPLE:  
          if ((millis() - sampleTimeMark) >= (SAMPLETIME_MS))
          {
            controlState = DIRECT;       
          }
     break;
        case OVER:
      {
          disableCW();
      }
      break; 
    

    default:
      {
        controlState = INIT;
      }
      break;
  }
}






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

  // Ticks the appropriate clockwise
  // or counter clockwise
  void azmControlTick()
  {
    controlSMtick();
    tickDrive();
    tickServos();


  }
