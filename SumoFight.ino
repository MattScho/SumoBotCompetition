/* 
* Sumo Competition 10/5/2018
* Bot/Team: TOMM
* Team Members: Matthew Schofield, Olga Koturlash and Tom Schofield
*/

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

unsigned int lineSensorValues[3];

// When the reading line sensor falls below this number the bot thinks it is on the line.
const uint16_t lineSensorThreshold = 700;

// Reverse speed.
const uint16_t reverseSpeed = 400;

const uint16_t PROX_LEFT_THRESHOLD = 4;
const uint16_t PROX_RIGHT_THRESHOLD = 3;

uint16_t kappa = 0;
uint16_t nu = 0;
uint16_t tao = 0;

// Turn speed.
const uint16_t turnSpeed = 400;

// Forward speed.
const uint16_t forwardSpeed = 400;

// Veer speeds
const uint16_t veerSpeedLow = 200;
const uint16_t veerSpeedHigh = 400;

// Attac speed
const uint16_t rammingSpeed = 400;

// Amount of time to panick and run backward when line is seen
const uint16_t reverseTime = 200;

// The minimum amount of time to spend scanning for nearby
// opponents, in milliseconds.
const uint16_t scanTimeMin = 200;

// The maximum amount of time to spend scanning for nearby
// opponents, in milliseconds.
const uint16_t scanTimeMax = 2100;

// The amount of time to wait between detecting a button press
// and actually starting to move, in milliseconds.  Typical robot
// sumo rules require 5 seconds of waiting.
const uint16_t waitTime = 5000;

// If the robot has been driving forward for this amount of time,
// in milliseconds, without reaching a border, the robot decides
// that it must be pushing on another robot and this is a
// stalemate, so it increases its motor speed.
const uint16_t stalemateTime = 4000;

// This enum lists the top-level states that the robot can be in.
enum State
{
  StatePausing,
  StateWaiting,
  StateScanning,
  StateDriving,
  StateBacking,
};

State state = StatePausing;

enum Direction
{
  DirectionLeft,
  DirectionRight,
  DirectionForward,
  DirectionBack
};

// scanDir is the direction the robot should turn the next time
// it scans for an opponent.
Direction scanDir = DirectionLeft;

// The time, in milliseconds, that we entered the current top-level state.
uint16_t stateStartTime;

// The time, in milliseconds, that the LCD was last updated.
uint16_t displayTime;

// This gets set to true whenever we change to a new state.
// A state can read and write this variable this in order to
// perform actions just once at the beginning of the state.
bool justChangedState;

// This gets set whenever we clear the display.
bool displayCleared;

void setup()
{
  // Uncomment if necessary to correct motor directions:
  // motors.flipLeftMotor(true);
  // motors.flipRightMotor(true);

  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();

  changeState(StatePausing);
}

void loop()
{
  bool buttonPress = buttonA.getSingleDebouncedPress();

  if (state == StatePausing)
  {
    // In this state, we just wait for the user to press button
    // A, while displaying the battery voltage every 100 ms.

    motors.setSpeeds(0, 0);

    if (justChangedState)
    {
      justChangedState = false;
      lcd.print(F("Press A"));
    }

    if (displayIsStale(100))
    {
      displayUpdated();
      lcd.gotoXY(0, 1);
      lcd.print(readBatteryMillivolts());
    }

    if (buttonPress)
    {
      // The user pressed button A, so go to the waiting state.
      changeState(StateWaiting);
    }
  }
  else if (buttonPress)
  {
    // The user pressed button A while the robot was running, so pause.
    changeState(StatePausing);
  }
  else if (state == StateWaiting)
  {
    // In this state, we wait for a while and then move on to the
    // scanning state.

    motors.setSpeeds(0, 0);

    uint16_t time = timeInThisState();

    if (time < waitTime)
    {
      // Display the remaining time we have to wait.
      uint16_t timeLeft = waitTime - time;
      lcd.gotoXY(0, 0);
      lcd.print(timeLeft / 1000 % 10);
      lcd.print('.');
      lcd.print(timeLeft / 100 % 10);
    }
    else
    {
      // We have waited long enough.  Start moving.
      changeState(StateScanning);
    }
  }
  else if (state == StateBacking)
  {
    // In this state, the robot drives in reverse.

    if (justChangedState)
    {
      justChangedState = false;
      lcd.print(F("back"));
    }


    if(scanDir == DirectionLeft){
      motors.setSpeeds(-reverseSpeed, -reverseSpeed+100);
    }else if(scanDir == DirectionRight){
      motors.setSpeeds(-reverseSpeed + 100, -reverseSpeed);
    }else{
       motors.setSpeeds(-reverseSpeed, -reverseSpeed);
    }
    // After backing up for a specific amount of time, start
    // scanning.
    if (timeInThisState() >= reverseTime)
    {
      changeState(StateScanning);
    }
  }
  else if (state == StateScanning)
  {
    // In this state the robot rotates in place and tries to find
    // its opponent.

    if (justChangedState)
    {
      justChangedState = false;
      lcd.print(F("scan"));
      ledRed(0);
      ledGreen(0);
      ledYellow(1);
    }

    if (scanDir == DirectionLeft)
    {
      motors.setSpeeds(-turnSpeed+200, turnSpeed);
      if(nu < 100){
        nu++;
      }else{
        scanDir = DirectionRight;
      }
    }
    else
    {
      motors.setSpeeds(turnSpeed, -turnSpeed+300);
      if(nu >0 and tao %3 == 0){
        nu--;
      }else{
        nu = 0;
        tao++;
        scanDir = DirectionLeft;
      }
    }

    uint16_t time = timeInThisState();


// !!! check !!
    /**if (time > scanTimeMax)
    {
      // We have not seen anything for a while, so start driving.
      changeState(StateDriving);
    }
    else if (time > scanTimeMin)
    {*/
      // Read the proximity sensors.  If we detect anything with
      // the front sensor, then start driving forwards.
      proxSensors.read();
      // !!!Break down to left right !!!
      if (proxSensors.countsFrontWithLeftLeds() > PROX_LEFT_THRESHOLD
        || proxSensors.countsFrontWithRightLeds() > PROX_RIGHT_THRESHOLD)
      {
        changeState(StateDriving);
      }
    
  }
  else if (state == StateDriving)
  {
    // In this state we drive forward while also looking for the
    // opponent using the proximity sensors and checking for the
    // white border.

    if (justChangedState)
    {
      justChangedState = false;
      lcd.print(F("drive"));
      ledRed(0);
      ledGreen(1);
      ledYellow(0);
    }

      motors.setSpeeds(forwardSpeed, forwardSpeed);
    // Check for borders.
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[0] < lineSensorThreshold)
    {
      scanDir = DirectionRight;
      changeState(StateBacking);
    }
    if (lineSensorValues[2] < lineSensorThreshold)
    {
      scanDir = DirectionLeft;
      changeState(StateBacking);
    }
    if(lineSensorValues[1] <  lineSensorThreshold/4){
      changeState(StateBacking);
    }

    // Read the proximity sensors to see if know where the
    // opponent is.
    proxSensors.read();
    uint8_t sum = proxSensors.countsFrontWithRightLeds() + proxSensors.countsFrontWithLeftLeds();
    int8_t diff = proxSensors.countsFrontWithRightLeds() - proxSensors.countsFrontWithLeftLeds();

    if (proxSensors.countsFrontWithLeftLeds() <= PROX_LEFT_THRESHOLD
        && proxSensors.countsFrontWithRightLeds() <= PROX_RIGHT_THRESHOLD && timeInThisState() > 500)
    {
      changeState(StateScanning);
    }
    else
    {
      //!!!! CHANGE!!!!!
      // We see something with the front sensor but it is not a
      // strong reading.
      motors.setSpeeds(forwardSpeed, forwardSpeed);

      if (proxSensors.countsFrontWithLeftLeds() <= PROX_LEFT_THRESHOLD
        && proxSensors.countsFrontWithRightLeds() > PROX_RIGHT_THRESHOLD)
      {
        // The right-side reading is stronger, so veer to the right.
        motors.setSpeeds(veerSpeedHigh, veerSpeedLow);
        if(kappa != 1){
          lcd.clear();
          kappa = 1;
          lcd.print(F("ATTACK RIGHT"));
        }
      }
      else if (proxSensors.countsFrontWithLeftLeds() > PROX_LEFT_THRESHOLD
        && proxSensors.countsFrontWithRightLeds() <= PROX_RIGHT_THRESHOLD)
      {
        // The left-side reading is stronger, so veer to the left.
        motors.setSpeeds(veerSpeedLow, veerSpeedHigh);
        if(kappa != 2){
          lcd.clear();
          kappa = 2;
          lcd.print(F("ATTACK LEFT"));
        }
      
      }
      else
      {
        // Both readings are equal, so just drive forward.
        motors.setSpeeds(forwardSpeed, forwardSpeed);
        if(kappa != 3){
          lcd.clear();
          kappa = 3;
          lcd.print(F("ATTAC"));
        }
      }
    }
  }
}

// Gets the amount of time we have been in this state, in
// milliseconds.  After 65535 milliseconds (65 seconds), this
// overflows to 0.
uint16_t timeInThisState()
{
  return (uint16_t)(millis() - stateStartTime);
}

// Changes to a new state.  It also clears the LCD and turns off
// the LEDs so that the things the previous state were doing do
// not affect the feedback the user sees in the new state.
void changeState(uint8_t newState)
{
  state = (State)newState;
  justChangedState = true;
  stateStartTime = millis();
  ledRed(0);
  ledYellow(0);
  ledGreen(0);
  lcd.clear();
  displayCleared = true;
}

// Returns true if the display has been cleared or the contents
// on it have not been updated in a while.  The time limit used
// to decide if the contents are staled is specified in
// milliseconds by the staleTime parameter.
bool displayIsStale(uint16_t staleTime)
{
  return displayCleared || (millis() - displayTime) > staleTime;
}

// Any part of the code that uses displayIsStale to decide when
// to update the LCD should call this function when it updates the
// LCD.
void displayUpdated()
{
  displayTime = millis();
  displayCleared = false;
}
