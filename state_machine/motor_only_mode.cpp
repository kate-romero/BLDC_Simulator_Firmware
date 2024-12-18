#include "motor_only_mode.h"
#include "shared_data.h"
#include <Adafruit_NeoPixel.h>
#include "my_display.h"
#include "positive_mod.h"

#define MY_CORE       0
#define NUM_RING      24
#define IN_H1         9
#define IN_H2         11
#define IN_H3         13
#define RECOVER_TIME  10    // min to prevent flashing: 1
#define MOTOR_COLOR		0x00FF00
#define US_PER_MIN    1000000
#define MOTOR_FILTER_WINDOW_SZ   24
#define SAMPLE_TIME_US        100000
#define VHALL_SCALE_MULT      98235
#define MV_SCALE_DIV       10
#define TEMP_FILTER_WINDOW_SZ 10

MeanFilter<long> motorMeanFilter(MOTOR_FILTER_WINDOW_SZ);
MeanFilter<long> passthroughMeanFilter(MOTOR_FILTER_WINDOW_SZ);
MeanFilter<long> interceptPositionFilter(MOTOR_FILTER_WINDOW_SZ);
MeanFilter<long> interceptVelocityMeanFilter(MOTOR_FILTER_WINDOW_SZ);
MeanFilter<float> tempFilter(TEMP_FILTER_WINDOW_SZ);

//              LSB -> MSB
int inPins[] = {IN_H3, IN_H2, IN_H1};
int inPinsLength = (int)(sizeof(inPins) / sizeof(*inPins));
// member variables
// int encoding[] = {5, 4, 6, 2, 3, 1};
// int encodingLength = (int)(sizeof(encoding) / sizeof(*encoding));

const _thermistor_info* thermInfoPtrs[] = {&therm_info_std, &therm_info_koford, &therm_info_kollmorgen};


void setupMotorOnlyMode()
{
  for(int i = 0; i < inPinsLength; i++)
  {
    pinMode(inPins[i], INPUT_PULLUP);
  }
  pinMode(28, INPUT);
}

int readHallState()
{
  int state = 0;
  for(int idx = 0; idx < inPinsLength; idx++)
  {
    state = state | (digitalRead(inPins[idx]) << idx);
  }
  return state;
}

int getStartStateIdx(int state)
{
  int i;
  for(i = 0; i < encodingLength; i++)
  {
    if(encoding[i] == state)
    {
      return i;
    }
  }
  return i;
}

int getHallRotationDirection(int state, int lastStateIdx)
{
  if(state == encoding[lastStateIdx])
  {
    return 0;
  }
  else if(state == encoding[positiveMod(lastStateIdx+1, encodingLength)])
  {
    return 1;
  }
  else if(state == encoding[positiveMod(lastStateIdx-1, encodingLength)])
  {
    return -1;
  }
  return encodingLength;
}

int convertToPixelPosition(int position, int mode)
{
  int positionDegrees;
  switch(mode)
  {
    case MOTOR_ONLY_MODE:
      positionDegrees = position * (360 / motorOnlyConfig.countsPerRev);
      break;
    case PASSTHROUGH_MODE:
      positionDegrees = position * (360 / passthroughConfig.countsPerRev);
      break;
    case INTERCEPT_POSITION_MODE:
      positionDegrees = position * (360 / interceptPositionConfig.countsPerRev);
      break;
    case INTERCEPT_VELOCITY_MODE:
      positionDegrees = position * (360 / interceptVelocityConfig.countsPerRev);
      break;
    defualt:
      mySerialPrintln("ERROR: unexpected mode value!");
      break;
  }
  return (positionDegrees / (360 / NUM_RING));
}

int calculateRPM(int meanDeltaTime, int mode)
{
  int rpm;
  if(meanDeltaTime == 0)
  {
    rpm = 0;
  }
  else
  {
    switch(mode)
    {
      case MOTOR_ONLY_MODE:
        rpm = (US_PER_MIN) / (motorOnlyConfig.countsPerRev * meanDeltaTime);
        break;
      case PASSTHROUGH_MODE:
        rpm = (US_PER_MIN) / (passthroughConfig.countsPerRev * meanDeltaTime);
        break;
      case INTERCEPT_POSITION_MODE:
        rpm = (US_PER_MIN) / (interceptPositionConfig.countsPerRev * meanDeltaTime);
        break;
      case INTERCEPT_VELOCITY_MODE:
        rpm = (US_PER_MIN) / (interceptVelocityConfig.countsPerRev * meanDeltaTime);
        break;
      defualt:
        mySerialPrintln("ERROR: unexpected mode value!");
        break;
    }
  }
  return rpm;
}

int tempRequested(int mode)
{
  int requested = 0;
  switch(mode)
  {
    case MOTOR_ONLY_MODE:
      requested = motorOnlyConfig.tempSensor;
    defualt:
      break;
  }
  return requested;
}

void monitorTemp(int mode)
{
  static uint32_t sampleTime = 0;
  static int filterCnt = 0;
  float meanHallV;
  uint32_t now = time_us_32();

  if(now >= sampleTime)
  {
    meanHallV = tempFilter.AddValue((float)(analogRead(MOTOR_TEMP_SENSE) * (float)ANALOG_VOLT_REF) / (float)ANALOG_RESOLUTION);
    sampleTime = now + SAMPLE_TIME_US;
    filterCnt++;
  }
  if(filterCnt >= 10)
  {
    sendToPrinter(TEMP_BUFF, meanHallV, mode);
    filterCnt = 0;
  }

}

void motorMode(int newMode, int clearInvalidCnt, int mode)
{
  // static int state = 0;
  static int errorFlag = 0;
  static int lastStateIdx = 0;
  static int motorDisplacement;
  static int pixelPosition;
  static int invalidCnt = 0;
  static uint32_t prevTime;

  if(tempRequested(mode))
  {
    monitorTemp(mode);
  }

  if(clearInvalidCnt)
  {
    invalidCnt = 0;
    sendToPrinter(INVALID_CNT_BUFF, invalidCnt, mode);
  }
  int state = readHallState();
  if(state == 0 || state == 7)
  {
    resetStateStrip(MY_CORE);
    setStrip(state, MY_CORE, MOTOR_COLOR);
    // pixels.show();
  }

  uint32_t currTime = time_us_32();

  if(newMode || errorFlag)
  {
    if(newMode)
    {
      invalidCnt = 0;
    }
    // Serial.print("--- STATE: ");
    // Serial.println(state);
    // Serial.println("--- NEW MODE");
    fillRing(MY_CORE, BLACK);   // TODO: on entry to mode?
    lastStateIdx = getStartStateIdx(state);
    if(lastStateIdx >= encodingLength)
    {
      errorFlag = 1;
      invalidCnt++;
      delay(RECOVER_TIME);    // allow for recover time; prevent flashing
    }
    else    // setup
    {
      errorFlag = 0;
      motorDisplacement = 0;
      prevTime = currTime;
      sendToPrinter(POSITION_BUFF, motorDisplacement, mode);
      sendToPrinter(VELOCITY_BUFF, 0, mode);
      pixelPosition = positiveMod(motorDisplacement, NUM_RING);
      resetStateStrip(MY_CORE);
      setStateStrip(lastStateIdx, MY_CORE, MOTOR_COLOR);
      setRing(0, MY_CORE, MOTOR_COLOR);
      // pixels.show();
    }
    sendToPrinter(INVALID_CNT_BUFF, invalidCnt, mode);
  }
  else
  {
    int direction = getHallRotationDirection(state, lastStateIdx);
    if(direction == encodingLength)
    {
      // Serial.print("--- STATE: ");
      // Serial.println(state);
      errorFlag = 1;
      sendToPrinter(INVALID_CNT_BUFF, ++invalidCnt, mode);
    }
    else if(direction != 0)    // movement -> update
    {
      // Serial.print("--- STATE: ");
      // Serial.println(state);
      // Serial.print("--- DIRECTION: ");
      // Serial.println(direction);
      motorDisplacement += direction;
      // Serial.print("--- MOTOR DISPLACEMENT: ");
      // Serial.println(motorDisplacement);
      setStateStrip(lastStateIdx, MY_CORE, BLACK);
      lastStateIdx = positiveMod(lastStateIdx + direction, encodingLength);
      setStateStrip(lastStateIdx, MY_CORE, MOTOR_COLOR);
      // Serial.print("--- LAST STATE IDX: ");
      // Serial.println(lastStateIdx);

      // print displacement, set pixels
      setRing(pixelPosition, MY_CORE, BLACK);
      sendToPrinter(POSITION_BUFF, motorDisplacement, mode);
      // Serial.print("--- DELTA TIME: ");
      // Serial.println(currTime - prevTime);
      long addedVal;
      sendToPrinter(VELOCITY_BUFF, (int)(addedVal = motorMeanFilter.AddValue(direction * (currTime - prevTime))), mode);    // cast okay?
      // Serial.print("--- ADDED VAL: ");
      // Serial.println(addedVal);
      prevTime = currTime;
      int motorPosition;
      switch(mode)
      {
        case MOTOR_ONLY_MODE:
          motorPosition = positiveMod(motorDisplacement, motorOnlyConfig.countsPerRev);
          break;
        case PASSTHROUGH_MODE:
          motorPosition = positiveMod(motorDisplacement, passthroughConfig.countsPerRev);
          break;
        case INTERCEPT_POSITION_MODE:
          motorPosition = positiveMod(motorDisplacement, interceptPositionConfig.countsPerRev);
          break;
        case INTERCEPT_VELOCITY_MODE:
          motorPosition = positiveMod(motorDisplacement, interceptVelocityConfig.countsPerRev);
          break;
        defualt:
          mySerialPrintln("ERROR: unexpected mode value!");
          break;
      }
      // Serial.print("--- MOTOR POSITION: ");
      // Serial.println(motorPosition);
      pixelPosition = convertToPixelPosition(motorPosition, mode);
      // Serial.print("--- PIXEL POSITION: ");
      // Serial.println(pixelPosition);
      setRing(pixelPosition, MY_CORE, MOTOR_COLOR);
      // pixels.show();
    }
  }
}