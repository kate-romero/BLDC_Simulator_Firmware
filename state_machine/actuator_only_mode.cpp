#include "actuator_only_mode.h"
#include "positive_mod.h"
#include "my_display.h"
#include "my_neopixels.h"

#define MY_CORE         1
#define NUM_RING    	  24
#define VELOCITY_RPM_STEP   1
#define US_PER_MINUTE   60000000

#define ACTUATOR_COLOR	0x0000FF
#define FORWARD_COLOR   0x0000FF
#define REVERSE_COLOR   0x800080
#define SAMPLE_TIME_US     100000
#define VHALL_SCALE_MULT   98235
#define VHALL_MV_SCALE_DIV 10
#define HALL_FILTER_WINDOW_SZ  10

RotaryEncoder *encoder = nullptr;

MeanFilter<long> hallMVFilter(HALL_FILTER_WINDOW_SZ);

// pins
int outPins[] = {OUT_H1, OUT_H2, OUT_H3};
int outPinsLength = (int)(sizeof(outPins) / sizeof(*outPins));
// member variables
int encoding[] = {5, 4, 6, 2, 3, 1};
int encodingLength = (int)(sizeof(encoding) / sizeof(*encoding));

void setupActuatorOnlyMode()
{
  encoder = new RotaryEncoder(ENC_PIN1, ENC_PIN2, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN2), checkPosition, CHANGE);
  for(int i = 0; i < outPinsLength; i++)
  {
    pinMode(outPins[i], OUTPUT);
  }
  
}

void checkPosition()
{
  encoder->tick();
}

void writeHallState(int encodingIdx)
{
  for(int i = 0; i < outPinsLength; i++)
  {
    digitalWrite(outPins[i], (encoding[encodingIdx] >> i) & 1);
  }
}

int voltageRequested(int mode)
{
  int requested;
  switch (mode)
  {
    case ACTUATOR_POSITION_MODE:
      requested = actuatorPositionConfig.measureVoltage;
      break;
    case ACTUATOR_VELOCITY_MODE:
      requested = actuatorVelocityConfig.measureVoltage;
      break;
    case INTERCEPT_POSITION_MODE:
      requested = interceptPositionConfig.measureVoltage;
      break;
    case INTERCEPT_VELOCITY_MODE:
      requested = interceptVelocityConfig.measureVoltage;
      break;
    default:
      Serial.println("ERROR: unexpected mode!");
      break;
  }
  return requested;
}

void monitorVoltage(int displayOn)
{
  static uint32_t sampleTime = 0;
  static int filterCnt = 0;
  int meanHallMV;
  uint32_t now = time_us_32();

  if(now >= sampleTime)
  {
    meanHallMV = hallMVFilter.AddValue((analogRead(HALL_POWER_SENSE) * ANALOG_VOLT_REF * VHALL_SCALE_MULT) / (ANALOG_RESOLUTION * VHALL_MV_SCALE_DIV));
    // Serial.print("Sample : ");
    // Serial.println(meanHallMV);
    sampleTime = now + SAMPLE_TIME_US;
    filterCnt++;
  }
  if(filterCnt >= 10)
  {
    // Serial.print("Display: ");
    // Serial.println(meanHallMV);
    if(displayOn)
    {
      displayMV(meanHallMV);
    }
    filterCnt = 0;
  }

}

void actuatorPositionMode(int newMode, int forceState, int displayOn, int mode)
{
  static int direction;
  static int pixelPosition;
  static int32_t encoderPosition = encoder->getPosition();
  int forceStateVal;

  if(voltageRequested(mode))
  {
    monitorVoltage(displayOn);
  }

  switch (forceState)
  {
    case FORCE_ZERO:
      forceStateVal = 0;
      break;
    case FORCE_SEVEN:
      forceStateVal = 7;
      break;
    default:
      break;
  }

  if((direction = (encoder->getPosition() - encoderPosition)) || newMode)
  {
    lastUserInteractionTimeUS = time_us_32();
    if(newMode)
    {
      encoder->setPosition(0);
    }
    // Serial.println(encoder->getPosition());
    resetInvalidStates(MY_CORE);
    if(newMode)
    {
      pixelPosition = 0;
      fillRing(MY_CORE, BLACK);
      resetStateStrip(MY_CORE);
      
      switch (mode)
      {
        case ACTUATOR_POSITION_MODE:
          if(actuatorPositionConfig.forceStartState0)
          {
            forceState = 1;
            forceStateVal = 0;
          }
          else if(actuatorPositionConfig.forceStartState7)
          {
            forceState = 1;
            forceStateVal = 7;
          }
          break;
        case INTERCEPT_POSITION_MODE:
          if(interceptPositionConfig.forceStartState0)
          {
            forceState = 1;
            forceStateVal = 0;
          }
          else if(interceptPositionConfig.forceStartState7)
          {
            forceState = 1;
            forceStateVal = 7;
          }
          break;
        defualt:
          Serial.println("ERROR: unexpected mode!");
          break;
      }
    }
    setRing(pixelPosition, MY_CORE, BLACK);
    setStateStrip((pixelPosition) % encodingLength, MY_CORE, BLACK);
    encoderPosition += direction;
    pixelPosition = positiveMod(encoderPosition, NUM_RING);
    // displayPosition(encoderPosition, NUM_RING, displayOn);
    switch (mode)
    {
      case ACTUATOR_POSITION_MODE:
        displayPosition(encoderPosition, actuatorPositionConfig.countsPerRev, displayOn);
        break;
      case INTERCEPT_POSITION_MODE:
        displayPosition(encoderPosition, interceptPositionConfig.countsPerRev, displayOn);
        break;
      default:
        Serial.println("ERROR: unexpected mode!");
        break;
    }
    if(!forceState)
    {
      writeHallState((pixelPosition) % encodingLength);
      setStateStrip((pixelPosition) % encodingLength, MY_CORE, ACTUATOR_COLOR);
    }
    setRing(pixelPosition, MY_CORE, ACTUATOR_COLOR);
    // pixels.show();
  }
  if(forceState)
  {
    writeHallState(forceStateVal);
    resetStateStrip(MY_CORE);
    setStrip(forceStateVal, MY_CORE, ACTUATOR_COLOR);
    mySerialPrint("forced state: ");
    mySerialPrintln(String(forceStateVal));
    // pixels.show();
  }
}

uint32_t getDelay(int rpm, int *stop, int mode)
{
  uint32_t stateDelay;
  if(rpm == 0)
  {
    *stop = 1;
    stateDelay = 0;
  }
  else
  {
    *stop = 0;
    switch (mode)
    {
      case ACTUATOR_VELOCITY_MODE:
        stateDelay = US_PER_MINUTE / (abs(rpm) * actuatorVelocityConfig.countsPerRev);
        break;
      case INTERCEPT_VELOCITY_MODE:
        stateDelay = US_PER_MINUTE / (abs(rpm) * interceptVelocityConfig.countsPerRev);
        break;
      default:
      Serial.println("ERROR: unexpected mode!");
      break;
    }
  }
  return stateDelay;
}

void actuatorVelocityMode(int newMode, int displayOn, int mode)
{
  static int encodingIdx;
  static int direction;
  static int oldRPM;
  static int userRPM;
  static uint32_t userDelay;
  static uint32_t targetTime;
  static int stop;
  uint32_t now;

  if(voltageRequested(mode))
  {
    monitorVoltage(displayOn);
  }

  if(newMode)
  {
    encoder->setPosition(0);
    encodingIdx = 0;
    direction = 0;
    oldRPM = 0;
    userRPM = 0;
    userDelay = 0;
    targetTime = 0;
    stop = 1;
    writeHallState(encodingIdx);
    resetInvalidStates(MY_CORE);
    setStateStrip(encodingIdx, MY_CORE, ACTUATOR_COLOR);
  }
  if((now = time_us_32()) >= targetTime && !stop)
  {
    setStateStrip(encodingIdx, MY_CORE, BLACK);
    encodingIdx = positiveMod(encodingIdx + direction, encodingLength);
    writeHallState(encodingIdx);
    setStateStrip(encodingIdx, MY_CORE, ACTUATOR_COLOR);
    // pixels.show();
    targetTime = now + userDelay;
  }
  userRPM = (encoder->getPosition()) * VELOCITY_RPM_STEP;
  if(userRPM != oldRPM || newMode)
  {
    lastUserInteractionTimeUS = time_us_32();
    userDelay = getDelay(userRPM, &stop, mode);
    targetTime = time_us_32() + userDelay;

    setRing(positiveMod(oldRPM, NUM_RING), MY_CORE, BLACK);
    if(userRPM < 0)
    {
      setRing(positiveMod(userRPM, NUM_RING), MY_CORE, REVERSE_COLOR);
      direction = -1;
    }
    else
    {
      setRing(positiveMod(userRPM, NUM_RING), MY_CORE, FORWARD_COLOR);
      direction = 1;
    }
    displayActuatorVelocity(userRPM, displayOn);
    // pixels.show();
    oldRPM = userRPM;
  }
}
