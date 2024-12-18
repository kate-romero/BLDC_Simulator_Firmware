
#include "shared_data.h"
#include "blinker.h"
#include "button_monitor.h"
#include "my_display.h"
#include "my_neopixels.h"
#include "positive_mod.h"
#include "MCP23S17.h"
#include "actuator_adc_tables.cpp"

#define SMALL_DELAY               10
#define PWR_BTN_PRESS_TIME_US     1000000
#define NINE_MINUTES_US           540000000
#define TEN_MINUTES_US            600000000
#define FIVE_SECONDS_US           5000000
#define ONE_AND_HALF_SECONDS      1500
#define SERIAL_TIMEOUT            1500
#define WARN_DISPLAY_TIME                5000
#define HEARTBEAT_DURATION_US     1000000
#define NPX_UPDATE_DELAY          10000
#define VBAT_SCALE_MULT_MV        1510
#define BATTERY_LOW_THRESH        3300
#define BATTERY_CRITICAL_THRESH   2700
#define START_ACTUATOR_POSITION   1
#define START_ACTUATOR_VELOCITY   2
#define START_MOTOR_ONLY          3
#define START_PASSTHROUGH         4
#define START_INTERCEPT_POSITION  5
#define START_INTERCEPT_VELOCITY  6
#define STOP                      7
#define CLEAR                     8
// #define ACTUATOR_POSITION_MODE    9
// #define ACTUATOR_VELOCITY_MODE    10
// #define MOTOR_ONLY_MODE           11
// #define PASSTHROUGH_MODE          12
// #define INTERCEPT_POSITION_MODE   13
// #define INTERCEPT_VELOCITY_MODE   14
#define STOP_MODE                 15
#define CONFIRM_CLEAR             16
#define CONFIRM_STOP              17
#define SHUTDOWN_SIGNAL           18
// // RP2040 Pins
// #define PWR_BTN_SENSE     8
// #define USB_VBUS_DETECT   10
// #define VBAT_SENSE        26
// // Port Expander Pins
// #define FIVE_V_EN                        0
// #define CONNECT_HALLS_MOTOR_TO_ACTUATOR  1
// #define ENABLE_HALL_PULL_UP              2
// #define FAKE_MOTOR_TEMP_10K              3
// #define FAKE_MOTOR_TEMP_1K               4
// #define VBAT_SENSE_ENABLE                5
// #define HEARTBEAT                        6
// #define CHARGE_SENSE                     8
// #define ENABLE_TEMP_BIAS                 9

static uint32_t npxUpdateTime1 = 0;
      uint32_t npxNow = time_us_32();

int sharedSetup = 0;
int shutdown = 0;
int serialPrint;
uint32_t lastUserInteractionTimeUS;
int currScreen = TWOG_LOGO;
MeanFilter<long> millivoltFilter(12);
MCP23S17 MCP(1, 4, 3, 2);   //  SW SPI   address 0x00
struct blinker greenModeA = {0, 0, &pixels, 0, 31, GREEN, 1000000, (0, 0, 0), 1000000};
struct blinker greenModeB = {0, 0, &pixels, 0, 31, GREEN, 500000, (0, 0, 0), 500000};
struct blinker blueModeA = {0, 0, &pixels, 0, 31, BLUE, 1000000, (0, 0, 0), 1000000};
struct blinker blueModeB = {0, 0, &pixels, 0, 31, BLUE, 500000, (0, 0, 0), 500000};
struct actuator_position_config actuatorPositionConfig = {24, 0, 0, 0, 0, 0, 0};
struct actuator_velocity_config actuatorVelocityConfig = {24, 0, 0, 0, 0};
struct motor_only_config motorOnlyConfig = {24, 0, 0, 0};
struct passthrough_config passthroughConfig = {24, 0};
struct intercept_position_config interceptPositionConfig = {24, 0, 0, 0, 0};
struct intercept_velocity_config interceptVelocityConfig = {24, 0, 0};

int fifo1to0Cnt;
int fifo0to1Cnt;

void setupMCP()
{
  Serial.print("MCP23S17_LIB_VERSION: ");
  Serial.println(MCP23S17_LIB_VERSION);
  delay(100);

  SPI.begin();

  bool b = MCP.begin();
  Serial.println(b ? "true" : "false");
  delay(100);

  MCP.pinMode8(0, 0x00);
  MCP.pinMode8(1, 0x00);
}

int detectPowerOff()
{
  uint32_t detectedTime = time_us_32();
  while(!digitalRead(PWR_BTN_SENSE))
  {
    if(time_us_32() >= detectedTime + PWR_BTN_PRESS_TIME_US)
    {
      // TODO: display logo
      return 1;
    }
    else
    {
      delay(SMALL_DELAY);
    }
  }
  return 0;
}

int convertToMillivolts(int analogVal)
{
  return (analogVal * ANALOG_VOLT_REF * VBAT_SCALE_MULT_MV) / ANALOG_RESOLUTION;
}

int sampleVbat()
{
  static uint32_t sampleTime = time_us_32() + FIVE_SECONDS_US;
  static int batteryLowCount = 0;
  static int batteryCriticalCount = 0;
  static int batteryGoodCount = 0;
  int meanMillivolts;
  uint32_t now;
  int batteryShutdown = 0;
  static int lowTestTime = sampleTime;

  if((now = time_us_32()) >= sampleTime)
  {
    MCP.write1(VBAT_SENSE_ENABLE, 1);
    delay(SMALL_DELAY);
    meanMillivolts = millivoltFilter.AddValue(convertToMillivolts(analogRead(VBAT_SENSE)));
    MCP.write1(VBAT_SENSE_ENABLE, 0);
    sampleTime = now + FIVE_SECONDS_US;

    if(meanMillivolts < BATTERY_CRITICAL_THRESH)
    {
      Serial.println("Battery critical?");
      if(++batteryCriticalCount > 5)
      {
        Serial.println("Confirmed battery critical");
        if(!digitalRead(USB_VBUS_DETECT))    // USB power not plugged in
        {
          batteryShutdown = 1;
        }
        else                                 // USB power plugged in
        {
          displayBatteryCritical();
        }
      }
    }
    else if(meanMillivolts < BATTERY_LOW_THRESH)
    // if(now < lowTestTime)    // debug
    {
      Serial.println("Battery low?");
      if(++batteryLowCount > 5)
      {
        Serial.println("Confirmed battery low");
        displayBatteryLow();
      }
    }
    else if(batteryCriticalCount || batteryLowCount)
    // else   // debug
    {
      Serial.println("Battery good?");
      if(++batteryGoodCount > 2)
      {
        Serial.println("Confirmed battery good");
        batteryCriticalCount = 0;
        batteryLowCount = 0;
        batteryGoodCount = 0;
        displayClearBattery();
      }
    }
  }
  return batteryShutdown;
}

void blinkHeartbeat()
{
  static uint32_t heartbeatChangeTime = 0;
  static int heartbeatOn = 0;
  uint32_t now = time_us_32();

  if(now >= heartbeatChangeTime)
  {
    heartbeatOn = !heartbeatOn;
    MCP.write1(HEARTBEAT, heartbeatOn);
    heartbeatChangeTime = now + HEARTBEAT_DURATION_US;
  }
}

void setup() {
  while(!sharedSetup) delay(10);
  Serial.println("***setting up***");
  while(!shutdown)
  {
    core0Main();
  }
  Serial.println("***shut down***");
}

void setup1(){
  setupMCP();
  MCP.write1(FIVE_V_EN, 1);
  setupDisplay();
  delay(ONE_AND_HALF_SECONDS);
  pinMode(USB_VBUS_DETECT, INPUT);    // no pullup, goes HIGH when USB connected
  if(digitalRead(USB_VBUS_DETECT))
  {
    Serial.begin(9600);
    int serialTimeout = millis() + SERIAL_TIMEOUT;
    while(millis() < serialTimeout)
    {
      if(!Serial) delay(SMALL_DELAY);
    }
    if(!Serial)
    {
      serialPrint = 0;
      currScreen = WARN_NO_SERIAL;
      displayBitmap(currScreen);
      delay(WARN_DISPLAY_TIME);
    }
    else
    {
      serialPrint = 1;    // comment out this line to turn off serial prints while connected to USB
    }
  }
  else
  {
    serialPrint = 0;
  }
  analogReadResolution(12);
  pinMode(PWR_BTN_SENSE, INPUT_PULLUP);
  setupButtons();
  setupNeopixels();
  setupActuatorOnlyMode();
  setupMotorOnlyMode();
  currScreen = MODE_SELECT;
  displayBitmap(currScreen);
  sharedSetup = 1;
  while(!shutdown)
  {
    core1Main();
  }
  delay(ONE_AND_HALF_SECONDS);
  display.clearDisplay();
  display.display();
  delay(100);   // OLED manufacturer recommended
  // wait for power button release
  while(!digitalRead(PWR_BTN_SENSE))
  {
    delay(SMALL_DELAY);
  }
  Serial.println("---shut down---");
  MCP.write1(FIVE_V_EN, 0);   // powers board down when not charging
  // nothing past this should execute unless board is plugged in
  // poll power button
  while(digitalRead(PWR_BTN_SENSE))
  {
    delay(SMALL_DELAY);
  }
  // chip level reset
  watchdog_enable(1, 1);
}

void loop() {}

void core0Main()
{
  static int mode;
  int newRun = 0;
  int clearInvalidCnt = 0;
  // get signal
  if(rp2040.fifo.available())
  {
    mySerialPrint("*** available: ");
    mySerialPrintln(String(rp2040.fifo.available()));
    uint32_t signal;
    if(!rp2040.fifo.pop_nb(&signal))
    {
      Serial.println("ERROR: fifo pop signal");
      while(1) delay(10);
    }
    else
    {
      fifo1to0Cnt--;
      mySerialPrint("FIFO POPPED 0a: ");
      mySerialPrintln(String(fifo1to0Cnt));
    }
    mySerialPrint("*** signal: ");
    mySerialPrintln(String(signal));
    // handle signal: set mode
    switch (signal)
    {
      case START_MOTOR_ONLY:
        mode = MOTOR_ONLY_MODE;
        newRun = 1;
        break;
      case START_INTERCEPT_POSITION:
        mode = INTERCEPT_POSITION_MODE;
        newRun = 1;
        break;
      case START_INTERCEPT_VELOCITY:
        mode = INTERCEPT_VELOCITY_MODE;
        newRun = 1;
        break;
      case START_PASSTHROUGH:
        mode = PASSTHROUGH_MODE;
        newRun = 1;
        break;
      case STOP:
        mode = STOP_MODE;
        break;
      case CLEAR:
        clearInvalidCnt = CLEAR;
        break;
      case SHUTDOWN_SIGNAL:
        return;
      default:
        Serial.print("ERROR: popped illegal signal: ");
        Serial.println(signal);
        while(1) delay(10);
        break;
    }
  }
  if(mode && newRun)                  // debug
  {                                   // debug
    mySerialPrint("*** mode: ");      // debug
    mySerialPrintln(String(mode));    // debug
  }                                   // debug
  // handle mode: run mode
  switch (mode)
  {
    case MOTOR_ONLY_MODE:
      /* FALLTHROUGH */
    case INTERCEPT_POSITION_MODE:
      /* FALLTHROUGH */
    case INTERCEPT_VELOCITY_MODE:
      /* FALLTHROUGH */
    case PASSTHROUGH_MODE:
      motorMode(newRun, clearInvalidCnt, mode);
      break;
    case STOP_MODE:
      mode = 0;
      if(!rp2040.fifo.push_nb(CONFIRM_STOP))
      {
        Serial.println("ERROR: fifo core0 CONFIRM_STOP");
        while(1) delay(10);
      }
      else
      {
        fifo0to1Cnt++;
        mySerialPrint("FIFO PUSHED 0c: ");
        mySerialPrintln(String(fifo0to1Cnt));
      }
      break;
    default:
      break;
  }
}

void loop1() {}

void core1Main()
{
  int newMode = 0;     // set to START_... signal on new mode screen entry
  int forceState = 0;
  int clearInvalidCnt = 0;
  int buttonPressed = buttonMonitor();
  uint32_t idleTimeUS = time_us_32() - lastUserInteractionTimeUS;
  static int userIdle;
  int batteryShutdown = 0;

  blinkHeartbeat();

  // check battery volts
  batteryShutdown = sampleVbat();

  // check idle time
  if(idleTimeUS >= NINE_MINUTES_US)
  {
    userIdle = 1;
    displaySlp();
  }
  else if(userIdle)
  {
    userIdle = 0;
    displayClearSlp();
  }
  // Power Off
  if(detectPowerOff() || idleTimeUS >= TEN_MINUTES_US || batteryShutdown)
  {
    clearStrip();
    clearRing();
    pixels.show();
    MCP.write1(HEARTBEAT, 0);
    displayBitmap(TWOG_LOGO);
    shutdown = 1;
    sendSignal(SHUTDOWN_SIGNAL);
    return;
  }

  // handle button press
  switch (buttonPressed)
  {
    case LEFT_BTN:
      mySerialPrintln(String("caught case LEFT_BTN"));
      int stop;
      handleLeft(&stop);
      stopRunningMode(stop);
      if(stop)
      {
        setHallDriveLow();
        clearStickyStrip();
        clearModePins();
        emptyPrintBuffs(currScreen);
        clearStrip();
        clearRing();
        pixels.show();
      }
      displayBitmap(currScreen);
      loadCheckboxes(currScreen);
      encoder->setPosition(0);
      break;
    case UP_BTN:
      mySerialPrintln(String("caught case UP_BTN"));
      handleUp();
      break;
    case DOWN_BTN:
      mySerialPrintln(String("caught case DOWN_BTN"));
      handleDown();
      break;
    case CENTER_BTN:
      mySerialPrintln(String("caught case CENTER_BTN"));
      handleCenter(&forceState, &clearInvalidCnt);
      break;
    case RIGHT_BTN:
      mySerialPrintln(String("caught case RIGHT_BTN"));
      handleRight(&newMode, &forceState, &clearInvalidCnt);
      setModePins();
      break;
    default:
      // no button press
      break;
  }

  monitorRotary();

  switch (currScreen)
  {
    case ACTUATOR_POSITION:
      actuatorPositionMode(newMode, forceState, 1, ACTUATOR_POSITION_MODE);
      break;
    case ACTUATOR_VELOCITY:
      actuatorVelocityMode(newMode, 1, ACTUATOR_VELOCITY_MODE);
      break;
    case MOTOR_ONLY:
      sendSignal(newMode);
      sendSignal(clearInvalidCnt);      
      receiveConfirmClearSignal();      
      monitorPrintBuffs();
      break;
    case INTERCEPT_POSITION_ACTUATOR:
      sendSignal(newMode);
      sendSignal(clearInvalidCnt);      
      receiveConfirmClearSignal();
      actuatorPositionMode(newMode, forceState, 1, INTERCEPT_POSITION_MODE);
      break;
    case INTERCEPT_POSITION_MOTOR:
      sendSignal(newMode);
      sendSignal(clearInvalidCnt);      
      receiveConfirmClearSignal();
      monitorPrintBuffs();
      actuatorPositionMode(newMode, forceState, 0, INTERCEPT_POSITION_MODE);
      break;
    case INTERCEPT_VELOCITY_ACTUATOR:
      sendSignal(newMode);
      sendSignal(clearInvalidCnt);      
      receiveConfirmClearSignal();
      actuatorVelocityMode(newMode, 1, INTERCEPT_VELOCITY_MODE);
      break;
    case INTERCEPT_VELOCITY_MOTOR:
      sendSignal(newMode);
      sendSignal(clearInvalidCnt);      
      receiveConfirmClearSignal();
      monitorPrintBuffs();
      actuatorVelocityMode(newMode, 0, INTERCEPT_VELOCITY_MODE);
    case PASSTHROUGH:
      sendSignal(newMode);
      sendSignal(clearInvalidCnt);      
      receiveConfirmClearSignal();      
      monitorPrintBuffs();
      break;
    default:
      break;
  }

  updateNeopixels(currScreen);
}

void setHallDriveLow()
{
  digitalWrite(OUT_H1, 0);
  digitalWrite(OUT_H2, 0);
  digitalWrite(OUT_H3, 0);
}

void monitorRotary()
{
  if(selector[currScreen] == 0)
  {
    switch(currScreen)
    {
      case ACTUATOR_POSITION_CONFIG:
        actuatorPositionConfig.countsPerRev += (((int)encoder->getDirection()) * 3);
        displayCountsPerRev(actuatorPositionConfig.countsPerRev);
        displayStateMode(currScreen);
        break;
      case ACTUATOR_VELOCITY_CONFIG:
        actuatorVelocityConfig.countsPerRev += (((int)encoder->getDirection()) * 3);
        displayCountsPerRev(actuatorVelocityConfig.countsPerRev);
        displayStateMode(currScreen);
        break;
      case MOTOR_ONLY_CONFIG:
        motorOnlyConfig.countsPerRev += (((int)encoder->getDirection()) * 3);
        displayCountsPerRev(motorOnlyConfig.countsPerRev);
        displayStateMode(currScreen);
        break;
      case INTERCEPT_POSITION_CONFIG:
        interceptPositionConfig.countsPerRev += (((int)encoder->getDirection()) * 3);
        displayCountsPerRev(interceptPositionConfig.countsPerRev);
        displayStateMode(currScreen);
        break;
      case INTERCEPT_VELOCITY_CONFIG:
        interceptVelocityConfig.countsPerRev += (((int)encoder->getDirection()) * 3);
        displayCountsPerRev(interceptVelocityConfig.countsPerRev);
        displayStateMode(currScreen);
        break;
      case PASSTHROUGH_CONFIG:
        passthroughConfig.countsPerRev += (((int)encoder->getDirection()) * 3);
        displayCountsPerRev(passthroughConfig.countsPerRev);
        displayStateMode(currScreen);
        break;
      default:
        break;
    }
  }
  else if(selector[currScreen] == 1)
  {
    switch(currScreen)
    {
      case ACTUATOR_POSITION_CONFIG:
        actuatorPositionConfig.stripMode = positiveMod(actuatorPositionConfig.stripMode + (int)encoder->getDirection(), 3);
        displayCountsPerRev(actuatorPositionConfig.countsPerRev);
        displayStateMode(currScreen);
        break;
      case ACTUATOR_VELOCITY_CONFIG:
        actuatorVelocityConfig.stripMode = positiveMod(actuatorVelocityConfig.stripMode + (int)encoder->getDirection(), 3);
        displayCountsPerRev(actuatorVelocityConfig.countsPerRev);
        displayStateMode(currScreen);
        break;
      case MOTOR_ONLY_CONFIG:
        motorOnlyConfig.stripMode = positiveMod(motorOnlyConfig.stripMode + (int)encoder->getDirection(), 3);
        displayCountsPerRev(motorOnlyConfig.countsPerRev);
        displayStateMode(currScreen);
        break;
      case INTERCEPT_POSITION_CONFIG:
        interceptPositionConfig.stripMode = positiveMod(interceptPositionConfig.stripMode + (int)encoder->getDirection(), 3);
        displayCountsPerRev(interceptPositionConfig.countsPerRev);
        displayStateMode(currScreen);
        break;
      case INTERCEPT_VELOCITY_CONFIG:
        interceptVelocityConfig.stripMode = positiveMod(interceptVelocityConfig.stripMode + (int)encoder->getDirection(), 3);
        displayCountsPerRev(interceptVelocityConfig.countsPerRev);
        displayStateMode(currScreen);
        break;
      case PASSTHROUGH_CONFIG:
        passthroughConfig.stripMode = positiveMod(passthroughConfig.stripMode + (int)encoder->getDirection(), 3);
        displayCountsPerRev(passthroughConfig.countsPerRev);
        displayStateMode(currScreen);
        break;
      default:
        break;
    }
  }
  if(currScreen == MOTOR_ONLY_CONFIG)
  {
    if(selector[currScreen] == 2)
    {
      motorOnlyConfig.thermInfoIdx = positiveMod((motorOnlyConfig.thermInfoIdx + (int)encoder->getDirection()), 4);
    }
    displayThermType(motorOnlyConfig.thermInfoIdx);
  }
}

void setModePins()
{
  switch (currScreen)
  {
    case ACTUATOR_POSITION:
      MCP.write1(ENABLE_HALL_PULL_UP, 0);
      MCP.write1(CONNECT_HALLS_MOTOR_TO_ACTUATOR, 0);
      MCP.write1(FAKE_MOTOR_TEMP_10K, actuatorPositionConfig.simulate10K);
      MCP.write1(FAKE_MOTOR_TEMP_1K, actuatorPositionConfig.simulate1K);
      MCP.write1(ENABLE_TEMP_BIAS, 0);
      break;
    case ACTUATOR_VELOCITY:
      MCP.write1(ENABLE_HALL_PULL_UP, 0);
      MCP.write1(CONNECT_HALLS_MOTOR_TO_ACTUATOR, 0);
      MCP.write1(FAKE_MOTOR_TEMP_10K, actuatorVelocityConfig.simulate10K);
      MCP.write1(FAKE_MOTOR_TEMP_1K, actuatorVelocityConfig.simulate1K);
      MCP.write1(ENABLE_TEMP_BIAS, 0);
      break;
    case MOTOR_ONLY:
      MCP.write1(ENABLE_HALL_PULL_UP, 1);
      MCP.write1(CONNECT_HALLS_MOTOR_TO_ACTUATOR, 0);
      MCP.write1(FAKE_MOTOR_TEMP_10K, 0);
      MCP.write1(FAKE_MOTOR_TEMP_1K, 0);
      MCP.write1(ENABLE_TEMP_BIAS, motorOnlyConfig.tempSensor);
      break;
    case PASSTHROUGH:
      MCP.write1(ENABLE_HALL_PULL_UP, 0);   // standard op: LOW (pull ups set by external board); no actuator: HIGH
      MCP.write1(CONNECT_HALLS_MOTOR_TO_ACTUATOR, 1);   // standard op: HIGH; no actuator: LOW
      MCP.write1(FAKE_MOTOR_TEMP_10K, 0);
      MCP.write1(FAKE_MOTOR_TEMP_1K, 0);
      MCP.write1(ENABLE_TEMP_BIAS, 0);
      break;
    case INTERCEPT_POSITION_ACTUATOR:
      /* FALLTHROUGH */
    case INTERCEPT_POSITION_MOTOR:
      /* FALLTHROUGH */
    case INTERCEPT_VELOCITY_ACTUATOR:
      /* FALLTHROUGH */
    case INTERCEPT_VELOCITY_MOTOR:
      MCP.write1(ENABLE_HALL_PULL_UP, 1);
      MCP.write1(CONNECT_HALLS_MOTOR_TO_ACTUATOR, 0);
      MCP.write1(FAKE_MOTOR_TEMP_10K, 0);
      MCP.write1(FAKE_MOTOR_TEMP_1K, 0);
      MCP.write1(ENABLE_TEMP_BIAS, 0);
      break;
    default:
      break;
  }
}

void clearModePins()
{
  MCP.write1(ENABLE_HALL_PULL_UP, 0);
  MCP.write1(CONNECT_HALLS_MOTOR_TO_ACTUATOR, 0);
  MCP.write1(FAKE_MOTOR_TEMP_10K, 0);
  MCP.write1(FAKE_MOTOR_TEMP_1K, 0);
  MCP.write1(ENABLE_TEMP_BIAS, 0);
}

void receiveConfirmClearSignal()
{
  if(rp2040.fifo.available())
  {
    uint32_t confirmClear;
    if(!rp2040.fifo.pop_nb(&confirmClear))
    {
      Serial.println("ERROR: fifo pop confirmClear");
      while(1) delay(10);
    }
    else
    {
      fifo1to0Cnt--;
      mySerialPrint("FIFO POPPED confirm clear: ");
      mySerialPrintln(String(fifo1to0Cnt));
      if(confirmClear != CONFIRM_CLEAR)
      {
        Serial.println("ERROR: wrong confirm clear val; check fifo push/pop order");
      }
      else
      {
        delay(100);   // TODO: start timer interrupt, avoid delay?
        select(currScreen, 0);
      }
    }
  }
}

void sendSignal(int signal)
{
  if(signal)
  {
    if(!rp2040.fifo.push_nb(signal))
    {
      Serial.print("ERROR: fifo push signal: ");
      Serial.println(signal);
      while(1) delay(10);
    }
    else
    {
      fifo1to0Cnt++;
      mySerialPrint("FIFO PUSHED 1c: ");
      mySerialPrintln(String(fifo1to0Cnt));
    }
  }
}

void stopRunningMode(int stop)
{
  if(stop)
  {
    if(!rp2040.fifo.push_nb(STOP))
    {
      Serial.println("ERROR: fifo core1 push STOP");
      while(1) delay(10);
    }
    else
    {
      fifo1to0Cnt++;
      mySerialPrint("FIFO PUSHED 1a: ");
      mySerialPrintln(String(fifo1to0Cnt));
      mySerialPrintln(String(rp2040.fifo.available()));
    }
    int caught;
    if((caught = rp2040.fifo.pop()) != CONFIRM_STOP)
    {
      Serial.println("ERROR: fifo core1 pop CONFIRM_STOP");
      Serial.print("received: ");
      Serial.println(caught);
      while(1) delay(10);
    }
    else
    {
      fifo0to1Cnt--;
      mySerialPrint("FIFO POPPED 1b: ");
      mySerialPrintln(String(fifo0to1Cnt));
      mySerialPrintln(String(rp2040.fifo.available()));
    }
  }
}

void getStartingForceState(uint32_t forceState)
{
  if(rp2040.fifo.available())
  {
    if(!rp2040.fifo.pop_nb(&forceState))
    {
      Serial.println("ERROR: fifo pop forceState");
      while(1) delay(10);
    }
    else
    {
      fifo1to0Cnt--;
      mySerialPrint("FIFO POPPED 0b: ");
      mySerialPrintln(String(fifo1to0Cnt));
    }
  }
  else
  {
    forceState = 0;
  }
}

void handleLeft(int *stop) {
  mySerialPrintln(String("Handle Left:"));
  mySerialPrint(String("start currScreen = "));
  mySerialPrintln(String(currScreen));
  // clear selector
  selector[currScreen] = 0;
  // go prev screen
  switch (currScreen)
  {
    case ACTUATOR_SELECT:
      currScreen = MODE_SELECT;
      break;
    case ACTUATOR_POSITION_CONFIG:
      currScreen = ACTUATOR_SELECT;
      break;
    case ACTUATOR_POSITION:
      *stop = STOP;
      currScreen = ACTUATOR_POSITION_CONFIG;
      break;
    case ACTUATOR_VELOCITY_CONFIG:
      currScreen = ACTUATOR_SELECT;
      break;
    case ACTUATOR_VELOCITY:
      *stop = STOP;
      currScreen = ACTUATOR_VELOCITY_CONFIG;
      break;
    case MOTOR_ONLY_CONFIG:
      currScreen = MODE_SELECT;
      break;
    case MOTOR_ONLY:
      *stop = STOP;
      currScreen = MOTOR_ONLY_CONFIG;
      break;
    case INTERCEPT_SELECT:
      currScreen = MODE_SELECT;
      break;
    case INTERCEPT_POSITION_CONFIG:
      currScreen = INTERCEPT_SELECT;
      break;
    case INTERCEPT_VELOCITY_CONFIG:
      currScreen = INTERCEPT_SELECT;
      break;
    case INTERCEPT_POSITION_ACTUATOR:
      *stop = STOP;
      currScreen = INTERCEPT_POSITION_CONFIG;
      break;
    case INTERCEPT_POSITION_MOTOR:
      *stop = STOP;
      currScreen = INTERCEPT_POSITION_CONFIG;
      break;
    case INTERCEPT_VELOCITY_ACTUATOR:
      *stop = STOP;
      currScreen = INTERCEPT_VELOCITY_CONFIG;
      break;
    case INTERCEPT_VELOCITY_MOTOR:
      *stop = STOP;
      currScreen = INTERCEPT_VELOCITY_CONFIG;
      break;
    case PASSTHROUGH_CONFIG:
      currScreen = MODE_SELECT;
      break;
    case PASSTHROUGH:
      *stop = STOP;
      currScreen = PASSTHROUGH_CONFIG;
      break;
    default:
      break;
  }
  mySerialPrint(String("end currScreen = "));
  mySerialPrintln(String(currScreen));
}

void handleUp() {
  mySerialPrintln(String("Handling Up:"));
  // mySerialPrint(String("start selector = "));
  // mySerialPrintln(String(selector[currScreen]));
  switch (currScreen)
  {
    // move hover up
    case ACTUATOR_SELECT:
      /* FALLTHROUGH */
    case ACTUATOR_POSITION:
      /* FALLTHROUGH */
    case ACTUATOR_POSITION_CONFIG:
      /* FALLTHROUGH */
    case ACTUATOR_VELOCITY_CONFIG:
      /* FALLTHROUGH */
    case MOTOR_ONLY_CONFIG:
      /* FALLTHROUGH */
    case INTERCEPT_SELECT:
      /* FALLTHROUGH */
    case INTERCEPT_POSITION_CONFIG:
      /* FALLTHROUGH */
    case INTERCEPT_VELOCITY_CONFIG:
      /* FALLTHROUGH */
    case PASSTHROUGH_CONFIG:
      /* FALLTHROUGH */
    case MODE_SELECT:
      hover(currScreen, 0);
      selector[currScreen] = positiveMod(selector[currScreen] - 1, optionCnt[currScreen]);
      if(selector[currScreen] > ITEMS_PER_SCREEN && selector[currScreen]  == optionCnt[currScreen] - 1)
      {
        Serial.print("SELECTOR: ");
        Serial.println(selector[currScreen]);
        scroll(currScreen, ITEMS_PER_SCREEN - optionCnt[currScreen]);
        Serial.print("OPTION COUNT: ");
        Serial.println(optionCnt[currScreen]);
      }
      else if(selector[currScreen] * -12 > topY[currScreen])  // selector is at or above top of screen
      {
        scroll(currScreen, 1);
      }
      hover(currScreen, 1);
      break;
    case INTERCEPT_POSITION_ACTUATOR:
      Serial.print("Start Screen: ");
      Serial.println(currScreen);
      Serial.print("Start Selector: ");
      Serial.println(selector[currScreen]);
      hover(currScreen, 0);
      selector[currScreen] = (selector[currScreen] + 1) % optionCnt[currScreen];
      if(selector[currScreen] == optionCnt[currScreen] - 1)
      {
        selector[currScreen] = 0;
        currScreen = INTERCEPT_POSITION_MOTOR;
        displayBitmap(currScreen);
      }
      else
      {
        hover(currScreen, 1);
      }
      Serial.print("End Screen: ");
      Serial.println(currScreen);
      Serial.print("End Selector: ");
      Serial.println(selector[currScreen]);
      break;
    case INTERCEPT_POSITION_MOTOR:
      Serial.print("Start Screen: ");
      Serial.println(currScreen);
      currScreen = INTERCEPT_POSITION_ACTUATOR;
      displayPartBitmap();
      Serial.print("End Screen: ");
      Serial.println(currScreen);
      break;
    case INTERCEPT_VELOCITY_ACTUATOR:
      Serial.print("Start Screen: ");
      Serial.println(currScreen);
      currScreen = INTERCEPT_VELOCITY_MOTOR;
      displayBitmap(currScreen);
      Serial.print("End Screen: ");
      Serial.println(currScreen);
      break;
    case INTERCEPT_VELOCITY_MOTOR:
      Serial.print("Start Screen: ");
      Serial.println(currScreen);
      currScreen = INTERCEPT_VELOCITY_ACTUATOR;
      displayPartBitmap();
      Serial.print("End Screen: ");
      Serial.println(currScreen);
      break;
    default:
      break;
  }
  // mySerialPrint(String("end selector = "));
  // mySerialPrintln(String(selector[currScreen]));
}

void handleDown() {
  mySerialPrintln(String("Handling Down:"));
  mySerialPrint(String("start selector = "));
  mySerialPrintln(String(selector[currScreen]));
  switch (currScreen)
  {
    // move hover up
    case ACTUATOR_SELECT:
      /* FALLTHROUGH */
    case ACTUATOR_POSITION:
      /* FALLTHROUGH */
    case ACTUATOR_POSITION_CONFIG:
      /* FALLTHROUGH */
    case ACTUATOR_VELOCITY_CONFIG:
      /* FALLTHROUGH */
    case MOTOR_ONLY_CONFIG:
      /* FALLTHROUGH */
    case INTERCEPT_SELECT:
      /* FALLTHROUGH */
    case INTERCEPT_POSITION_CONFIG:
      /* FALLTHROUGH */
    case INTERCEPT_VELOCITY_CONFIG:
      /* FALLTHROUGH */
    case PASSTHROUGH_CONFIG:
      /* FALLTHROUGH */
    case MODE_SELECT:
      hover(currScreen, 0);
      selector[currScreen] = (selector[currScreen] + 1) % optionCnt[currScreen];
      if(optionCnt[currScreen] > ITEMS_PER_SCREEN && selector[currScreen]  == 0)
      {
        topY[currScreen] = 0;
        scroll(currScreen, 0);
      }
      else if(selector[currScreen] > 2 && selector[currScreen] < optionCnt[currScreen] - 1)   // selector is at bottom of screen
      {
        scroll(currScreen, -1);
      }
      hover(currScreen, 1);
      break;
    case INTERCEPT_POSITION_ACTUATOR:
      Serial.print("Start Screen: ");
      Serial.println(currScreen);
      Serial.print("Start Selector: ");
      Serial.println(selector[currScreen]);
      hover(currScreen, 0);
      selector[currScreen] = (selector[currScreen] + 1) % optionCnt[currScreen];
      Serial.print("Start Selector + 1: ");
      Serial.println(selector[currScreen]);
      if(selector[currScreen] == optionCnt[currScreen] - 1)
      {
        Serial.println("Changing screen");
        Serial.print("Option Count: ");
        Serial.println(optionCnt[currScreen]);
        selector[currScreen] = 0;
        currScreen = INTERCEPT_POSITION_MOTOR;
        displayBitmap(currScreen);
      }
      else
      {
        hover(currScreen, 1);
      }
      Serial.print("End Screen: ");
      Serial.println(currScreen);
      Serial.print("End Selector: ");
      Serial.println(selector[currScreen]);
      break;
    case INTERCEPT_POSITION_MOTOR:
      Serial.print("Start Screen: ");
      Serial.println(currScreen);
      currScreen = INTERCEPT_POSITION_ACTUATOR;
      displayPartBitmap();
      Serial.print("End Screen: ");
      Serial.println(currScreen);
      break;
    case INTERCEPT_VELOCITY_ACTUATOR:
      Serial.print("Start Screen: ");
      Serial.println(currScreen);
      currScreen = INTERCEPT_VELOCITY_MOTOR;
      displayBitmap(currScreen);
      Serial.print("End Screen: ");
      Serial.println(currScreen);
      break;
    case INTERCEPT_VELOCITY_MOTOR:
      Serial.print("Start Screen: ");
      Serial.println(currScreen);
      currScreen = INTERCEPT_VELOCITY_ACTUATOR;
      displayPartBitmap();
      Serial.print("End Screen: ");
      Serial.println(currScreen);
      break;
    default:
      break;
  }
  mySerialPrint(String("end selector = "));
  mySerialPrintln(String(selector[currScreen]));
}

void handleCenter(int *forceState, int *clearInvalidCnt) {
  mySerialPrintln(String("Handling Center:"));
  switch (currScreen)
  {
    case ACTUATOR_SELECT:
      centerActuatorSelect();                // go next screen
      break;
    case MODE_SELECT:
      centerModeSelect();                    // go next screen
      break;
    case INTERCEPT_SELECT:
      centerInterceptSelect();               // go next screen              
      break;
    case INTERCEPT_POSITION_ACTUATOR:
      /* FALLTHROUGH */
    case ACTUATOR_POSITION:
      centerActuatorPosition(forceState);    // force state
      break;
    case MOTOR_ONLY:
      /* FALLTHROUGH */
    case INTERCEPT_POSITION_MOTOR:
      /* FALLTHROUGH */
    case INTERCEPT_VELOCITY_MOTOR:
      /* FALLTHROUGH */
    case PASSTHROUGH:
      centerMotorRunning(clearInvalidCnt);       // clear invalid state count
      break;
    case ACTUATOR_POSITION_CONFIG:
      centerActuatorPositionConfig(forceState);  // toggle configs
      break;
    case ACTUATOR_VELOCITY_CONFIG:     // TODO: spinner
      centerActuatorVelocityConfig();            // toggle configs
      break;
    case MOTOR_ONLY_CONFIG:            // TODO: spinner
      centerMotorOnlyConfig();                   // toggle configs
      break;
    case INTERCEPT_POSITION_CONFIG:    // TODO: spinner
      centerInterceptPositionConfig();           // toggle configs
      break;
    case INTERCEPT_VELOCITY_CONFIG:    // TODO: spinner
      centerInterceptVelocityConfig();           // toggle configs
      break;
    case PASSTHROUGH_CONFIG:           // TODO: spinner
      centerPassthroughConfig();                // toggle configs
      break;
    default:
      break;
  }
}

void handleRight(int *newMode, int *forceState, int *clearInvalidCnt) {
  switch (currScreen)
  {
    case ACTUATOR_SELECT:
      encoder->setPosition(0);
      centerActuatorSelect();               // go next screen
      break;
    case MODE_SELECT:
      encoder->setPosition(0);
      centerModeSelect();                 // go next screen
      break;
    case INTERCEPT_SELECT:
      encoder->setPosition(0);
      centerInterceptSelect();               // go next screen              
      break;
    case ACTUATOR_POSITION:
      /* FALLTHROUGH */
    case ACTUATOR_VELOCITY:
      /* FALLTHROUGH */
    case MOTOR_ONLY:
      /* FALLTHROUGH */
    case INTERCEPT_POSITION_ACTUATOR:
      /* FALLTHROUGH */
    case INTERCEPT_POSITION_MOTOR:
      /* FALLTHROUGH */
    case INTERCEPT_VELOCITY_ACTUATOR:
      /* FALLTHROUGH */
    case INTERCEPT_VELOCITY_MOTOR:
      /* FALLTHROUGH */
    case PASSTHROUGH:
      clearStickyStrip();
      break;
    case ACTUATOR_POSITION_CONFIG:
      selector[currScreen] = 0;
      currScreen = ACTUATOR_POSITION;     // go mode screen
      displayPartBitmap();
      *newMode = START_ACTUATOR_POSITION;
      break;
    case ACTUATOR_VELOCITY_CONFIG:
      selector[currScreen] = 0;
      currScreen = ACTUATOR_VELOCITY;     // go mode screen
      displayPartBitmap();
      *newMode = START_ACTUATOR_VELOCITY;
      break;
    case MOTOR_ONLY_CONFIG:
      selector[currScreen] = 0;
      currScreen = MOTOR_ONLY;            // go mode screen
      displayPartBitmap();
      *newMode = START_MOTOR_ONLY;
      break;
    case INTERCEPT_POSITION_CONFIG:
      selector[currScreen] = 0;
      currScreen = INTERCEPT_POSITION_ACTUATOR;   // go mode screen
      displayPartBitmap();
      *newMode = START_INTERCEPT_POSITION;
      break;
    case INTERCEPT_VELOCITY_CONFIG:
      selector[currScreen] = 0;
      currScreen = INTERCEPT_VELOCITY_ACTUATOR;   // go mode screen
      displayPartBitmap();
      *newMode = START_INTERCEPT_VELOCITY;
      break;
    case PASSTHROUGH_CONFIG:
      selector[currScreen] = 0;
      currScreen = PASSTHROUGH;                   // go mode screen
      displayBitmap(currScreen);
      *newMode = START_PASSTHROUGH;
      break;
    default:
      break;
  }
}

void displayPartBitmap()
{
  displayBitmap(currScreen);
  switch (currScreen)
  {
    case ACTUATOR_POSITION:
      if(actuatorPositionConfig.measureVoltage)
      {
        displayVoltageBitmap();
      }
      break;
    case ACTUATOR_VELOCITY:
      if(actuatorVelocityConfig.measureVoltage)
      {
        displayVoltageBitmap();
      }
      break;
    case  MOTOR_ONLY:
      if(motorOnlyConfig.tempSensor)
        {
          displayTempBitmap();
        }
      break;
    case INTERCEPT_POSITION_ACTUATOR:
      if(interceptPositionConfig.measureVoltage)
      {
        displayVoltageBitmap();
      }
      break;
    case INTERCEPT_VELOCITY_ACTUATOR:
      if(interceptVelocityConfig.measureVoltage)
      {
        displayVoltageBitmap();
      }
      break;
    default:
			Serial.print("ERROR: unexpected case displayModeBitmap: ");
      Serial.println(currScreen);
			while(1) delay(10);
      break;
  }
}

// go next screen
void centerActuatorSelect()
{
  mySerialPrint(String("start currScreen = "));
  mySerialPrintln(String(currScreen));
  int saveSelector = selector[currScreen];
  selector[currScreen] = 0;
	switch (saveSelector)
	{
		case 0:
			currScreen = ACTUATOR_POSITION_CONFIG;
			break;
		case 1:
			currScreen = ACTUATOR_VELOCITY_CONFIG;
			break;
		default:
			Serial.println("ERROR: handle center case actuator select");
			while(1) delay(10);
      break;
	}
  mySerialPrint(String("end currScreen = "));
  mySerialPrintln(String(currScreen));
  displayBitmap(currScreen);
  loadCheckboxes(currScreen);
}

// go next screen
void centerModeSelect()
{
  mySerialPrint(String("*start currScreen = "));
  mySerialPrintln(String(currScreen));
	int saveSelector = selector[currScreen];
  selector[currScreen] = 0;
	switch (saveSelector)
	{
		case 0:
			currScreen = ACTUATOR_SELECT;
			break;
		case 1:
			currScreen = MOTOR_ONLY_CONFIG;
			break;
		case 2:
			currScreen = PASSTHROUGH_CONFIG;
			break;
		case 3:
			currScreen = INTERCEPT_SELECT;
			break;
		default:
			Serial.println("ERROR: handle center case mode select");
			while(1) delay(10);
      break;
	}
  mySerialPrint(String("*end currScreen = "));
  mySerialPrintln(String(currScreen));
  displayBitmap(currScreen);
  loadCheckboxes(currScreen);
}

// go next screen
void centerInterceptSelect()
{
  mySerialPrint(String("start currScreen = "));
  mySerialPrintln(String(currScreen));
  int saveSelector = selector[currScreen];
  selector[currScreen] = 0;
	switch (saveSelector)
	{
		case 0:
			currScreen = INTERCEPT_POSITION_CONFIG;
			break;
		case 1:
			currScreen = INTERCEPT_VELOCITY_CONFIG;
			break;
		default:
			Serial.println("ERROR: handle center case intercept select");
			while(1) delay(10);
      break;
	}
  mySerialPrint(String("end currScreen = "));
  mySerialPrintln(String(currScreen));
  displayBitmap(currScreen);
  loadCheckboxes(currScreen);
}

// force state
void centerActuatorPosition(int *forceState)
{
  select(currScreen, 1);
  delay(100); // TODO: start timer interrupt, avoid delay?
  select(currScreen, 0);
  mySerialPrint(String("Selected force state: "));
	switch (selector[currScreen])
	{
		case 0:
      mySerialPrintln("0");
			*forceState = FORCE_ZERO;
			break;
		case 1:
    mySerialPrintln("7");
			*forceState = FORCE_SEVEN;
			break;
    default:
			Serial.println("ERROR: unexpected case centerActuatorPosition");
			while(1) delay(10);
      break;
	}
}

// clear invalid counter
void centerMotorRunning(int *clearInvalidCnt)
{
  mySerialPrintln(String("Send clear invalid counter"));
  select(currScreen, 1);
  delay(100); // TODO: start timer interrupt, avoid delay?
  select(currScreen, 0);
	*clearInvalidCnt = CLEAR;
}

// toggle configs
void centerActuatorPositionConfig(int *forceState)
{
  mySerialPrintln(String("Toggle actuator position configs:"));
	switch (selector[currScreen])
	{
		case 2:
			actuatorPositionConfig.simulate1K = 0;
      fillCheckbox(currScreen, 1, 0);
			actuatorPositionConfig.simulate10K = !actuatorPositionConfig.simulate10K;
			toggle(currScreen, actuatorPositionConfig.simulate10K);
      mySerialPrint(String("simulate 10K: "));
      mySerialPrintln(String(actuatorPositionConfig.simulate10K));
			break;
		case 3:
			actuatorPositionConfig.simulate10K = 0;
			fillCheckbox(currScreen, 0, 0);
			actuatorPositionConfig.simulate1K = !actuatorPositionConfig.simulate1K;
			toggle(currScreen, actuatorPositionConfig.simulate1K);
      mySerialPrint(String("simulate 1K: "));
      mySerialPrintln(String(actuatorPositionConfig.simulate1K));
			break;
		case 4:
			actuatorPositionConfig.measureVoltage = !actuatorPositionConfig.measureVoltage;
			toggle(currScreen, actuatorPositionConfig.measureVoltage);
      mySerialPrint(String("measure voltage: "));
      mySerialPrintln(String(actuatorPositionConfig.measureVoltage));
			break;
		case 5:
			actuatorPositionConfig.forceStartState7 = 0;
			fillCheckbox(currScreen, 4, 0);
			actuatorPositionConfig.forceStartState0 = !actuatorPositionConfig.forceStartState0;
			toggle(currScreen, actuatorPositionConfig.forceStartState0);
			if(actuatorPositionConfig.forceStartState0)
			{
				*forceState = FORCE_ZERO;
			}
      mySerialPrint(String("force state 0"));
      mySerialPrintln(String(actuatorPositionConfig.forceStartState0));
			break;
		case 6:
			actuatorPositionConfig.forceStartState0 = 0;
			fillCheckbox(currScreen, 3, 0);
			actuatorPositionConfig.forceStartState7 = !actuatorPositionConfig.forceStartState7;
			toggle(currScreen, actuatorPositionConfig.forceStartState7);
			if(actuatorPositionConfig.forceStartState7)
			{
				*forceState = FORCE_SEVEN;
			}
      mySerialPrint(String("force state 7"));
      mySerialPrintln(String(actuatorPositionConfig.forceStartState7));
			break;
    default:
			Serial.println("ERROR: unexpected case centerActuatorPositionConfig");
			while(1) delay(10);
      break;
	}
}

// toggle configs
void centerActuatorVelocityConfig()
{
  mySerialPrintln(String("Toggle actuator velocity configs:"));
	switch (selector[currScreen])
	{
		case 2:
			actuatorVelocityConfig.simulate1K = 0;
			fillCheckbox(currScreen, 2, 0);
			actuatorVelocityConfig.simulate10K = !actuatorVelocityConfig.simulate10K;
			toggle(currScreen, actuatorVelocityConfig.simulate10K);
      mySerialPrint(String("simulate 10K: "));
      mySerialPrintln(String(actuatorVelocityConfig.simulate10K));
			break;
		case 3:
			actuatorVelocityConfig.simulate10K = 0;
			fillCheckbox(currScreen, 1, 0);
			actuatorVelocityConfig.simulate1K = !actuatorVelocityConfig.simulate1K;
			toggle(currScreen, actuatorVelocityConfig.simulate1K);
      mySerialPrint(String("simulate 1K: "));
      mySerialPrintln(String(actuatorVelocityConfig.simulate1K));
			break;
		case 4:
			actuatorVelocityConfig.measureVoltage = !actuatorVelocityConfig.measureVoltage;
			toggle(currScreen, actuatorVelocityConfig.measureVoltage);
      mySerialPrint(String("measure voltage:"));
      mySerialPrintln(String(actuatorVelocityConfig.measureVoltage));
			break;
    default:
			Serial.println("ERROR: unexpected case centerActuatorVelocityConfig");
			while(1) delay(10);
      break;
	}
}

// toggle configs
void centerMotorOnlyConfig()
{
  mySerialPrintln(String("Toggle motor only configs:"));
  switch (selector[currScreen])
  {
    case 3:
      motorOnlyConfig.tempSensor = !motorOnlyConfig.tempSensor;
      toggle(currScreen, motorOnlyConfig.tempSensor);
      mySerialPrint(String("temp sensor:"));
      mySerialPrintln(String(motorOnlyConfig.tempSensor));
      break;
    default:
      Serial.println("ERROR: unexpected case centermotorOnlyConfig");
			while(1) delay(10);
      break;
  }
}

// toggle configs
void centerInterceptPositionConfig()
{
  switch (selector[currScreen])
  {
    case 2:
      interceptPositionConfig.measureVoltage = !interceptPositionConfig.measureVoltage;
      toggle(currScreen, interceptPositionConfig.measureVoltage);
      break;
    case 3:
      interceptPositionConfig.forceStartState0 = !interceptPositionConfig.forceStartState0;
      toggle(currScreen, interceptPositionConfig.forceStartState0);
      break;
    case 4:
      interceptPositionConfig.forceStartState7 = !interceptPositionConfig.forceStartState7;
      toggle(currScreen, interceptPositionConfig.forceStartState7);
    default:
      Serial.println("ERROR: unexpected case centerInterceptPositionConfig");
			while(1) delay(10);
      break;
  }
}

// toggle configs
void centerInterceptVelocityConfig()
{
  switch (selector[currScreen])
  {
    case 2:
      interceptVelocityConfig.measureVoltage = !interceptVelocityConfig.measureVoltage;
      toggle(currScreen, interceptVelocityConfig.measureVoltage);
      break;
    default:
      Serial.println("ERROR: unexpected case centerInterceptVelocityConfig");
			while(1) delay(10);
      break;
  }
}

// toggle configs (no checkboxes)
void centerPassthroughConfig()
{
}
