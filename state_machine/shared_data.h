#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>
#include <cstdint>
#include <Adafruit_NeoPixel.h>
#include <RotaryEncoder.h>
#include <Adafruit_SH110X.h>
#include "my_print.h"
#include "my_neopixels.h"
// TODO: remove after structs are moved
#include "actuator_only_mode.h"
#include "motor_only_mode.h"

#define BLACK   			0x000000
#define RED						0xFF0000
#define GREEN         0x00FF00
#define BLUE          0x0000FF
#define NUM_RING			24
#define ENC_PIN1      22
#define ENC_PIN2      20
#define FORCE_ZERO    1
#define FORCE_SEVEN   2
#define ACTUATOR_POSITION_MODE    9
#define ACTUATOR_VELOCITY_MODE    10
#define MOTOR_ONLY_MODE           11
#define PASSTHROUGH_MODE          12
#define INTERCEPT_POSITION_MODE   13
#define INTERCEPT_VELOCITY_MODE   14
#define ANALOG_VOLT_REF           3
#define ANALOG_RESOLUTION         4096
// RP2040 Pins
#define PWR_BTN_SENSE     8
#define USB_VBUS_DETECT   10
#define VBAT_SENSE        26
#define HALL_POWER_SENSE  27
#define MOTOR_TEMP_SENSE  28
// Port Expander Pins
#define FIVE_V_EN                        0
#define CONNECT_HALLS_MOTOR_TO_ACTUATOR  1
#define ENABLE_HALL_PULL_UP              2
#define FAKE_MOTOR_TEMP_10K              3
#define FAKE_MOTOR_TEMP_1K               4
#define VBAT_SENSE_ENABLE                5
#define HEARTBEAT                        6
#define CHARGE_SENSE                     8  // complement?
#define ENABLE_TEMP_BIAS                 9

extern Adafruit_NeoPixel pixels;

extern RotaryEncoder *encoder;

extern Adafruit_SH1106G display;

extern int encoding[];

extern int encodingLength;

// TODO: put in control loop .h file
struct intercept_position_config
{
  int countsPerRev;

  int measureVoltage;

  int forceStartState0;

  int forceStartState7;

  int stripMode;
};

struct intercept_velocity_config
{
  int countsPerRev;

  int measureVoltage;

  int stripMode;
};

extern struct actuator_position_config actuatorPositionConfig;

extern struct actuator_velocity_config actuatorVelocityConfig;

extern struct motor_only_config motorOnlyConfig;

extern struct passthrough_config passthroughConfig;

extern struct intercept_position_config interceptPositionConfig;

extern struct intercept_velocity_config interceptVelocityConfig;

extern uint32_t lastUserInteractionTimeUS;

// extern const _thermistor_info therm_info_std;

// extern const _thermistor_info therm_info_std_3v3;

// extern const _thermistor_info therm_info_koford;

// extern const _thermistor_info therm_info_kollmorgen;

#endif