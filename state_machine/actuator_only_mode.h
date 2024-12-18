#ifndef ACTUATOR_ONLY_MODE_H
#define ACTUATOR_ONLY_MODE_H

#include "shared_data.h"

#define OUT_H1			    19
#define OUT_H2			    21
#define OUT_H3			    23

struct actuator_position_config
{
  int countsPerRev;
  int simulate10K;
  int simulate1K;
  int measureVoltage;
  int forceStartState0;
  int forceStartState7;
  int stripMode;
};

struct actuator_velocity_config
{
  int countsPerRev;
  int simulate10K;
  int simulate1K;
  int measureVoltage;
  int stripMode;
};

void setupActuatorOnlyMode();

void setActuatorOnlyPins();

void checkPosition();

void writeHallState(int);

void actuatorPositionMode(int, int, int, int);

uint32_t getDelay(int, int *, int);

int voltageRequested(int);

void monitorVoltage(int);

void actuatorVelocityMode(int, int, int);

extern RotaryEncoder *encoder;

#endif