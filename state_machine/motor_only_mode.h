#ifndef MOTOR_ONLY_MODE_H
#define MOTOR_ONLY_MODE_H

#include "shared_data.h"
#include "MeanFilterLib.h"
#include "actuator_adc_tables.cpp"

struct motor_only_config
{
  int countsPerRev;

  int tempSensor;

  int thermInfoIdx;

  int stripMode;
};

struct passthrough_config
{
  int countsPerRev;

  int stripMode;
};

extern MeanFilter<long> motorMeanFilter;
extern MeanFilter<long> passthroughMeanFilter;
extern MeanFilter<long> interceptPositionFilter;
extern MeanFilter<long> interceptVelocityMeanFilter;
extern const _thermistor_info* thermInfoPtrs[];

void setupMotorOnlyMode();

int readHallState();

int getStartStateIdx(int);

int getHallRotationDirection(int, int);

int convertToPixelPosition(int);

int calculateRPM(int, int);

int tempRequested(int);

void monitorTemp(int);

void motorMode(int, int, int);

#endif