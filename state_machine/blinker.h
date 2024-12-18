/*
 * DEPRECIATED: does not incorporate new npxManager
 */
#ifndef BLINKER_H
#define BLINKER_H

#include <cstdint>
#include <Adafruit_NeoPixel.h>

struct blinker
{
  int lightsOn;
  unsigned long prevTime;
  Adafruit_NeoPixel *pixels;
  int first;
  int last;
  uint32_t colorOn;
  int onDur;		// microseconds
  uint32_t colorOff;
  int offDur;		// microseconds
};

void update(struct blinker *, uint32_t);

void blink(struct blinker *);
#endif