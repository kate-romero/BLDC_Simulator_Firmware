#ifndef MY_NEOPIXELS_H
#define MY_NEOPIXELS_H
#include <cstdint>

#define ON_DUR          250000
#define OFF_DUR         250000
#define WARNING_DUR     2500000

void setupNeopixels();

void clearStrip();

void clearRing();

void fillRing(int, uint32_t);

void setStrip(int, int, uint32_t);

void setRing(int, int, uint32_t);

void resetStateStrip(int);

void resetInvalidStates(int);

void setStateStrip(int, int, uint32_t);

void updateNeopixels(int);

void standardStripMode();

uint32_t dimColorDiv(uint32_t, uint32_t);

uint32_t dimColorSub(uint32_t);

void clearStickyStrip();

void stickyStripMode();

void fadeStripMode();

// void neopixelBlinkWarn();

#endif