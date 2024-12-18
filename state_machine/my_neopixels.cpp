#include "shared_data.h"
#include "my_neopixels.h"
#include "my_display.h"
#include "shared_data.h"
#include "positive_mod.h"
#include <queue>
// #include <stdlib.h>
// #include "blinker.h"

#define NPIX_PIN      15
#define NUM_PIXELS    32
#define BRIGHTNESS		10
#define NUM_RING			24
#define NUM_STRIP			8
#define RING_START		8
#define NPX_UPDATE_DELAY    3
#define STANDARD_STRIP_MODE   0
#define STICKY_STRIP_MODE     1
#define FADE_STRIP_MODE       2
#define FULL_BRIGHT_DIV       1
#define HALF_BRIGHT_DIV       2
#define DIM_SUB               5   // higher value: faster fade (and vice versa)

Adafruit_NeoPixel pixels(NUM_PIXELS, NPIX_PIN, NEO_GRB + NEO_KHZ800);

uint32_t npxManager[32][2];
int stickyHasBeenSet[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint32_t stickyBrightnessDiv[8] = {1, 1, 1, 1, 1, 1, 1, 1};
uint32_t stickyColor[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint32_t fadeColor[8] = {0, 0, 0, 0, 0, 0, 0, 0};

std::queue<int> pixelCore0Buff;

void setupNeopixels()
{
  for(int i = 0; i < 32; i++)
  {
    for(int j = 0; j < 2; j++)
    {
      npxManager[i][j] = BLACK;
    }
  }
  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  pixels.clear();
  pixels.show();
}

// should only be called by core1; clears strip and npxManager values for BOTH cores
void clearStrip()
{
	for(int i = 0; i < RING_START; i++)
	{
    for(int j = 0; j < 2; j++)
    {
      npxManager[i][j] = BLACK;
    }
    pixels.setPixelColor(i, BLACK);
	}
}

// should only be called by core1; clears ring and npxManager values for BOTH cores
void clearRing()
{
	for(int i = RING_START; i < NUM_PIXELS; i++)
	{
    for(int j = 0; j < 2; j++)
    {
      npxManager[i][j] = BLACK;
    }
    pixels.setPixelColor(i, BLACK);
	}
}

void fillRing(int core, uint32_t color) 
{
	for(int i = RING_START; i < NUM_PIXELS; i++)
	{
    npxManager[i][core] = color;
	}
}

void setStrip(int position, int core, uint32_t color)
{
  npxManager[position][core] = color;
}

void setRing(int position, int core, uint32_t color)
{
  npxManager[position + RING_START][core] = color;
}

void resetStateStrip(int core)
{
  clearStrip();
  setStrip(0, core, RED);
  setStrip(7, core, RED);
}

void resetInvalidStates(int core)
{
  setStrip(0, core, RED);
  setStrip(7, core, RED);
}

void setStateStrip(int idx, int core, uint32_t color)
{
  npxManager[idx + 1][core] = color;
}

// should only be called by core1
void updateNeopixels(int currScreen)
{
  static uint32_t lastMsTime = 0;
  uint32_t now = millis();
  if(now % NPX_UPDATE_DELAY == 0 && now != lastMsTime)    // if flashing random pixels, increase NPX_UPDATE_DELAY
  {
    lastMsTime = now;
    for(int i = RING_START; i < NUM_PIXELS; i++)
    {
      pixels.setPixelColor(i, (npxManager[i][0] | npxManager[i][1]));
    }
    int stripMode = STANDARD_STRIP_MODE;
    switch (currScreen)
    {
      case ACTUATOR_POSITION:
        stripMode = actuatorPositionConfig.stripMode;
        break;
      case ACTUATOR_VELOCITY:
        stripMode = actuatorVelocityConfig.stripMode;
        break;
      case MOTOR_ONLY:
        stripMode = motorOnlyConfig.stripMode;
        break;
      case INTERCEPT_POSITION_ACTUATOR:
        /* FALLTHROUGH */
      case INTERCEPT_POSITION_MOTOR:
        stripMode = interceptPositionConfig.stripMode;
        break;
      case INTERCEPT_VELOCITY_ACTUATOR:
        /* FALLTHROUGH */
      case INTERCEPT_VELOCITY_MOTOR:
        stripMode = interceptVelocityConfig.stripMode;
        break;
      case PASSTHROUGH:
        stripMode = passthroughConfig.stripMode;
        break;
      default:
        break;
    }
    switch (stripMode)
    {
      case STANDARD_STRIP_MODE:
        standardStripMode();
        break;
      case STICKY_STRIP_MODE:
        stickyStripMode();
        break;
      case FADE_STRIP_MODE:
        fadeStripMode();
        break;
    }
    pixels.show();
  }
}

void standardStripMode()
{
  for(int i = 0; i < RING_START; i++)
  {
    pixels.setPixelColor(i, (npxManager[i][0] | npxManager[i][1]));
  }
}

uint32_t dimColorDiv(uint32_t color, uint32_t brightnessDiv)
{
  int red = (color & 0xFF0000) >> 16;
  int green = (color & 0x00FF00) >> 8;
  int blue = color & 0x0000FF;
  red = red / brightnessDiv;
  green = green / brightnessDiv;
  blue = blue / brightnessDiv;
  return ((red << 16) | (green << 8) | blue);
}

uint32_t dimColorSub(uint32_t color/*, uint32_t brightnessSub*/)
{
  Serial.print("Color: ");
  Serial.println(color);
  // Serial.print("brightness sub: ");
  // Serial.println(brightnessSub);
  int red = (color & 0xFF0000) >> 16;
  Serial.println(red);
  int green = (color & 0x00FF00) >> 8;
  Serial.println(green);
  int blue = color & 0x0000FF;
  Serial.println(blue);
  Serial.println("subtracting:");
  if(red >= DIM_SUB)
  {
    red -= DIM_SUB;
  }
  else
  {
    red = 0;
  }
  Serial.println(red);
  if(green >= DIM_SUB)
  {
    green -= DIM_SUB;
  }
  else
  {
    green = 0;
  }
  Serial.println(green);
  if(blue >= DIM_SUB)
  {
    blue -= DIM_SUB;
  }
  else
  {
    blue = 0;
  }
  Serial.println(blue);
  Serial.print("dim color: ");
  Serial.println(((red << 16) | (green << 8) | blue));
  return ((red << 16) | (green << 8) | blue);
}

void clearStickyStrip()
{
  for(int i = 0; i < RING_START; i++)
  {
    stickyHasBeenSet[i] = 0;
    stickyColor[i] = 0;
  }
}

void stickyStripMode()
{
  for(int i = 0; i < RING_START; i++)
  {
    uint32_t pixelColor = npxManager[i][0] | npxManager[i][1];
    if(pixelColor)
    {
      stickyHasBeenSet[i] = 1;
      stickyBrightnessDiv[i] = FULL_BRIGHT_DIV;
    }
    else if(stickyHasBeenSet[i])
    {
      stickyBrightnessDiv[i] = HALF_BRIGHT_DIV;
    }
    stickyColor[i] = pixelColor | stickyColor[i];
    pixels.setPixelColor(i, dimColorDiv(stickyColor[i], stickyBrightnessDiv[i]));
  }
}

void fadeStripMode()
{
  for(int i = 0; i < RING_START; i++)
  {
    uint32_t pixelColor = npxManager[i][0] | npxManager[i][1];
    fadeColor[i] = dimColorSub(fadeColor[i]/*, fadeSub[i]*/);
    if(pixelColor)
    {
      fadeColor[i] = pixelColor;
    }
    pixels.setPixelColor(i, fadeColor[i]);
  }
}
