#ifndef ACTUATOR_ADC_TABLES_H__
#define ACTUATOR_ADC_TABLES_H__
#include <cstdint>
// #include "actuator_adc.h"
// #include "motor.h"//for fault bits
// #include "nuttx/config.h"
typedef struct {
  const uint16_t num_entries;
  const float vbias;
  const float rbias;
  const float *imp;
  const float *deg;
} _thermistor_info;

/***************** Standard Thermistor values *********************/
#define THERM_ENTRIES_STD       21
//Lookup table for NCP18XH103F03RB, standard thermistor used on all our products
//Note that values for -50 and 130-150 are interpolated, the rest are provided
//by the manufacturer.
const float therm_impedance_std[THERM_ENTRIES_STD] = {
    325000.0f,
    195652.0f,
    113347.1f,
     68236.7f,
     42506.2f,
     27218.6f,
     17925.5f,
     12080.5f,
      8314.5f,
      5833.6f,
      4160.9f,
      3014.3f,
      2227.5f,
      1668.5f,
      1268.0f,
       973.8f,
       758.0f,
       596.4f,
       383.9f,
       282.2f,
       207.4f,
};

//degree c
const float therm_deg_std[THERM_ENTRIES_STD] = {
    -50.0f,
    -40.0f,
    -30.0f,
    -20.0f,
    -10.0f,
      0.0f,
     10.0f,
     20.0f,
     30.0f,
     40.0f,
     50.0f,
     60.0f,
     70.0f,
     80.0f,
     90.0f,
    100.0f,
    110.0f,
    120.0f,
    130.0f,
    140.0f,
    150.0f,
};

const _thermistor_info therm_info_std = {
    .num_entries = THERM_ENTRIES_STD,
    .vbias = 3.0f,
    .rbias = 1000.0f,
    .imp = therm_impedance_std,
    .deg = therm_deg_std,
};

/***************** Koford 129 Motor Thermistor values *********************/

#define THERM_ENTRIES_KOFORD  41
const float therm_impedance_koford[THERM_ENTRIES_KOFORD] = {
  334850.0f,
  236250.0f,
  168700.0f,
  121850.0f,
   89000.0f,
   65650.0f,
   48880.0f,
   36735.0f,
   27850.0f,
   21285.0f,
   16395.0f,
   12750.0f,
    9990.0f,
    7880.0f,
    6260.0f,
    5000.0f,
    4019.0f,
    3249.0f,
    2641.0f,
    2158.0f,
    1773.0f,
    1474.0f,
    1233.0f,
    1035.0f,
     874.0f,
     741.0f,
     631.0f,
     539.0f,
     462.0f,
     398.0f,
     344.0f,
     299.0f,
     261.0f,
     228.0f,
     200.0f,
     177.0f,
     156.0f,
     138.0f,
     123.0f,
     110.0f,
      98.0f,
};

//degree c
const float therm_deg_koford[THERM_ENTRIES_KOFORD] = {
  -50.0f,
  -45.0f,
  -40.0f,
  -35.0f,
  -30.0f,
  -25.0f,
  -20.0f,
  -15.0f,
  -10.0f,
   -5.0f,
    0.0f,
    5.0f,
   10.0f,
   15.0f,
   20.0f,
   25.0f,
   30.0f,
   35.0f,
   40.0f,
   45.0f,
   50.0f,
   55.0f,
   60.0f,
   65.0f,
   70.0f,
   75.0f,
   80.0f,
   85.0f,
   90.0f,
   95.0f,
  100.0f,
  105.0f,
  110.0f,
  115.0f,
  120.0f,
  125.0f,
  130.0f,
  135.0f,
  140.0f,
  145.0f,
  150.0f,
};


const _thermistor_info therm_info_koford = {
    .num_entries = THERM_ENTRIES_KOFORD,
    .vbias = 3.0f,
    .rbias = 1000.0f,
    .imp = therm_impedance_koford,
    .deg = therm_deg_koford,
};

/***************** Kollmorgen KBM Motor Thermistor values *********************/
#define THERM_ENTRIES_KOLLMORGEN  6
const float therm_impedance_kollmorgen[THERM_ENTRIES_KOLLMORGEN] = {
    803.10f,
    960.90f,
    1000.0f,
    1194.0f,
    1385.10f,
    1573.30f
};
//degree c
const float therm_deg_kollmorgen[THERM_ENTRIES_KOLLMORGEN] = {
    -50.0f,
    -10.0f,
    0.0f,
    50.0f,
    100.0f,
    150.0f,
};

const _thermistor_info therm_info_kollmorgen = {
    .num_entries = THERM_ENTRIES_KOLLMORGEN,
    .vbias = 3.0f,
    .rbias = 1000.0f,
    .imp = therm_impedance_kollmorgen,
    .deg = therm_deg_kollmorgen,
};
#endif