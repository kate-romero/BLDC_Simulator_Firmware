/*
 * actuator_adc.c
 *
 *  Created on: Jul 27, 2012
 *      Author: ctaglia
 */

#include <cstdint>
// #include "globals.h"
#include "actuator_adc_tables.cpp"
// a_mux_channel           mux_chan;
// extern _mtr_info        mtr_info;
// extern _fault           sys_faults;
extern bool             Received_Sp_Cmd;
#if defined(CONFIG_ANALOG_VELOCITY_CONTROL) || defined(CONFIG_ANALOG_POSITION_CONTROL_LINEAR) || defined(CONFIG_ANALOG_POSITION_CONTROL_BISTABLE)
static int32_t          volt_in_buf[CONFIG_ANALOG_CONTROL_RUNNING_AVERAGE_COUNT];
#endif
//#define ADC_CTRL_DBG
#ifdef ADC_CTRL_DBG
#define adcdbg(...) syslog(LOG_INFO, __VA_ARGS__)
#else
#define adcdbg(...)
#endif
/*************************************************************************************
 * Declarations
 *************************************************************************************/
static float    compute_thermistor(float vsense, const _thermistor_info *info);
static float    compute_thermistor_r(float rsense, const _thermistor_info *info);
static float    calc_r_from_v(float vsense, float vbias, float rbias);
// static void     handle_external_analog_value_update(uint32_t ticks);
// static void     handle_sys_volt_analogue_value_update(uint32_t ticks);
// static void     handle_brake_current_analog_value_update(int32_t ticks);
// static void     handle_temp_analogue_value_update(uint32_t ticks, int8_t* temp, const _thermistor_info *info);
// static void     check_overvoltage(void);
#ifdef CONFIG_THERMAL_LIMITED_BOOST
static void     update_boost_accumulator(uint32_t motor_current);
#endif
/**********************************************************************************
 * init_adc
 * Configure ADC drivers and configs
 *
 ***********************************************************************************/
 
/***************************************************************************
 * compute_thermistor
 *
 *  Get the temperature from the voltage
 *************************************************************************/
static float compute_thermistor(float vsense, const _thermistor_info *info) {
  float r;

  r = calc_r_from_v(vsense, info->vbias, info->rbias);
//  syslog(LOG_INFO, "Resistance: %ue-1000\n", ((uint32_t)(r*1000.0f)));
  return compute_thermistor_r(r, info);
}

/***************************************************************************
 * compute_thermistor
 *
 *  Get the temperature from the resistance
 *************************************************************************/
static float compute_thermistor_r(float rsense, const _thermistor_info *info) {
  float pct_x;
  uint16_t i;
  const uint16_t entries = info->num_entries;
  const float *imp = info->imp;
  const float *deg = info->deg;

  for (i = 1; i < entries; i++) {
    if (rsense >= imp[i]) {
      pct_x = (imp[i-1] - rsense) / (imp[i - 1] - imp[i]);
      return ((deg[i] - deg[i - 1]) * pct_x) + deg[i - 1];
    }
  }
  return deg[entries -1];
}

/* Calculates the thermistor resistance given the measured voltage.
 * Makes assumptions about the reference voltage and the bias resistor.
 * if at some point these change, they could be incorporated into the
 * const _thermistor_info structure.
 */
static float calc_r_from_v(float vsense, float vbias, float rbias) {
  float i, r;

  if (vsense > (vbias - 0.1f)) {
    return 1000000.0f; //r is very large
  }
  i = (vbias - vsense) / rbias;
  r = vsense / (float)i;

  return r;
}
