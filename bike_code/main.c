
#include <stdio.h>
#include <string.h>
#include "nrf_drv_pwm.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"
#include "bsp.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
// nrf includes
#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "nrfx_gpiote.h"
#include "nrfx_saadc.h"

// buckler includes
#include "buckler.h"
#include "display.h"

#define OUTPUT_PIN 26
#define pi acos(-1.0)
#define radToDeg (180.0 / pi)
// system_bias = 2.885 / 3.0
#define lsb_size ((0.6 * 6) / 4096)
#define slope (.420 * 2.885 / 3.0) // .420 * system_bias
#define bias (-1.5 * 2.885 / 3.0) // -1.5 * system_bias
// ADC channels
#define X_CHANNEL 0
#define Y_CHANNEL 1
#define Z_CHANNEL 2
double values[100];
int counter = 0;
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
// Declare different arrays which create several different colors
nrf_pwm_values_individual_t red_values[] = {
13,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,100,100,
};
nrf_pwm_values_individual_t green_values[] = {
6,6,6,6,6,6,6,6,
13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
100,100
};
nrf_pwm_values_individual_t off_values[] = {
13,13,13,13,13,13,13,13,
13,13,13,13,13,13,13,13,
13,13,13,13,13,13,13,13,
};
nrf_pwm_sequence_t const red_seq =
{
    .values.p_individual = red_values,
    .length          = NRF_PWM_VALUES_LENGTH(red_values),
    .repeats         = 0,
    .end_delay       = 0
};
nrf_pwm_sequence_t const green_seq =
{
    .values.p_individual = green_values,
    .length          = NRF_PWM_VALUES_LENGTH(green_values),
    .repeats         = 0,
    .end_delay       = 0
};

nrf_pwm_sequence_t const off_seq =
{
    .values.p_individual = off_values,
    .length          = NRF_PWM_VALUES_LENGTH(off_values),
    .repeats         = 0,
    .end_delay       = 0
};

// callback for SAADC events
void saadc_callback (nrfx_saadc_evt_t const * p_event) {
  // don't care about adc callbacks
}

// sample a particular analog channel in blocking mode
nrf_saadc_value_t sample_value (uint8_t channel) {
  nrf_saadc_value_t val;
  ret_code_t error_code = nrfx_saadc_sample_convert(channel, &val);
  APP_ERROR_CHECK(error_code);
  return val;
}

// BUCKER/nRF INITIALIZATION CODE

void init_RTT() {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();

  printf("RTT initialized!\n");
}

void init_accelerometer() {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize analog to digital converter
  nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;
  saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;
  error_code = nrfx_saadc_init(&saadc_config, saadc_callback);
  APP_ERROR_CHECK(error_code);

  // initialize analog inputs
  // configure with 0 as input pin for now
  nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(0);
  channel_config.gain = NRF_SAADC_GAIN1_6; // input gain of 1/6 Volts/Volt, multiply incoming signal by (1/6)
  channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL; // 0.6 Volt reference, input after gain can be 0 to 0.6 Volts

  // specify input pin and initialize that ADC channel for X, Y, and Z
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_X;
  error_code = nrfx_saadc_channel_init(X_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_Y;
  error_code = nrfx_saadc_channel_init(Y_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_Z;
  error_code = nrfx_saadc_channel_init(Z_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  printf("Accelerometer initialized!\n");
}
void sample_accel(double* x_acc, double* y_acc, double* z_acc) {
  nrf_saadc_value_t x_val = sample_value(X_CHANNEL);
  nrf_saadc_value_t y_val = sample_value(Y_CHANNEL);
  nrf_saadc_value_t z_val = sample_value(Z_CHANNEL);

  *x_acc = (x_val * lsb_size + bias) / slope;
  *y_acc = (y_val * lsb_size + bias) / slope;
  *z_acc = (z_val * lsb_size + bias) / slope;

  // Used to calculate angles of rotation
  //double theta = atan(x_acc / pow(pow(y_acc, 2) + pow(z_acc, 2), 0.5)) * radToDeg;
  //double psi = atan(y_acc / pow(pow(x_acc, 2) + pow(z_acc, 2), 0.5)) * radToDeg;
  //double phi = atan(z_acc / pow(pow(y_acc, 2) + pow(x_acc, 2), 0.5)) * radToDeg;
}

// Set duty cycle between 0 and 100%
void pwm_update_color(uint8_t color)
{
    if (color == 0){
        // We perform playback for red
        nrf_drv_pwm_simple_playback(&m_pwm0, &red_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
    } else if (color == 1){
        // We perform playback for green
        nrf_drv_pwm_simple_playback(&m_pwm0, &green_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
    } else {
        // We assume they want to play yellow
        nrf_drv_pwm_simple_playback(&m_pwm0, &off_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
    }
}
void pwm_init(void)
{
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            OUTPUT_PIN, 
        },
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 21,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));
}
void led_init(void){
    NRF_CLOCK->TASKS_HFCLKSTART = 1; 
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
        ;
    pwm_init();
    pwm_update_color(2);
}
static int compare (const void * a, const void * b)
{
  if (*(double*)a > *(double*)b) return 1;
  else if (*(double*)a < *(double*)b) return -1;
  else return 0;  
}
// median and IQR based method
void stats(double* median, double* upper_iqr, double* lower_iqr){
  int length = counter+1;
  qsort (values, length, sizeof(double), compare);  
  *median = values[length/2];
  double iqr = values[(3*length)/4]-values[length/4];
  *upper_iqr = (iqr)*7 + values[3*length/4];
  *lower_iqr = values[length/4] - (iqr)*7;
}
// standard deviation and mean based method1
bool detected(void){
    double x;double y;double z;
    sample_accel(&x,&y,&z);
    if (x > 0.0){
        return false;
    }
    values[counter] = x*-1;
    double median; double upper_iqr; double lower_iqr; 
    stats(&median,&upper_iqr,&lower_iqr);
    if (x*-1 > upper_iqr || x*-1 < lower_iqr){
        printf("OUTLIAR");
        printf("\n upper %f\n",upper_iqr);
        printf("\n lower %f\n",lower_iqr);
        printf("\n value %f\n",x*-1);
        return true;
    }
    return false;
}
int main(void)
{
    led_init();
    init_accelerometer();
    pwm_update_color(0);
    while(1)
    {
        nrf_delay_ms(100);
        if (detected()){
            // TODO we should add a timed call in here to turn this off after like 3 seconds or something, also while this timer is activated we dont append 
            // any data to our online detection scheme because this is all
            pwm_update_color(1);
        }
        counter +=1;
        counter = counter % 100;
    }
}


/** @} */
