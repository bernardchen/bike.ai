// Non-intrusive bicycle lighting system
// includes brake, turn, and blind-spot lighting

// C includes
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

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

// project includes
#include "ultrasonic_ranger.h"

// macros for accelerometer
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

// Bucker LED array
static uint8_t LEDS[3] = {BUCKLER_LED0, BUCKLER_LED1, BUCKLER_LED2};

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

void init_SD_logging() {
  ret_code_t error_code = NRF_SUCCESS;

  // Enable SoftDevice (used to get RTC running)
  nrf_sdh_enable_request();

  // Initialize GPIO driver
  if (!nrfx_gpiote_is_init()) {
    error_code = nrfx_gpiote_init();
  }
  APP_ERROR_CHECK(error_code);

  // Configure GPIOs
  nrf_gpio_cfg_output(BUCKLER_SD_ENABLE);
  nrf_gpio_cfg_output(BUCKLER_SD_CS);
  nrf_gpio_cfg_output(BUCKLER_SD_MOSI);
  nrf_gpio_cfg_output(BUCKLER_SD_SCLK);
  nrf_gpio_cfg_input(BUCKLER_SD_MISO, NRF_GPIO_PIN_NOPULL);

  nrf_gpio_pin_set(BUCKLER_SD_ENABLE);
  nrf_gpio_pin_set(BUCKLER_SD_CS);

  // Initialize SD card
  const char filename[] = "testfile.log";
  const char permissions[] = "a"; // w = write, a = append

  // Start file
  simple_logger_init(filename, permissions);

  printf("SD card logging initialized!\n");
}

void init_buckler_LEDs() {
  ret_code_t error_code = NRF_SUCCESS;
  nrf_gpio_cfg_output(LEDS[3]);

  // initialize GPIO driver
  if (!nrfx_gpiote_is_init()) {
    error_code = nrfx_gpiote_init();
  }
  APP_ERROR_CHECK(error_code);

  // configure leds
  // manually-controlled (simple) output, initially set
  nrfx_gpiote_out_config_t out_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
  for (int i=0; i<3; i++) {
    error_code = nrfx_gpiote_out_init(LEDS[i], &out_config);
    APP_ERROR_CHECK(error_code);
  }
}

/**@brief Function starting the internal LFCLK oscillator.
 *
 * @details This is needed by RTC1 which is used by the Application Timer
 *          (When SoftDevice is enabled the LFCLK is always running and this is not needed).
 */
// obtained from tutorial: 
// https://devzone.nordicsemi.com/nordic/short-range-guides/b/software-development-kit/posts/application-timer-tutorial 
static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

/***** SHOULD BE CALLED AT THE VERY START OF MAIN SO EVERYTHING INVOLVING app_timer IS SET UP *****/
static void set_up_app_timer(void)
{
  lfclk_request();
  ret_code_t error_code = app_timer_init();
  APP_ERROR_CHECK(error_code);
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

typedef enum {
  RIGHT,
  LEFT,
  OFF,
  ON
} on_off_state_t;

int main (void) {
  // Initialization code
  init_RTT();
  init_accelerometer();
  init_buckler_LEDs();
  set_up_app_timer();
  /********* ultrasonic ranger stuff *********/
  init_ultrasonic_ranger(D, 1);
  // LEDs for output of something nearby
  // TODO: CHANGE WHEN HAVE REAL LEDs
  // nrf_gpio_cfg_output(BUCKLER_LED0);
  printf("Buckler initialized!\n");

  // State machines
  on_off_state_t brake_state = OFF;
  on_off_state_t proximity_state = OFF;
  on_off_state_t turn_state = OFF;

  // Loop forever
  while (1) {
    // Determines sampling rate
    // TODO: Figure out how to to dealsy because ultrasonic_ranger can't delay more than 1ms
    //nrf_delay_ms(10);

    for (int i=0; i<3; i++) {
      nrf_gpio_pin_toggle(LEDS[i]);
    }

    long range = ultrasonic_ranger_loop_call();
    //printf("Ultrasonic ranger range: %ld\n", range);

    // STATE MACHINES
    // State machine for brake
    switch(brake_state) {
      case OFF: {
        break;
      }
      case ON: {
        break;
      }
    }

    // State machine for proximity sensors
    switch(proximity_state) {
      case OFF: {
        break;
      }
      case ON: {
        break;
      }
    }

    // State machine for turn indicators
    switch(turn_state) {
      case OFF: {
        break;
      }
      case LEFT: {
        break;
      }
      case RIGHT: {
        break;
      }
    }
  }
}
