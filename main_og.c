// Non-intrusive bicycle lighting system
// includes brake, turn, and blind-spot lighting

// C includes
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "boards.h"
#include "nrf_drv_i2s.h"
#include <math.h>

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
// FASTLED depends
#define NLEDS 16
#define RESET_BITS 6
#define I2S_BUFFER_SIZE 3*NLEDS + RESET_BITS


static uint32_t m_buffer_tx[I2S_BUFFER_SIZE];
static volatile int nled = 1;
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
static void data_handler(uint32_t const * p_data_received,
                         uint32_t       * p_data_to_send,
                         uint16_t         number_of_words)
{
    // Non-NULL value in 'p_data_to_send' indicates that the driver needs
    // a new portion of data to send.
    if (p_data_to_send != NULL)
    {
        // do nothing - buffer is updated elsewhere
    }
}


float calculateSD(float data[])
{
    float sum = 0.0, mean, standardDeviation = 0.0;
    int i;
    for(i=0; i<10; ++i)
    {
        sum += data[i];
    }
    mean = sum/10;
    for(i=0; i<10; ++i)
        standardDeviation += pow(data[i] - mean, 2);
    return sqrt(standardDeviation/10);
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

void init_display(char* message) {
  ret_code_t error_code = NRF_SUCCESS;

  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);

  display_write(message, DISPLAY_LINE_0);

  printf("Display initialized!\n");
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
}bool compute_break_trigger(x_acc,y_acc,z_acc){
	return true;
}


typedef enum {
  RIGHT,
  LEFT,
  OFF,
  ON
} on_off_state_t;

#include "app_pwm.h"

APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.
void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    
}

int main (void) {
  // Initialization code
  init_RTT();
  init_accelerometer();
  printf("Buckler initialized!\n");
  double x_acc;
  double y_acc;
  double z_acc;
  // State machines
  on_off_state_t brake_state = OFF;
  on_off_state_t proximity_state = OFF;
  on_off_state_t turn_state = OFF;

low_power_pwm_t low_power_pwm;
low_power_pwm_config_t low_power_pwm_config = LOW_POWER_PWM_DEFAULT_CONFIG(LEDS_MASK);      // use default configuration with given LEDs mask
low_power_pwm_init((&low_power_pwm), &low_power_pwm_config, pwm_handler);
low_power_pwm_duty_set(&low_power_pwm, 50);                                                 // set duty cycle to 50
low_power_pwm_start(&low_power_pwm, LEDS_MASK);

  // Loop forever
  while (1) {
    for (uint8_t i = 0; i < 40; ++i)
    {
        value = (i < 20) ? (i * 5) : (100 - (i - 20) * 5);
        
        /* Se the duty cycle - keep trying until PWM is ready. */
        while (app_pwm_channel_duty_set(&PWM1, 0, value) == NRF_ERROR_BUSY);
        while (app_pwm_channel_duty_set(&PWM1, 1, value) == NRF_ERROR_BUSY);
        nrf_delay_ms(25);
    }

    // Determines sampling rate
 //    nrf_delay_ms(100);
 //    sample_accel(&x_acc,&y_acc,&z_acc);
 //    printf("X %lf\n Y %lf\n Z %lf\n",x_acc,y_acc,z_acc);
 //    if (compute_break_trigger(x_acc,y_acc,z_acc)){
 //    	printf("y");
	// }

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
