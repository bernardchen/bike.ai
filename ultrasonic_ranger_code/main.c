// Analog accelerometer app
//
// Reads data from the ADXL327 analog accelerometer

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "app_error.h"
#include "app_timer.h"
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

#include "buckler.h"

#include "ultrasonic_ranger.h"

#include "display.h"
#define PIR_MOTION_SENSOR 2 
#define pi acos(-1.0)
#define radToDeg (180.0 / pi)

// ADC channels
#define X_CHANNEL 0
#define Y_CHANNEL 1
#define Z_CHANNEL 2

// callback for SAADC events
void saadc_callback (nrfx_saadc_evt_t const * p_event) {
  // don't care about adc callbacks
  printf("\n\n\nsaadc callback happening\n\n\n");
}

// sample a particular analog channel in blocking mode
nrf_saadc_value_t sample_value (uint8_t channel) {
  nrf_saadc_value_t val;
  ret_code_t error_code = nrfx_saadc_sample_convert(channel, &val);
  APP_ERROR_CHECK(error_code);
  return val;
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




int main (void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  // nrf_gpio_cfg_input(BUCKLER_GROVE_A1,NRF_GPIO_PIN_NOPULL);
  // nrf_gpio_cfg_input(BUCKLER_GROVE_A0,NRF_GPIO_PIN_NOPULL);
  // nrf_gpio_cfg_input(BUCKLER_GROVE_D0, NRF_GPIO_PIN_NOPULL);
  // nrf_gpio_cfg_input(BUCKLER_GROVE_D1, NRF_GPIO_PIN_NOPULL);
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

  // specify input pin and initialize that ADC channel
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_X;
  error_code = nrfx_saadc_channel_init(X_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  // specify input pin and initialize that ADC channel
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_Y;
  error_code = nrfx_saadc_channel_init(Y_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  // specify input pin and initialize that ADC channel
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_Z;
  error_code = nrfx_saadc_channel_init(Z_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);




  /********* ultrasonic ranger stuff *********/
  set_up_app_timer();
  init_ultrasonic_ranger(D);
  nrf_gpio_cfg_output(BUCKLER_LED0);


  // initialization complete
  printf("Buckler initialized!\n");

  double system_bias = 2.885 / 3.0;
  double lsb_size = (0.6 * 6) / 4096;
  double slope = .420 * system_bias;
  double bias = -1.5 * system_bias;
  // loop forever
  
  // init display
  // nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  // nrf_drv_spi_config_t spi_config = {
  //   .sck_pin = BUCKLER_LCD_SCLK,
  //   .mosi_pin = BUCKLER_LCD_MOSI,
  //   .miso_pin = BUCKLER_LCD_MISO,
  //   .ss_pin = BUCKLER_LCD_CS,
  //   .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
  //   .orc = 0,
  //   .frequency = NRF_DRV_SPI_FREQ_4M,
  //   .mode = NRF_DRV_SPI_MODE_2,
  //   .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  // };
  // error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  // APP_ERROR_CHECK(error_code);
  // display_init(&spi_instance);
  // display_write("Hello, Human!", DISPLAY_LINE_0);
  // printf("Display initialized!\n");
  // display_write("no", 1);


  //init SD card logging
  /*printf("trying to init logger\n");
  simple_logger_init("test1.txt", "a");
  printf("logger inited\n");
  simple_logger_power_on();  
  printf("logger powered on\n");*/

  while (1) {
    // sample analog inputs
    nrf_saadc_value_t x_val = sample_value(X_CHANNEL);
    nrf_saadc_value_t y_val = sample_value(Y_CHANNEL);
    nrf_saadc_value_t z_val = sample_value(Z_CHANNEL);

    double x_acc = (x_val * lsb_size + bias) / slope;
    double y_acc = (y_val * lsb_size + bias) / slope;
    double z_acc = (z_val * lsb_size + bias) / slope;

    double theta = atan(x_acc / pow(pow(y_acc, 2) + pow(z_acc, 2), 0.5)) * radToDeg;
    double psi = atan(y_acc / pow(pow(x_acc, 2) + pow(z_acc, 2), 0.5)) * radToDeg;
    double phi = atan(z_acc / pow(pow(y_acc, 2) + pow(x_acc, 2), 0.5)) * radToDeg;



    /**** Ultrasonic ranger stuff ****/
    long range = ultrasonic_ranger_loop_call();
    //printf("Range: %ld\n", range);

    if (range <= 250)
    {
      nrf_gpio_pin_clear(BUCKLER_LED0);  
    }
    else
    {
      nrf_gpio_pin_set(BUCKLER_LED0);
    }
    
    /******** ORIGINAL ULTRASONIC TESTING **********/
    /*
    printf("SEND OUT SOUND\n");
    nrf_gpio_cfg_output(BUCKLER_GROVE_D0);
    nrf_gpio_pin_clear(BUCKLER_GROVE_D0);
    nrf_delay_ms(2);
    nrf_gpio_pin_set(BUCKLER_GROVE_D0);
    nrf_delay_ms(10);
    nrf_gpio_pin_clear(BUCKLER_GROVE_D0);

    printf("CHANGE TO INPUT\n");

    nrf_gpio_cfg_input(BUCKLER_GROVE_D0, NRF_GPIO_PIN_NOPULL);

    // uint32_t left_sensed = nrf_gpio_pin_read(BUCKLER_GROVE_A0); 
    // uint32_t left_othersensed = nrf_gpio_pin_read(BUCKLER_GROVE_A1);
    for (int i = 0; i < 40; i++)
    {
      printf("Elapsed time (millisec): %2d, D: %x\n", //0: %x, A1: %x, D0: %x, D1: %x\n",
        i,
        // nrf_gpio_pin_read(BUCKLER_GROVE_A0),
        // nrf_gpio_pin_read(BUCKLER_GROVE_A1),
        nrf_gpio_pin_read(BUCKLER_GROVE_D0)); //,
        // nrf_gpio_pin_read(BUCKLER_GROVE_D1));
      nrf_delay_ms(1);
    }
    
    printf("\nEnd input read\n\n");
    //*/

    //printf("pin3: %x pin4: %x",left_sensed,left_othersensed);
    // display results
    //printf("x: %lfg\ty: %lfg\tz:%lfg\n", x_acc, y_acc, z_acc);

    // char buf[16] = {0};
    // snprintf(buf, 16, "%lf", x_acc);
    // display_write(buf, 0);
    // if (left_sensed || left_othersensed) {
    //     display_write("Object Detected", 1);
    // }

    //nrf_delay_ms(500);
    //nrf_delay_ms(1);
  }
}


