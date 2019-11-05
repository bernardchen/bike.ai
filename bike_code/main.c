// Non-intrusive bicycle lighting system
// includes brake, turn, and blind-spot lighting

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

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

#include "buckler.h"

#include "display.h"

#define pi acos(-1.0)
#define radToDeg (180.0 / pi)

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

int main (void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();

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

  // initialization complete
  printf("Buckler initialized!\n");

  double system_bias = 2.885 / 3.0;
  double lsb_size = (0.6 * 6) / 4096;
  double slope = .420 * system_bias;
  double bias = -1.5 * system_bias;
  // loop forever
  
// SD CARD INIT --------------------
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

  // init display
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
  display_write("Logging to SD", DISPLAY_LINE_0);
  printf("Display initialized!\n");

  time_t rawtime;
  struct tm *timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  simple_logger_log("STARTING RUN AT TIME %s ----------\n", asctime(timeinfo));

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


    // display results
    printf("x: %.5fg\ty: %.5fg\tz:%.5fg\n", x_acc, y_acc, z_acc);
    //char buf[16] = {0};
    //snprintf(buf, 16, "%lf", x_acc);
    //display_write(buf, 0);

    // log to SD card
    char x_buf[10];
    char y_buf[10];
    char z_buf[10];
    gcvt(x_acc, 10, x_buf);
    gcvt(y_acc, 10, y_buf);
    gcvt(z_acc, 10, z_buf);
    simple_logger_log("x:%.10sg, y: %.10sg, z: %.10sg\n", x_buf, y_buf, z_buf);
    printf("Wrote line to SD card\n");
    //printf("x: %lfdeg\ty: %lfdeg\tz:%lfdeg\n", theta, psi, phi);
    nrf_delay_ms(50);
  }
}


