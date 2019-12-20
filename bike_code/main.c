// Non-intrusive bicycle lighting system
// includes brake, turn, and blind-spot lighting

// C includes
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

// nrf includes
#include "nrf_drv_pwm.h"
#include "app_util_platform.h"
#include "nrf_drv_clock.h"
#include "boards.h"
#include "bsp.h"
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
#include "mpu9250.h"

// project includes
#include "gpio.h"
#include "ultrasonic_ranger.h"
#include "turn_ble.h"

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
#define SAMPLE_SIZE 40

// ultrasonic ranger proximity constants
#define LOOP_HOLD_AMOUNT (9) // number of loops that a ultrasonic ranger needs to be held for it to change

// turn signal constants
#define TURN_DETECTED_ACCEL_THRESH (0.4)
#define TURN_TIMEOUT (10)

// LED constants
#define OUTPUT_PIN_BRAKE (12)
#define OUTPUT_PIN_LEFT_TURN (26)
#define OUTPUT_PIN_RIGHT_TURN (11)

// brake constants
#define BRAKE_LIGHT_ON_SECS (1)

// Create timer
APP_TIMER_DEF(main_timer);

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

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

/********************* BUCKER/nRF INITIALIZATION CODE *********************/

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

void init_mpu9250() {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);

  // initialize MPU-9250 driver
  mpu9250_init(&twi_mngr_instance);
  printf("MPU-9250 initialized\n");
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
  lfclk_request(); // need to call this to make the oscillator start so timers actually work
  ret_code_t error_code = app_timer_init();
  APP_ERROR_CHECK(error_code);
}

// timer stuff for timeout
uint32_t turn_time_on = 0; // turn_time_on used for turn signal state machine
uint32_t button_press_time = 0; // to make sure a single button press is not registered twice
uint32_t brake_time_on = 0; // brake_time_on used for brake state machine
bool light_toggle = true; // used to make turn lights blink
bool pwm_set = false; // PWMs need to only be set once, so this bool will keep track of that
uint8_t millisec_counter = 0; // to count to four to make sure other counters only change on the second
// Callback function for timer
void main_timer_callback() {
  if (millisec_counter == 3) // 0 indexed
  {
    millisec_counter = 0;
    turn_time_on++;
    button_press_time++;
    brake_time_on++;  
  }
  else
  {
    millisec_counter++;
  }
  light_toggle = !light_toggle; // makes it so blinks at a rate of a change every 250ms
  pwm_set = true; // says that pwm needs to be set
}

void init_main_timer() {
  app_timer_create(&main_timer, APP_TIMER_MODE_REPEATED, (app_timer_timeout_handler_t) main_timer_callback);
  app_timer_start(main_timer, APP_TIMER_TICKS(250), NULL); // 250 milliseconds
  printf("Timer initialized\n");
}

void init_button0() {
  gpio_config(28, 0);
}


/********************* Sample values from the accelerometers *********************/
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

void sample_9250_accelerometer(float* x_axis, float* y_axis, float* z_axis) {
  mpu9250_measurement_t accelerometer_measurement = mpu9250_read_accelerometer();
  *x_axis = accelerometer_measurement.x_axis;
  *y_axis = accelerometer_measurement.y_axis;
  *z_axis = accelerometer_measurement.z_axis;
}



/************************* PWM configurations for LEDs *************************/
// Inspired by simple_pwm example in nRF forums
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);
static nrf_drv_pwm_t m_pwm2 = NRF_DRV_PWM_INSTANCE(2);

// Declare different arrays which create several different colors
nrf_pwm_values_individual_t red_values[] = {
    6,6,6,6,6,6,6,6,
    6,6,6,6,6,6,6,6,
    6,6,6,6,6,6,6,6,
};
nrf_pwm_values_individual_t green_values[] = {
   6,6,6,6,6,6,6,6,
    13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
    100,100,
};
nrf_pwm_values_individual_t yellow_values[] = {
   6,6,6,6,6,6,6,6,
     6,6,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
     100,100
};
nrf_pwm_values_individual_t off_values[] = {
   13,13,13,13,13,13,13,13,
     13,13,13,13,13,13,13,13,
    13,13,13,13,13,13,13,13,
};

// create actual data types needed for pwm to work
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
nrf_pwm_sequence_t const yellow_seq =
{
    .values.p_individual = yellow_values,
    .length          = NRF_PWM_VALUES_LENGTH(yellow_values),
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

// Initializes the pwm. DON'T CALL. Will be handled by led_init.
void pwm_init(void)
{
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            OUTPUT_PIN_BRAKE,
        },
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 21,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));
    nrf_drv_pwm_config_t const config1 =
    {
        .output_pins =
        {
            OUTPUT_PIN_LEFT_TURN,
            
        },
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 21,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm1, &config1, NULL));
    nrf_drv_pwm_config_t const config2 =
    {
        .output_pins =
        {
            OUTPUT_PIN_RIGHT_TURN,
            
        },
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 21,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm2, &config2, NULL));
}

// The function to call to initialize LEDs and everything related (like pwm)
void led_init(void){
    NRF_CLOCK->TASKS_HFCLKSTART = 1; 
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
        ;
    pwm_init();
}


/************************* Brake detection *************************/
double values[SAMPLE_SIZE];
int counter = 0;
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
  *upper_iqr = (iqr)*2.75 + values[3*length/4];
  *lower_iqr = values[length/4] - (iqr)*2.75;
}
// standard deviation and mean based method1
bool detected_non_residual(void){
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
// need to fix a bit for both of these functions residual detected function
// standard deviation and mean based method1
bool detected_residual(void){
    double x;double y;double z;
    sample_accel(&x,&y,&z);
    if (x > 0.0){
        return false;
    }
    int length = 0;
    if (counter < SAMPLE_SIZE){
        length = counter/2;
    } else {
        length = SAMPLE_SIZE/2;
   }
    values[counter] = (x*-1 - values[length]);
    double median; double upper_iqr; double lower_iqr;
    stats(&median,&upper_iqr,&lower_iqr);
    if (x > upper_iqr || x < lower_iqr){
//        printf("OUTLIAR");
//        printf("\n upper %f\n",upper_iqr);
//        printf("\n lower %f\n",lower_iqr);
//        printf("\n value %f\n",x*-1);
        return true;
    }
    return false;
}

// main detected function that will call the proper one
// based on configured mode
// if is_residual is 1, calls that one
// otherwise, calls non-residual
bool detected(uint8_t is_residual)
{
  if (is_residual == 1)
  {
    return detected_residual();
  }
  else
  {
    return detected_non_residual();
  }
}

/************************* LED Lighting configs *************************/
// brakes is pwm0
void turn_off_brake_lights()
{
  nrf_drv_pwm_simple_playback(&m_pwm0, &off_seq, 40, NRF_DRV_PWM_FLAG_STOP);
}
void turn_on_brake_lights(uint8_t brake_color)
{
  if (brake_color == 0)
  {
    nrf_drv_pwm_simple_playback(&m_pwm0, &green_seq, 40, NRF_DRV_PWM_FLAG_STOP);
  }
  else if (brake_color == 1)
  {
    nrf_drv_pwm_simple_playback(&m_pwm0, &red_seq, 40, NRF_DRV_PWM_FLAG_STOP);
  }
  else
  {
    nrf_drv_pwm_simple_playback(&m_pwm0, &yellow_seq, 40, NRF_DRV_PWM_FLAG_STOP);
  }
}
// left is pwm1
void turn_off_left_lights()
{
  nrf_drv_pwm_simple_playback(&m_pwm1, &off_seq, 40, NRF_DRV_PWM_FLAG_STOP);
}
void turn_on_left_lights(uint8_t brake_color)
{
  if (brake_color == 0)
  {
    nrf_drv_pwm_simple_playback(&m_pwm1, &green_seq, 40, NRF_DRV_PWM_FLAG_STOP);
  }
  else if (brake_color == 1)
  {
    nrf_drv_pwm_simple_playback(&m_pwm1, &red_seq, 40, NRF_DRV_PWM_FLAG_STOP);
  }
  else
  {
    nrf_drv_pwm_simple_playback(&m_pwm1, &yellow_seq, 40, NRF_DRV_PWM_FLAG_STOP);
  }
}
// right is pwm2
void turn_off_right_lights()
{
  nrf_drv_pwm_simple_playback(&m_pwm2, &off_seq, 40, NRF_DRV_PWM_FLAG_STOP);
}
void turn_on_right_lights(uint8_t brake_color)
{
  if (brake_color == 0)
  {
    nrf_drv_pwm_simple_playback(&m_pwm2, &green_seq, 40, NRF_DRV_PWM_FLAG_STOP);
  }
  else if (brake_color == 1)
  {
    nrf_drv_pwm_simple_playback(&m_pwm2, &red_seq, 40, NRF_DRV_PWM_FLAG_STOP);
  }
  else
  {
    nrf_drv_pwm_simple_playback(&m_pwm2, &yellow_seq, 40, NRF_DRV_PWM_FLAG_STOP);
  }
}
 
// make turn lights flash
void turn_left_blink(uint8_t color)
{
  if (pwm_set)
  { 
    if (light_toggle)
    {
      turn_on_left_lights(color);
    }
    else
    {
      turn_off_left_lights();
    }
    pwm_set = false;
  }   
}
void turn_right_blink(uint8_t color)
{
  if (pwm_set)
  {
    if (light_toggle)
    {
      turn_on_right_lights(color);
    }
    else
    {
      turn_off_right_lights();
    }
    pwm_set = false;
  }
}

// States for each of the three sub-state machines (brake light, turn light, proximity light)
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
  init_mpu9250();
  init_buckler_LEDs();
  set_up_app_timer();
  init_main_timer();
  init_button0();
  /********* LED stuff *********/
  led_init();
  turn_off_brake_lights();
  turn_off_left_lights();
  turn_off_right_lights();
  /********* turn ble stuff *********/
  ble_init();
  /********* ultrasonic ranger stuff *********/
  init_ultrasonic_ranger(A, D, 1);

  printf("Buckler initialized!\n");

  // State machines
  on_off_state_t brake_state = OFF;
  on_off_state_t left_proximity_state = OFF;
  on_off_state_t right_proximity_state = OFF;
  on_off_state_t turn_state = OFF;

  // Set variables
  float velocity = 0;
  long left_range = 0;
  long right_range = 0;
  long distance_threshold = 200; // represents distance in centimeters

  // variable just for proximity sensor so that false reads don't change the state
  uint16_t num_in_a_row_left = 0;
  uint16_t num_in_a_row_right = 0;

  // Used for configurations set by the iOS app
  /* brake_mode: 0 is NON-RESIDUAL, 1 is RESIDUAL
   * turn_light_color: 0 is GREEN, 1 is RED, 2 is YELLOW
   * brake_light_color: 0 is GREEN, 1 is RED, 2 is YELLOW
   * proximity_dist: 0 is ZERO METERS, 1 is ONE METER, 2 is TWO METERS
   */
  uint8_t brake_mode = 0; // non-residual
  uint8_t turn_light_color = 0; // green
  uint8_t brake_light_color = 1; // red
  uint8_t proximity_dist = 2; // 200 cm

  // Loop forever
  while (1) {
    // Determines sampling rate
  	nrf_delay_ms(1);

    for (int i=0; i<3; i++) {
      nrf_gpio_pin_toggle(LEDS[i]);
    }

    // GET MEASUREMENTS AND INPUTS
    /************************************** APP SETTINGS **********************************/
    uint8_t app_info = 0;
    sample_app();
    app_info = get_app_info();

    // Mask characteristic value to split into individual fields
    brake_mode = (app_info & 0x3);
    turn_light_color = (app_info >> 6) & 0x3;
    brake_light_color = (app_info >> 4) & 0x3;
    proximity_dist = (app_info >> 2) & 0x3;
    //printf("appinfo: %d, turn %d, brake %d, proximity %d, mode %d\n", app_info, turn_light_color, brake_light_color, proximity_dist, brake_mode);
    
    // set distance_threshold to proper value based on config
    if (proximity_dist == 0)
    {
      distance_threshold = 0;
    }
    else if (proximity_dist == 1)
    {
      distance_threshold = 100;
    }
    else
    {
      distance_threshold = 200;
    }

    /************************************** TURNING **************************************/
    float x_acc, y_acc, z_acc;
    sample_9250_accelerometer(&x_acc, &y_acc, &z_acc);
	  // Add accelerometer sample
    bool turned_left = (y_acc < -TURN_DETECTED_ACCEL_THRESH), turned_right = (y_acc > TURN_DETECTED_ACCEL_THRESH);
    // check if button pressed
    bool ble_left = false;
    bool ble_right = false;
    sample_buttons();
    if (button_press_time > 0 && get_left_pressed()) {
      ble_left = true;
      button_press_time = 0;
      reset_left_button();
    }
    else if (button_press_time > 0 && get_right_pressed()) {
      ble_right = true;
      button_press_time = 0;
      reset_right_button();
    }
//    printf("timer: %i\n", button_press_time);
    bool left_turn = false, right_turn = false;
	/************************************** HALL EFFECT **************************************/
    // TODO: Implement hall-effect sensors to update velocity
    // Used to check left_turn and right_turn (i.e. tilted left/right && velocity > 0);
    //printf("X: %f, Y: %f, Z: %f\n", x_acc, y_acc, z_acc);
    //velocity = velocity + x_acc * 0.01;
    //printf("Velocity: %f\n", velocity);

    /************************************** PROXIMITY **************************************/
    left_range = ultrasonic_get_left_distance_cm();
    right_range = ultrasonic_get_right_distance_cm();
  //  printf("Ultrasonic LEFT ranger range: %ld\n", left_range);
   // printf("Ultrasonic RIGHT ranger range: %ld\n", right_range);

    // STATE MACHINES
    // State machine for brake
    //printf("brake color %d\n", brake_light_color);
    switch(brake_state) {
      case OFF: {
        //printf("Brake light is OFF\n");
        if (detected(brake_mode))
        {
          //printf("DETECTED BRAKE\n");
          //printf("BRAKE COLOR %d\n", brake_light_color);
          brake_time_on = 0;
          brake_state = ON;
          turn_on_brake_lights(brake_light_color);
        }
        break;
      }
      case ON: {
        //printf("Brake light is ON\n");
        if (brake_time_on > BRAKE_LIGHT_ON_SECS && !detected(brake_mode))
        {
          //printf("Turning off brake lights\n");
          turn_off_brake_lights();
          brake_time_on = 0;
          brake_state = OFF;
        }
        break;
      }
    }
    counter +=1;
    counter = counter % SAMPLE_SIZE;

    // State machine for proximity sensors
    // will only change if there are LOOP_HOLD_AMOUNT times in a row
    // machine for left side
    switch(left_proximity_state) {
      case OFF: {
      	if (left_range <= 250)
      	{
      		num_in_a_row_left += 1;

      	}
      	else
      	{
      		num_in_a_row_left = 0;
      	}

      	if (num_in_a_row_left >= LOOP_HOLD_AMOUNT && left_range <= distance_threshold) // already happened 9 times and tenth is also true
      	{
      		num_in_a_row_left = 0;
      		left_proximity_state = ON;
      		turn_on_left_proxi_led();
      	}	
        break;
      }
      case ON: {
      	if (left_range > distance_threshold)
      	{
      		num_in_a_row_left += 1;
      	}
      	else
      	{
      		num_in_a_row_left = 0;
      	}
      	if (num_in_a_row_left >= LOOP_HOLD_AMOUNT && left_range > distance_threshold)
      	{
      		num_in_a_row_left = 0;
      		left_proximity_state = OFF;
      		turn_off_left_proxi_led();
      	}
        break;
      }
    }
    // machine for right side
    switch(right_proximity_state) {
      case OFF: {
      	if (right_range <= 250)
      	{
      		num_in_a_row_right += 1;

      	}
      	else
      	{
      		num_in_a_row_right = 0;
      	}

      	if (num_in_a_row_right >= LOOP_HOLD_AMOUNT && right_range <= distance_threshold) // already happened 9 times and tenth is also true
      	{
      		num_in_a_row_right = 0;
      		right_proximity_state = ON;
      		turn_on_right_proxi_led();
      	}	
        break;
      }
      case ON: {
      	if (right_range > distance_threshold)
      	{
      		num_in_a_row_right += 1;
      	}
      	else
      	{
      		num_in_a_row_right = 0;
      	}
      	if (num_in_a_row_right >= LOOP_HOLD_AMOUNT && right_range > distance_threshold)
      	{
      		num_in_a_row_right = 0;
      		right_proximity_state = OFF;
      		turn_off_right_proxi_led();
      	}
        break;
      }
    }

    // State machine for turn indicators
    switch(turn_state) {
      case OFF: {
        if (ble_left) {
          turn_state = LEFT;
          turn_on_left_lights(turn_light_color);
          turn_off_right_lights();
          left_turn = true;
          turn_time_on = 0;
        } else if (ble_right) {
          turn_state = RIGHT;
          turn_off_left_lights();
          turn_on_right_lights(turn_light_color);
          right_turn = true;
          turn_time_on = 0;
        } else {
          turn_state = OFF;
         // printf("IN STATE OFF\n");
        }
        break;
      }
      case LEFT: {
        if (ble_left || turn_time_on > TURN_TIMEOUT || turned_left) {
          //printf("ble_left: %i, turn_time: %i, turned_left: %i\n", ble_left, turn_time_on > 60, turned_left);
          turn_state = OFF;
          left_turn = false;
          turn_off_left_lights();
          turn_off_right_lights();
        } else if (ble_right) {
          turn_state = RIGHT;
          turn_off_left_lights();
          turn_on_right_lights(turn_light_color);
          right_turn = true;
          left_turn = false;
          turn_time_on = 0;
        } else {
          turn_state = LEFT;
          left_turn = true;
          //printf("IN STATE LEFT\n");
        }
        break;
      }
      case RIGHT: {
        if (ble_right || turn_time_on > TURN_TIMEOUT || turned_right) {
          turn_state = OFF;
          right_turn = false;
          turn_off_left_lights();
          turn_off_right_lights();
        } else if (ble_left) {
          turn_state = LEFT;
          turn_on_left_lights(turn_light_color);
          turn_off_right_lights();
          left_turn = true;
          right_turn = false;
          turn_time_on = 0;
        } else {
          turn_state = RIGHT;
          right_turn = true;
          //printf("IN STATE RIGHT\n");
        }
        break;
      }
    }

    // turn on the corresponding LEDs if in a turning state
    if (left_turn)
    {
      turn_left_blink(turn_light_color);
    }
    else if (right_turn)
    {
      turn_right_blink(turn_light_color);
    }
  }
}
