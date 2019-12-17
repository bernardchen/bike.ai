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
#define SAMPLE_SIZE 50

// ultrasonic ranger proximity constants
#define LOOP_HOLD_AMOUNT (9) // number of loops that a ultrasonic ranger needs to be held for it to change
#define DIST_THRESHOLD (250) // the distance an object has to be within to turn on the sensor (in cm)

// turn signal constants
#define TURN_DETECTED_ACCEL_THRESH (0.4)

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
  lfclk_request();
  ret_code_t error_code = app_timer_init();
  APP_ERROR_CHECK(error_code);
}

// timer stuff for timeout
// Callback function for timer
uint32_t turn_time_on = 0; // turn_time_on used for turn signal state machine
uint32_t button_press_time = 0;
uint32_t brake_time_on = 0; // brake_time_on used for brake state machine
void main_timer_callback() {
  turn_time_on++;
  button_press_time++;
  brake_time_on++;
}

void init_main_timer() {
  app_timer_create(&main_timer, APP_TIMER_MODE_REPEATED, (app_timer_timeout_handler_t) main_timer_callback);
  app_timer_start(main_timer, APP_TIMER_TICKS(1000), NULL); // 1000 milliseconds
  printf("Timer initialized\n");
}

void init_button0() {
  gpio_config(28, 0);
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

void sample_9250_accelerometer(float* x_axis, float* y_axis, float* z_axis) {
  mpu9250_measurement_t accelerometer_measurement = mpu9250_read_accelerometer();
  *x_axis = accelerometer_measurement.x_axis;
  *y_axis = accelerometer_measurement.y_axis;
  *z_axis = accelerometer_measurement.z_axis;
}



/************************* PWM Stuff for the buggy LEDs *************************/
// Inspired by simple_pwm example in nRF forums
#define OUTPUT_PIN (26)
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);

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


// Set duty cycle between 0 and 100%
void pwm_update_color(uint8_t color)
{
    if (color == 0){
        // We perform playback for red
        nrf_drv_pwm_simple_playback(&m_pwm0, &red_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
    } else if (color == 1){
        // We perform playback for green
        nrf_drv_pwm_simple_playback(&m_pwm0, &green_seq, 40, NRF_DRV_PWM_FLAG_STOP);
    } else {
        // We assume they want to play yellow
        nrf_drv_pwm_simple_playback(&m_pwm0, &green_seq, 40, NRF_DRV_PWM_FLAG_STOP);
        nrf_drv_pwm_simple_playback(&m_pwm1, &green_seq, 40, NRF_DRV_PWM_FLAG_STOP);


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
    nrf_drv_pwm_config_t const config1 =
       {
           .output_pins =
           {
               2,
               
           },
           .base_clock   = NRF_PWM_CLK_16MHz,
           .count_mode   = NRF_PWM_MODE_UP,
           .top_value    = 21,
           .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
           .step_mode    = NRF_PWM_STEP_AUTO
       };
       APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm1, &config1, NULL));
}
void led_init(void){
    NRF_CLOCK->TASKS_HFCLKSTART = 1; 
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
        ;
    pwm_init();
    pwm_update_color(1);
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
// need to fix a bit for both of these functions residual detected function
// standard deviation and mean based method1
/*bool detected(void){
    double x;double y;double z;
    sample_accel(&x,&y,&z);
    if (x > 0.0){
        return false;
    }
    length = 0
    if (counter < 100){
        length = counter/2
 }
    values[counter] = (x*-1-values[]);
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
 */


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
  /********* turn ble stuff *********/
  ble_init();
  /********* ultrasonic ranger stuff *********/
  init_ultrasonic_ranger(A, D, 1);
  // LEDs for output of something nearby
  // TODO: CHANGE WHEN HAVE REAL LEDs
  // nrf_gpio_cfg_output(BUCKLER_LED0);
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

  // variable just for proximity sensor so that false reads don't change 
  uint16_t num_in_a_row_left = 0;
  uint16_t num_in_a_row_right = 0;

  // Loop forever
  while (1) {
    // Determines sampling rate
    // TODO: Figure out how to to dealsy because ultrasonic_ranger can't delay more than 1ms
  	uint32_t start_count = app_timer_cnt_get();
  	nrf_delay_ms(1);

    for (int i=0; i<3; i++) {
      nrf_gpio_pin_toggle(LEDS[i]);
    }
    pwm_update_color(1);

    // GET MEASUREMENTS AND INPUTS
    /************************************** TURNING **************************************/
    float x_acc, y_acc, z_acc;
    sample_9250_accelerometer(&x_acc, &y_acc, &z_acc);
    bool turned_left = (y_acc > TURN_DETECTED_ACCEL_THRESH), turned_right = (y_acc < -TURN_DETECTED_ACCEL_THRESH);
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
    bool left_turn, right_turn = false;
    /************************************** TURNING **************************************/
    uint16_t brake_type = 0;
    uint16_t brake_color = 0;
    uint16_t turn_color = 0;
    uint16_t motion_dist = 0;
    sample_app();
    brake_type = get_brake_type();
    brake_color = get_brake_color();
    turn_color = get_turn_color();
    motion_dist = get_motion_dist();
    printf("braketype %x, brakecolor %x, turncolor %x, motiondist %x\n", brake_type, brake_color, turn_color, motion_dist);

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
    switch(brake_state) {
      case OFF: {
        //printf("Brake light is OFF\n");
        pwm_update_color(2); // 2 is off
        if (detected())
        {
          printf("DETECTED BRAKE\n");
          brake_time_on = 0;
          brake_state = ON;
        }
        break;
      }
      case ON: {
        //printf("Brake light is ON\n");
        if (brake_time_on > 2 && !detected())
        {
          printf("Turning off brake lights\n");
          brake_time_on = 0;
          brake_state = OFF;
        }
        pwm_update_color(1); // green
        break;
      }
    }
    counter +=1;
    counter = counter % SAMPLE_SIZE;

    // State machine for proximity sensors
    // will only change if there are LOOP_HOLD_AMOUNT times in a row
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

      	if (num_in_a_row_left >= LOOP_HOLD_AMOUNT && left_range <= DIST_THRESHOLD) // already happened 4 times and fifth is also true
      	{
      		num_in_a_row_left = 0;
      		left_proximity_state = ON;
      		turn_on_left_proxi_led();
      	}	
        break;
      }
      case ON: {
      	if (left_range > DIST_THRESHOLD)
      	{
      		num_in_a_row_left += 1;
      	}
      	else
      	{
      		num_in_a_row_left = 0;
      	}
      	if (num_in_a_row_left >= LOOP_HOLD_AMOUNT && left_range > DIST_THRESHOLD)
      	{
      		num_in_a_row_left = 0;
      		left_proximity_state = OFF;
      		turn_off_left_proxi_led();
      	}
        break;
      }
    }
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

      	if (num_in_a_row_right >= LOOP_HOLD_AMOUNT && right_range <= DIST_THRESHOLD) // already happened 4 times and fifth is also true
      	{
      		num_in_a_row_right = 0;
      		right_proximity_state = ON;
      		turn_on_right_proxi_led();
      	}	
        break;
      }
      case ON: {
      	if (right_range > DIST_THRESHOLD)
      	{
      		num_in_a_row_right += 1;
      	}
      	else
      	{
      		num_in_a_row_right = 0;
      	}
      	if (num_in_a_row_right >= LOOP_HOLD_AMOUNT && right_range > DIST_THRESHOLD)
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
          left_turn = true;
          turn_time_on = 0;
        } else if (ble_right) {
          turn_state = RIGHT;
          right_turn = true;
          turn_time_on = 0;
        } else {
          turn_state = OFF;
         // printf("IN STATE OFF\n");
        }
        break;
      }
      case LEFT: {
        if (ble_left || turn_time_on > 60 || turned_left) {
          printf("ble_left: %i, turn_time: %i, turned_left: %i\n", ble_left, turn_time_on > 60, turned_left);
          turn_state = OFF;
        } else if (ble_right) {
          turn_state = RIGHT;
          right_turn = true;
          turn_time_on = 0;
        } else {
          turn_state = LEFT;
          left_turn = true;
          printf("IN STATE LEFT\n");
        }
        break;
      }
      case RIGHT: {
        if (ble_right || turn_time_on > 60 || turned_right) {
          turn_state = OFF;
        } else if (ble_left) {
          turn_state = LEFT;
          left_turn = true;
          turn_time_on = 0;
        } else {
          turn_state = RIGHT;
          right_turn = true;
          printf("IN STATE RIGHT\n");
        }
        break;
      }
    }

    // Handle turn_state outputs
    if (left_turn) {
      // turn on left turn lights
    } else if (right_turn) {
      // turn on right turn lights
    }

    uint32_t end_count = app_timer_cnt_get();
    uint32_t time_microsec = app_timer_ticks_to_usec(end_count - start_count);
   // printf("Time to run loop: %d\n", time_microsec);
  }
}
