/**
 * To figure out how to use BLE, we heavily relied on example code
 * provided by Nordic's SDK, mainly in examples/ble_central/multilink,
 * that can be found at:
 * https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v13.x.x/
 */

// Inits all of the stuff needed for ble
void ble_init(void);

// sample the left/right turn buttons
// function needs to be called repeatedly in the loop to see if a button has been pressed
void sample_buttons();

// get whether or not left and right buttons are pressed
bool get_left_pressed(void);
bool get_right_pressed(void);

// Let's turn_ble.h know that the code using this
// has gotten the button press so it is ready to
// read another press
void reset_left_button(void);
void reset_right_button(void);

// sample the app information
void sample_app();

// get the app info information
uint8_t get_app_info(void);
