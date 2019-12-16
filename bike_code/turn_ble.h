
// Inits all of the stuff needed for ble
void ble_init(void);

// sample the left/right turn buttons
// function needs to be called repeatedly in the loop to see if a button has been pressed
void sample_buttons();

// get whether or not left and right buttons are pressed
bool get_left_pressed(void);
bool get_right_pressed(void);

void reset_left_button(void);
void reset_right_button(void);