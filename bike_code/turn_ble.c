/**
 * To figure out how to use BLE, we heavily relied on example code
 * provided by Nordic's SDK, mainly in examples/ble_central/multilink,
 * that can be found at:
 * https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v13.x.x/
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "turn_ble.h"

#include "nrf_delay.h"

#define APP_BLE_CONN_CFG_TAG      1                                     /**< Tag that refers to the BLE stack configuration that is set with @ref sd_ble_cfg_set. The default tag is @ref APP_BLE_CONN_CFG_TAG. */
#define APP_BLE_OBSERVER_PRIO     3                                     /**< BLE observer priority of the application. There is no need to modify this value. */

#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50)                   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */

static char const m_target_periph_name[] = "iTAG            ";             /**< Name of the device to try to connect to. This name is searched for in the scanning report data. */
static char const app_name[] = "LIL";

uint8_t left_connected = 0;
uint16_t left_conn_handle = 999;
uint16_t left_char_handle = 0;
uint16_t left_num_handles = 0;

uint8_t right_connected = 0;
uint16_t right_conn_handle = 999;
uint16_t right_char_handle = 0;
uint16_t right_num_handles = 0;

uint8_t app_connected = 0;
uint16_t app_conn_handle = 999;
uint16_t app_info_handle = 0;
uint16_t app_num_handles = 0;

// booleans representing whether or not left and right buttons have been pressed
// main must call reset to say it has handled the button press
bool left_is_pressed = false;
bool right_is_pressed = false;
uint8_t app_info = 0;

static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
            printf("CONNECTED!\n");
        } break;
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            printf("NRF_BLE_SCAN_EVT_CONNECTING_ERROR\n");
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            break;
    }
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, app_name);
    APP_ERROR_CHECK(err_code);

    printf("Filters set!\n");
}


/**@brief Function for starting scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    printf("Start scanning for BLE buttons (%s) and app (%s).\n", (uint32_t)m_target_periph_name, (uint32_t)app_name);
    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_READ_RSP:
        {
          ble_gattc_evt_t const * p_gattc_evt = &p_ble_evt->evt.gattc_evt;
          ble_gattc_evt_read_rsp_t value_read = p_gattc_evt->params.read_rsp;



          uint8_t ble_data = value_read.data[0];
          
		  if (p_gattc_evt->conn_handle == left_conn_handle || p_gattc_evt->conn_handle == right_conn_handle) {
			  if (ble_data == 1) {
					if (!left_is_pressed && p_gattc_evt->conn_handle == left_conn_handle) {
						printf("LEFT BUTTON PRESSED!\n");
						left_is_pressed = true;
					} else if (!right_is_pressed && p_gattc_evt->conn_handle == right_conn_handle) {
						printf("RIGHT BUTTON PRESSED!\n");
						right_is_pressed = true;
					}
					
					// RESET
					ble_gattc_write_params_t write_params;
					uint8_t value_to_write[1] = {0x00};       

					write_params.write_op = BLE_GATT_OP_WRITE_REQ;                      
					write_params.handle   = left_char_handle;             
					write_params.offset   = 0;                                                          
					write_params.len      = 1;                                                              
					write_params.p_value  = value_to_write;                                                 
					
					ret_code_t err_code = sd_ble_gattc_write(p_gap_evt->conn_handle, &write_params);
					APP_ERROR_CHECK(err_code);
				}
		  // Else, reading from our app
		  } else {
            printf("Value len: %i\n", value_read.len);
            printf("Values read: %d\n", value_read.data[0]);
              printf("Values read: %d\n", value_read.data[1]);
              printf("num handles!lasdjg %d\n", app_num_handles);
              
		  	app_info = ble_data;
		  	err_code = sd_ble_gap_disconnect(app_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
		  }
        } break;

        // Upon connection, check which peripheral is connected, initiate DB
        // discovery, update LEDs status, and resume scanning, if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            printf("Connection 0x%x established, starting DB discovery.\n",
                         p_gap_evt->conn_handle);

            APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

            ble_uuid_t btn_uuid;

            btn_uuid.type = BLE_UUID_TYPE_BLE;
            btn_uuid.uuid = 0xffe0;
            err_code = ble_db_discovery_evt_register(&btn_uuid);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc,
                                              p_gap_evt->conn_handle);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }

            if (ble_conn_state_central_conn_count() != NRF_SDH_BLE_CENTRAL_LINK_COUNT) {
                scan_start();
            }

        } break;

        // Upon disconnection, reset the connection handle of the peer that disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            printf("LBS central link 0x%x disconnected (reason: 0x%x)\n",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
            if (p_gap_evt->conn_handle == left_conn_handle) {
                printf("LEFT DISCONNECTED\n");
                left_connected = 0;
                left_conn_handle = 999;
                left_char_handle = 0;
                left_num_handles = 0;
			} else if (p_gap_evt->conn_handle == app_conn_handle) {
                printf("APP DISCONNECTED\n");
                app_connected = 0;
                app_conn_handle = 999;
                app_info_handle = 0;
				app_num_handles = 0;
            } else if (p_gap_evt->conn_handle == right_conn_handle){
                printf("RIGHT DISCONNECTED\n");
                right_connected = 0;
                right_conn_handle = 999;
                right_char_handle = 0;
                right_num_handles = 0;
            } else {
                printf("DISCONNECTED FROM UNKNOWN HANDLE\n");
            }

            // Start scanning.
            scan_start();
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // Timeout for scanning is not specified, so only the connection requests can time out.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST.");
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT client timeout event.
            NRF_LOG_DEBUG("GATT client timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT server timeout event.
            NRF_LOG_DEBUG("GATT server timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    printf("DISC_HANDLER CALLED!\n");
    if (p_evt->evt_type == BLE_DB_DISCOVERY_SRV_NOT_FOUND) {
      printf("Something went wrong, Service not found\n");
    }

	// Button service has one characteristic
	if (p_evt->params.discovered_db.char_count == 1) {
		if (!left_connected) {
			left_connected = 1;
			left_conn_handle = p_evt->conn_handle;
			left_char_handle = p_evt->params.discovered_db.charateristics[0].characteristic.handle_value;
			left_num_handles = p_evt->params.discovered_db.char_count;
			printf("LEFT:\n Conn_handle: %x\nChar_handles: %x\nNum_handles: %i\n", left_conn_handle, left_char_handle, left_num_handles);
		} else {
			right_connected = 1;
			right_conn_handle = p_evt->conn_handle;
			right_char_handle = p_evt->params.discovered_db.charateristics[0].characteristic.handle_value;
			right_num_handles = p_evt->params.discovered_db.char_count;
			printf("RIGHT:\n Conn_handle: %x\nChar_handles: %x\nNum_handles: %i\n", right_conn_handle, right_char_handle, right_num_handles);
		}
	// App service four characteristics
	} else if (p_evt->params.discovered_db.char_count == 4) {
		app_connected = 1;
		app_conn_handle = p_evt->conn_handle;
		app_info_handle = p_evt->params.discovered_db.charateristics[0].characteristic.handle_value;
		app_num_handles = p_evt->params.discovered_db.char_count;
		printf("APP:\n Conn_handle: %x\nInfo handle: %x\nNum_handles: %i\n", app_conn_handle, app_info_handle, app_num_handles);
	}
}


/** @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function to be called every time in the loop to see if a button has been pressed
 */
void sample_buttons() {
    if (right_num_handles > 0) {
      ret_code_t err_code = sd_ble_gattc_read(right_conn_handle, right_char_handle, 0);
      if (err_code != NRF_ERROR_BUSY) {
        APP_ERROR_CHECK(err_code);
      }
    }

    if (left_num_handles > 0) {
      ret_code_t err_code = sd_ble_gattc_read(left_conn_handle, left_char_handle, 0);
      if (err_code != NRF_ERROR_BUSY) {
        APP_ERROR_CHECK(err_code);
      }
    }
}

/**@brief Function to be called every time in the loop to update app info
 */
void sample_app() {
    if (app_num_handles > 0 && ble_conn_state_valid(app_conn_handle)) {
      ret_code_t err_code = sd_ble_gattc_read(app_conn_handle, app_info_handle, 0);
      if (err_code != NRF_ERROR_BUSY) {
      //  APP_ERROR_CHECK(err_code);
      }
    }
}


/**@brief Calls all the necessary inits to get BLE working
 */
void ble_init(void)
{
    // Initialize.
    ble_stack_init();
    gatt_init();
    db_discovery_init();
    ble_conn_state_init();
    scan_init();

    // Start execution.
    printf("BLE connection started.\n");
    scan_start();
}

// if the left button has been pressed
bool get_left_pressed(void)
{
    return left_is_pressed;
}
// if the right button has been pressed
bool get_right_pressed(void)
{
    return right_is_pressed;
}

uint8_t get_app_info(void)
{
	return app_info;
}

// let's turn_ble.c know the button press has been read
void reset_left_button(void)
{
    left_is_pressed = false;
}
void reset_right_button(void)
{
    right_is_pressed = false;
}
