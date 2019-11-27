// BLE RX app
//
// Receives BLE advertisements with data

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_util.h"
#include "simple_ble.h"

// BLE configuration
// This is mostly irrelevant since we are scanning only
static simple_ble_config_t ble_config = {
        // BLE address is c0:98:e5:49:00:00
        .platform_id       = 0x49,    // used as 4th octet in device BLE address
        .device_id         = 0x0005,  // Last two octets of device address
        .adv_name          = "EE149", // irrelevant in this example
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS), // send a packet once per second (minimum is 20 ms)
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS), // irrelevant if advertising only
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS), // irrelevant if advertising only
};
simple_ble_app_t* simple_ble_app;

// TODO: implement BLE advertisement callback
void ble_evt_adv_report(ble_evt_t const* p_ble_evt) {
  // ble_gap_addr_t address = p_ble_evt->evt.gap_evt.params.adv_report.peer_addr;
  // Flic button macs : 80:e4:da:73:b4:50, 80:e4:da:73:b6:8d
  // if (address.addr[0] == 0x8D && address.addr[5] == 0x80 && address.addr[3] == 0xDA) {
  //   //printf("%x:%x:%x:%x:%x:%x\n", address.addr[0], address.addr[1], address.addr[2], address.addr[3], address.addr[4], address.addr[5]);
  //   ble_data_t data = p_ble_evt->evt.gap_evt.params.adv_report.data;
  //   uint8_t* data_addr = data.p_data;
  //   while (data_addr[1] != 0xFF) {
  //     data_addr += data_addr[0] + 1;
  //     printf("Type: 0x%x, Data: 0x %x %x\n", data_addr[1], data_addr[2], data_addr[3]);
  //   }
  //   printf("Type: 0x%x, Manufacturer: 0x %x %x\n", data_addr[1], data_addr[2], data_addr[3]);
  // }
}

// Flic button 1
//f02adfc0-26e7-11e4-9edc-0002a5d5c51b
static simple_ble_service_t robot_service = {{
    .uuid128 = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xdc,0x9e,
                0xe4,0x11,0xe7,0x26,0xc0,0xdf,0x2a,0xf0}
}};

int main(void) {

  // Setup BLE
  // Note: simple BLE is our own library. You can find it in `nrf5x-base/lib/simple_ble/`
  simple_ble_app = simple_ble_init(&ble_config);
  advertising_stop();

  // TODO: Start scanning
  scanning_start();

  while(1) {
    // Sleep while SoftDevice handles BLE
    power_manage();
  }
}