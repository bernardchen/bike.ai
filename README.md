
# Bike.ai
Bike.ai is a modular system providing non-intrusive brake and turn lighting, “blind-spot” proximity detection, and an accompanying iOS application. The system is designed with non-intrusiveness in mind, so all components are mostly wireless and can be easily attached to any bicycle without extensive wiring and fits to any and all bicycles. Built for Fall 2019 EECS 149 Fall 2019 at UC Berkeley.

[See our demo](https://youtu.be/Q-gUB0No8FQ)

## Getting Started

This project is built with with nRF52x microcontroller with a 'Berkeley Buckler' development board attachment. Base code and the build system was borrowed from:
[https://github.com/lab11/nrf52x-base](https://github.com/lab11/nrf52x-base)
[https://github.com/lab11/buckler](https://github.com/lab11/buckler)

Cloning this repository and running `git submodule update --init --recursive` should pull in proper dependencies needed.

Run `make flash` in `/bike_code` to flash code onto the nRF board.

### Hardware required

- [nRF52 Development Kit](https://www.nordicsemi.com/Software-and-Tools/Development-Kits/nRF52-DK)
- [Berkeley Buckler](https://github.com/lab11/buckler)
- [Grove Ultrasonic Ranger](https://www.seeedstudio.com/Grove-Ultrasonic-Distance-Sensor.html) (x2)
- LEDs (x2)
- WS2812B LED Strips (x3)
- iTag BLE buttons (x2)


### Directory Structure
```
├── bike_code
│   └── <Main application code>
├── logs_and_data
│   ├── driverBehaviorDataset
│   │   └── <Braking analysis using ML>
│   ├── logs
│   │   └── <Test ride log files>
│   └── outlier_analysis
│       └── <Braking analysis using online outlier detection>
├── mobile
│   ├── EE149BIKE
│   │   └── <iOS application for customizing configurations with our device>
├── PDFs
│   └── <Documents>
├── boards
│   └── <Board-specific headers and Makefiles>
├── libraries
│   └── <Various libraries>
├── nrf5x-base
│   └── <Submodule: build tools and files for nRF projects>
└── media
    └── <Images used in repo>
```

## More information and pictures

The final report for our project can be found under `/PDFs/final_report.pdf`. It includes more specific implementation details. The project poster can be found under `/PDFs/poster.pdf`. Pictures can be found under `/media`.

## Contributors

Bernard Chen
Arjun Mishra
Michael Duong

## References
- [Adafruit NeoPixel Guide](https://learn.adafruit.com/adafruit-neopixel-uberguide/advanced-coding)
- [Grove Ultrasonic Ranger Wiki](http://wiki.seeedstudio.com/Grove-Ultrasonic_Ranger/)
- [nRF SDK v15.3.0](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.3.0%2Findex.html)
- [Reverse Engineering iTag BLE button](https://thejeshgn.com/2017/06/20/reverse-engineering-itag-bluetooth-low-energy-button/)

## Acknowledgments

* GSI Neal for project idea