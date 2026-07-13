# **WORK IN PROGRESS**

# DRV8214 I2C Control Library

## Overview

This library provides a set of straightforward commands to control and configure the **DRV8214** brushed DC motor driver from Texas Instruments via the I2C interface. The DRV8214 is a high-performance integrated H-bridge motor driver featuring speed and position detection through ripple counting, along with additional functionalities such as motor speed and voltage regulation, stall detection, current sensing, and protection circuitry.

## Features

- **I2C Communication**: Seamlessly interface with the DRV8214 over I2C for efficient motor control.
- **Motor Control**: Configure easily the various regulation and control modes of the driver.
- **Motion Control**: Basic motion functions like turnForward to more advanced ones like turnXRipples or turnXRotations. 
- **Configuration Settings**: Access and modify device parameters to suit specific application requirements.
- **Status Monitoring**: Retrieve real-time data on motor performance and fault conditions.

## Getting Started

### Prerequisites

- **Hardware**: A microcontroller with I2C capability (e.g., Arduino, ESP32, STM32) and the DRV8214 motor driver.
- **Software**: Arduino IDE or PlatformIO for code development and uploading.

### Installation

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/DRV8214_I2C_Library.git
   ```

2. **Include the Library**: Add the cloned library to your project's libraries directory.

### Usage


```cpp
#include <DRV8214.h>

#define IPROPI_RESISTOR 1000 // Value in Ohms of the resistor connected to IPROPI pin
#define NUM_RIPPLES 156 // Number of current ripples per output shaft revolution 

// Initialize the driver with the I2C address, driver ID, and hardware dependant values
DRV8214 motorDriver(0x60, 0, IPROPI_RESISTOR, NUM_RIPPLES);

// Create a configuration struct for the driver
DRV8214_Config driver_config;

void setup() {
    Wire.begin();
    motorDriver.init(driver_config);
}

void loop() {
    // Your code here
    // example for Arduino/ESP32
    motorDriver.turnXRipples(50000, true, true, 1);
    delay(5000);
    motorDriver.turnXRipples(50000, false, true, 1);
    delay(5000);
}
```


## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

## Acknowledgments

Special thanks to the open-source community and Texas Instruments for their comprehensive documentation and support.

---

*Note: This library is under active development. Features and implementations are subject to change. Users are encouraged to regularly update and refer to the latest documentation.* 
