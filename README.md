# Notes


git clone https://github.com/Agroecology-Lab/lizard

cd lizard

ls /dev/tty* (look for /dev/ttyUSB0 or /dev/ttyACM0).

docker run -it -v ~/lizard:/project --device=/dev/ttyACM0 espressif/idf:release-v5.5

./build.py esp32s3 --clean

./espresso.py -d flash /dev/ttyACM0 

./espresso.py --chip esp32s3 flash /dev/ttyACM0


# Lizard: Hardware Control Language

Lizard is a domain-specific language designed for defining and controlling the behavior of hardware components on embedded systems. It is optimized to run on microcontrollers connected to peripherals such as motor controllers, sensors, and actuators. Lizard is frequently used in conjunction with higher-level control systems like ROS (Robot Operating System) or RoSys, serving as the low-level "lizard brain" of a machine to ensure basic safety and execute time-critical actions with high reliability [1].

The primary goal of Lizard is to replace the need for compiling and deploying custom C++ code for every hardware configuration change. Instead, it provides a flexible, text-based language that allows for on-the-fly modifications to hardware behavior, significantly accelerating development and iteration cycles.

## Key Features

Lizard offers a range of features designed to simplify embedded hardware control and development.

| Feature | Description |
| :--- | :--- |
| **Interactive Shell** | Provides a shell-like interface over serial communication for easy development and debugging. |
| **High Performance** | Ensures quick and resource-efficient execution on embedded hardware. |
| **Data Integrity** | Includes built-in checksums to detect and prevent transmission errors. |
| **Continuous Safety** | Supports safety conditions that are checked continuously to prevent hazardous states. |
| **Distributed Control** | Has the ability to work across multiple interconnected microcontrollers. |
| **Readable Syntax** | Features a human-readable syntax that is easy to learn and type, similar to Python. |
| **Persistent Configuration** | Allows for "startup" commands that are persistently applied after every boot. |

## Supported Hardware

Lizard is designed to run on Espressif microcontrollers and supports a wide array of hardware modules out of the box.

### Supported Microcontrollers

Lizard is primarily developed and tested for the following Espressif chips:

*   **ESP32**
*   **ESP32-S3**

The project is built using the **ESP-IDF v5.3.1** development framework [2].

### Hardware Modules

Lizard includes a rich library of modules for interfacing with various hardware components. The table below provides a summary of the available modules [3].

| Category | Module | Description |
| :--- | :--- | :--- |
| **Core & System** | `Core` | Provides access to microcontroller-level functions like time, heap, and restart. |
| | `Bluetooth` | Enables communication over Bluetooth Low Energy (BLE). |
| | `Input` / `Output` | Controls digital GPIO pins for inputs (buttons) and outputs (LEDs). |
| | `PwmOutput` | Generates PWM signals for controlling servos or dimming LEDs. |
| | `AnalogInput` | Reads analog voltage levels from ADC pins. |
| **Communication** | `CAN` | Manages communication over the CAN bus. |
| | `Serial` | Provides a standard UART serial interface. |
| | `Mcp23017` | Controls an I2C-based 16-pin port expander. |
| **Sensors** | `IMU` | Interfaces with an Inertial Measurement Unit for orientation and motion data. |
| **Motor Control** | `ODriveMotor` | Controls ODrive motor controllers. |
| | `RmdMotor` | Controls RMD series smart motors. |
| | `RoboClaw` | Interfaces with RoboClaw motor controllers. |
| | `StepperMotor` | Provides control for standard stepper motors. |
| **CANopen** | `CanOpenMaster` | Acts as a master node in a CANopen network. |
| | `CanOpenMotor` | Controls CANopen-compatible motors. |
| | `DunkerMotor` | Specifically supports Dunker motors via CANopen. |

## Getting Started

This guide will walk you through setting up your environment and installing Lizard on a supported microcontroller.

### Prerequisites

*   An **Espressif ESP32 or ESP32-S3** development board.
*   **Python 3** and `pip` installed on your computer.
*   **Git** for cloning the repository.
*   **USB to UART Bridge VCP Drivers** from Silicon Labs to ensure proper serial communication [4].

### Installation Steps

1.  **Clone the Repository**
    Open your terminal and clone the Lizard GitHub repository:
    ```sh
    git clone https://github.com/zauberzeug/lizard.git
    cd lizard
    ```

2.  **Install Dependencies**
    Install the required Python packages using `pip`:
    ```sh
    python3 -m pip install -r requirements.txt
    ```

3.  **Initialize Submodules**
    Lizard uses Git submodules for some of its dependencies. Initialize them with:
    ```sh
    git submodule update --init --recursive
    ```

4.  **Download and Flash Firmware**
    Download the latest release from the [releases page](https://github.com/zauberzeug/lizard/releases). Unpack the `lizard_firmware_and_devtools_vX.Y.Z.zip` file. Then, connect your ESP32 board and flash the firmware using the `espresso.py` script. Make sure to replace `/dev/ttyUSB0` with the correct serial device path for your system.
    ```sh
    sudo ./espresso.py flash -d /dev/ttyUSB0
    ```

5.  **Verify the Installation**
    You can verify that Lizard is running correctly by connecting to the interactive monitor:
    ```sh
    ./monitor.py /dev/ttyUSB0
    ```
    Once connected, you can run a command like `core.info()` to see the firmware version and other system details.

## Development

If you wish to contribute to Lizard or build the firmware from source, follow these steps.

### Building from Source

After making changes to the C++ implementation or the language definition (`language.owl`), you can compile the firmware using the provided script. This script utilizes a Docker container with the correct ESP-IDF environment.

```sh
./compile.sh
```

Once the build is complete, you can flash the newly compiled `build/lizard.bin` file using the `espresso.py` script as described in the installation steps.

### Debugging

If the device crashes, it may produce a backtrace. The `backtrace.sh` script can be used to translate the memory addresses into readable source code locations.

```sh
./backtrace.sh <addresses>
```

## Documentation

For a complete language reference, detailed module APIs, and further examples, please refer to the official documentation at **[lizard.dev](https://lizard.dev)**.

## License

Lizard is open-source software released under the **MIT License** [5].

## References

[1] Zauberzeug GmbH. (2025). *Lizard GitHub Repository*. [https://github.com/zauberzeug/lizard](https://github.com/zauberzeug/lizard)
[2] Zauberzeug GmbH. (2025). *Update Lizard to IDF 5.3.1*. [https://github.com/zauberzeug/lizard/pull/68](https://github.com/zauberzeug/lizard/pull/68)
[3] Zauberzeug GmbH. (2025). *Lizard Module Reference*. [https://lizard.dev/module_reference/](https://lizard.dev/module_reference/)
[4] Silicon Labs. (2025). *USB to UART Bridge VCP Drivers*. [https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
[5] Zauberzeug GmbH. (2021). *Lizard License*. [https://github.com/zauberzeug/lizard/blob/main/LICENSE](https://github.com/zauberzeug/lizard/blob/main/LICENSE)
