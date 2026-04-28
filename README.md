# Conveyor Belt Control System

This project contains the code for running a conveyor belt. It is a student project for the CPS course at Tec de Monterrey.

This is an ESP-IDF project developed in VS Code. Most of the components are based on the official ESP-IDF drivers.
The main logic is implemented using ESP-IDF components. The main application file initializes the components and handles basic debug printing.

## Current Goals

- Add support for an inductive sensor
- Implement the complete FSM (Finite State Machine) logic
- Improve documentation (generate better Doxygen output)
- Develop more robust components (reduce issues caused by polling functions)
- Improve debug output by using consistent logging within the components themselves
- Make digital_inputs more fexible (not so static, more options like diffrent debounce times)
  -FIX BDC Motor Driver. At the Moment the PID control and set_pwm cant work parrallel. I had to coment the pid control to make it work. (quick and dirty fix)

## Hardware Requirements

This project targets a **NodeMCU ESP32-S** development board. Currently, no external hardware is attached, but the planned basic pinout is structured as follows:

| Component                | Pin (Example/TBD) | Description                                              |
| :----------------------- | :---------------- | :------------------------------------------------------- |
| BDC Motor PWM A          | GPIO 13           | PWM signal for forward direction                         |
| BDC Motor PWM B          | GPIO 15           | PWM signal for reverse direction                         |
| Encoder / Decoder (opt.) | Not connected     | Rotary encoder inputs (PCNT). External decoder optional. |
| Inductive Sensor         | GPIO 34           | Detects metallic objects                                 |
| Lightgate START          | GPIO 22           | Light sensor at conveyor start (safety / position)       |
| Lightgate END            | GPIO 23           | Light sensor at conveyor end (safety / position)         |
| Handguards / Buttons     | 21,18,19,5        | Right(18), Left(19), Reset(21), Emergency(5)             |
| LED Control              | GPIO 4            | Status LED                                               |
| Servo PWM                | GPIO 27           | PWM signal for servo control                             |

_(Note: The exact GPIO assignment is subject to change as hardware integration progresses.)_
It can always be changed later in the Kconfig.

## Architecture and Dependencies

The project relies on the official ESP-IDF framework and uses modular components (`components/`) for encapsulation.

### External Dependencies

- `espressif/bdc_motor`: Brushed DC motor driver
- `espressif/pid_ctrl`: PID controller for speed/position regulation

### Main FSM States

The system logic is governed by a Finite State Machine (`main_fsm.c`) with the following primary states:

- **IDLE**
- **BELT_FORWARD**
- **BELT_REVERSE**
- **BELT_STOPPED**
- **UNSECURE_HANDS**
- **ERROR**

## Build, Flash, and Monitor

This project is built using the **ESP-IDF VS Code Extension**.
To build, flash, and monitor the project, simply use the ESP-IDF toolbar located at the bottom of the VS Code window:

- ⚙️ **Build:** Compiles the project.
- ⚡ **Flash:** Uploads the compiled binary to the NodeMCU ESP32-S.
- 🖥️ **Monitor:** Opens the serial monitor to view debug outputs.

## Documentation

The code is documented using Doxygen. To generate the HTML documentation:

1. Ensure Doxygen is installed on your system.
2. Open a terminal in the project root directory.
3. Run the following command:
   ```
   doxygen Doxyfile
   ```
4. Open the generated `docs/html/index.html` file in your browser to view the documentation.
