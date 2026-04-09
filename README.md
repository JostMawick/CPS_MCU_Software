# Conveyor Belt Control System

This project contains the code for running a conveyor belt. It is a student project for the CPS course at Tec de Monterrey.

This is an ESP-IDF project developed in VS Code. Most of the components are based on the official ESP-IDF drivers.
The main logic is implemented using ESP-IDF components. The main application file initializes the components and handles basic debug printing.

## Current Goals

- Add support for an inductive sensor
- Implement the complete FSM (Finite State Machine) logic
- Improve documentation (generate better Doxygen output)
- Develop more robust components (reduce issues caused by polling functions)
- Transition the FSM from polling to an event-based architecture
- Improve debug output by using consistent logging within the components themselves

## Hardware Requirements

This project targets a **NodeMCU ESP32-S** development board. Currently, no external hardware is attached, but the planned basic pinout is structured as follows:

| Component        | Pin (Example/TBD) | Description                      |
| :--------------- | :---------------- | :------------------------------- |
| BDC Motor PWM A  | `TBD`             | PWM signal for forward direction |
| BDC Motor PWM B  | `TBD`             | PWM signal for reverse direction |
| Encoder A        | `TBD`             | PCNT input A                     |
| Encoder B        | `TBD`             | PCNT input B                     |
| Inductive Sensor | `TBD`             | Detects metallic objects         |
| Light Barrier    | `TBD`             | Conveyor belt limits / safety    |
| User Buttons     | `TBD`             | Up, Down, Stop controls          |

_(Note: The exact GPIO assignment is subject to change as hardware integration progresses.)_

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
