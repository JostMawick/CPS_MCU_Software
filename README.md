# Conveyor Belt Control System

This project contains the code for running a conveyor belt. It is a student project for the CPS course at Tec de Monterrey.
The main logic is implemented using ESP-IDF components. The main application file initializes the components and handles basic debug printing.

## Current Goals

- Add support for an inductive sensor
- Implement the complete FSM (Finite State Machine) logic
- Improve documentation (generate better Doxygen output)
- Develop more robust components (reduce issues caused by polling functions)
- Transition the FSM from polling to an event-based architecture
- Improve debug output by using consistent logging within the components themselves
