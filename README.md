AXON 3-Axis CNC Controller Firmware

Primary Author: Shivam Garg
Research Group: Placid Ferreira Research Group
Institution: University of Illinois Urbana-Champaign

This code was developed for academic research purposes.
Please notify the author before reusing or modifying.
Shared under academic collaboration - (c) 2025

Acknowledgements

Aryan Shroff for contributions in setting up pin definitions for encoders, limit switches, PWM signal outputs, and homing sequence logic.

Code Overview

This firmware runs a 3-axis CNC machine on a Teensy 4.1, acting as the real-time motion controller that directly interfaces with hardware. It coordinates with a Raspberry Pi, which serves as a data buffer and high-level manager (Cloud NC backend).

Key Supported Functions

Ethernet Communication
Establishes a persistent TCP connection with the Raspberry Pi server for receiving commands and sending back log data.

Runtime Configuration
Enters configuration mode on startup, waiting for the server to send all essential control parameters such as PI gains, control loop frequency, and data logging settings.

Closed-Loop PI Control
Implements a high-frequency Proportional-Integral (PI) control loop for the X, Y, and Z axes using feedback from quadrature encoders.

Advanced Control Features
Uses direction-dependent gains and a "friction kick" mechanism to overcome static friction and improve tracking accuracy.

Command Stream Processing
Processes a custom, compact binary command stream from a lock-free queue.

High-Speed Data Logging
Captures real-time data (target vs. actual position) in a large circular buffer to avoid blocking the main control loop. Logged data is transmitted back to the server in efficient chunks.

Hardware Interfacing
Manages PWM signals for motor drivers, reads limit switches, and handles an emergency stop interrupt.

Homing Sequence
Automated routine to establish a machine-zero reference point for the X, Y, and Z axes.
