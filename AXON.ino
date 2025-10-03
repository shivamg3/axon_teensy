/********************************************************************************
 * @brief   Firmware for the AXON 3-Axis CNC Controller on Teensy 4.1
 * * ##############################################################################
 * # Primary Author: Shivam Garg
 * # Research Group: Placid Ferreira Research Group
 * # Institution: University of Illinois Urbana-Champaign
 * #
 * # This code was developed for academic research purposes.
 * # Please notify the author before reusing or modifying.
 * #
 * # Shared under academic collaboration - (c) 2025
 * ##############################################################################
 *
 * ---
 *
 * ## Acknowledgements
 *
 * - Aryan Shroff for his contributions in setting up the pin definitions for the encoders,
 * limit switches, and PWM signal outputs, and the homing sequence logic.
 *
 *
 * ## Code Details
 *
 * This firmware is designed to run a 3-axis CNC machine, acting as the real-time
 * motion controller that directly interfaces with hardware. It coordinates with a
 * Raspberry Pi, which serves as a data buffer and high-level manager (Cloud NC backend).
 * The system is built for the **Teensy 4.1** microcontroller.
 *
 * Key supported functions include:
 *
 * - **Ethernet Communication:** Establishes a persistent TCP connection with the
 * Raspberry Pi server for receiving commands and sending back log data.
 *
 * - **Runtime Configuration:** Enters a configuration mode on startup, waiting for
 * the server to send a packet containing all essential control parameters, such
 * as PI gains, control loop frequency, and data logging settings.
 *
 * - **Closed-Loop PI Control:** Implements a high-frequency Proportional-Integral (PI)
 * control loop for the X, Y, and Z axes using feedback from quadrature encoders.
 *
 * - **Advanced Control Features:** Utilizes direction-dependent gains and a
 * "friction kick" mechanism to overcome static friction and improve tracking
 * accuracy.
 *
 * - **Command Stream Processing:** Processes a custom, compact binary command stream
 * from a lock-free queue.
 *
 * - **High-Speed Data Logging:** Captures real-time data (target vs. actual position)
 * in a large circular buffer to avoid blocking the main control loop. Logged data
 * is transmitted back to the server in efficient chunks.
 *
 * - **Hardware Interfacing:** Manages PWM signals for motor drivers, reads limit
 * switches, and handles an emergency stop interrupt.
 *
 * - **Homing Sequence:** Includes an automated homing routine to establish a
 * machine-zero reference point for the X, Y and Z axes.
 *
 ********************************************************************************/

#include <NativeEthernet.h>
#include <IntervalTimer.h>
#include <Arduino.h>
#include <Encoder.h>

// --- Pin Definitions ---
const int STOP_BUTTON_PIN = 15;
// For X Axis
const int x_ENCODER_PHASE_A_PIN = 3;
const int x_ENCODER_PHASE_B_PIN = 2;
// For Y Axis
const int y_ENCODER_PHASE_A_PIN = 1;
const int y_ENCODER_PHASE_B_PIN = 0;
// For Z Axis
const int z_ENCODER_PHASE_A_PIN = 4;
const int z_ENCODER_PHASE_B_PIN = 5;

// For X Axis
const int x_MOTOR_FORWARD_PIN = 11;  // "motor ref +" - tells the motor to move in the positive direction.
const int x_MOTOR_REVERSE_PIN = 12; // "motor ref -" - tells the motor to move in the negative direction.
// For Y Axis
const int y_MOTOR_FORWARD_PIN = 13;  // "motor ref +" - tells the motor to move in the positive direction.
const int y_MOTOR_REVERSE_PIN = 14; // "motor ref -" - tells the motor to move in the negative direction.
// For Z Axis
const int z_MOTOR_FORWARD_PIN = 9;  // "motor ref +" - tells the motor to move in the positive direction.
const int z_MOTOR_REVERSE_PIN = 10; // "motor ref -" - tells the motor to move in the negative direction.

// Limit Switches
const int x_LIMIT_SWITCH_LEFT_PIN = 6;
const int x_LIMIT_SWITCH_RIGHT_PIN = 7;

//const int y_LIMIT_SWITCH_LEFT_PIN = 6;
//const int y_LIMIT_SWITCH_RIGHT_PIN = 7;

const int z_LIMIT_SWITCH_LEFT_PIN = 30;
const int z_LIMIT_SWITCH_RIGHT_PIN = 29 ;

/**
 * @brief Defines the structure of the configuration packet sent from the Pi.
 *
 * This struct groups all tunable parameters into a single, contiguous block of memory.
 * This makes it easy to send and receive the entire configuration in one efficient operation.
 * Using fixed-size types (e.g., uint32_t) ensures that the data structure has the
 * exact same size and layout on both the Raspberry Pi and the 32-bit Teensy.
 */
#pragma pack(push, 1)
struct ConfigurationPacket {
    // Controller settings
    uint32_t control_loop_interval_us;
    int32_t  error_deadband;
    int32_t  integral_active_zone;
    float    integral_max_sum;

    // --- Logging Settings ---
    uint8_t  enable_data_logging;
    uint16_t logging_frequency_multiplier;
    uint32_t batch_data_interval_us;

    // X-Axis Gains
    float kp_xf;
    float ki_xf;
    int32_t friction_kick_xf;
    float kp_xr;
    float ki_xr;
    int32_t friction_kick_xr;

    // Y-Axis Gains
    float kp_yf;
    float ki_yf;
    int32_t friction_kick_yf;
    float kp_yr;
    float ki_yr;
    int32_t friction_kick_yr;

    // Z-Axis Gains
    float kp_zf;
    float ki_zf;
    int32_t friction_kick_zf;
    float kp_zr;
    float ki_zr;
    int32_t friction_kick_zr;
};
#pragma pack(pop)

// =================================================================================
// CONFIGURABLE PARAMETERS
// These variables are now non-const and volatile so they can be changed at runtime
// by the configuration packet. They are initialized with default "safe" values.
// =================================================================================
volatile unsigned int CONTROL_LOOP_INTERVAL_US = 50;
volatile float CONTROL_LOOP_INTERVAL_S = 50 / 1000000.0f;
volatile uint32_t frequencyThreshold = 1000000 / 50;
const int PWM_RESOLUTION = 8;
const int MAX_PWM_VALUE = 255;


// --- PI Control Gains & Limits ---
volatile int ERROR_DEADBAND = 5;
volatile float KP_xf = 0.25;
volatile float KI_xf = 2.0;
volatile int FRICTION_KICK_xf = 80;
volatile float KP_xr = 0.25;
volatile float KI_xr = 2.0;
volatile int FRICTION_KICK_xr = 70;
volatile float KP_yf = 0.3;
volatile float KI_yf = 5.0;
volatile int FRICTION_KICK_yf = 80;
volatile float KP_yr = 0.3;
volatile float KI_yr = 5.0;
volatile int FRICTION_KICK_yr = 80;
volatile float KP_zf = 0.4;
volatile float KI_zf = 2.0;
volatile int FRICTION_KICK_zf = 85;
volatile float KP_zr = 0.6;
volatile float KI_zr = 2.0;
volatile int FRICTION_KICK_zr = 60;
volatile int INTEGRAL_ACTIVE_ZONE = 100;
volatile float INTEGRAL_MAX_SUM = 2000.0;

// NEW/MODIFIED VARIABLES FOR ETHERNET CONTROL
volatile bool DataLoggingMode = false;
volatile uint16_t logging_frequency_multiplier = 1; // Now a 16-bit integer for larger values
volatile uint16_t logging_counter = 0;              // Counter used in the ISR


// A flag to signal from the timer interrupt to the main loop.
// 'volatile' is important because it's used by an interrupt.
volatile bool send_batchdata_1 = false; // <-- ADD THIS FLAG
volatile unsigned int BATCH_DATA_INTERVAL_US = 1000000;


// --- Status Printing ---
unsigned long lastStatusPrintMicros = 0;
const unsigned long STATUS_PRINT_INTERVAL_US = 1000000; // 1 second
// =================================================================================
// CORE DATA STRUCTURES (structs.h)
// =================================================================================

// --- Generic Queue and Data Packet Definition ---
const int STRUCT_QUEUE_LEN = 32; // Must be a power of 2 for efficient modulo
const int STRUCT_SIZE = 5120;   // n4KB as requested
const int LOG_QUEUE_SIZE = 150 * 1024; // 150KB buffer
const int SEND_CHUNK_SIZE = 50 * 1024;  // Send data in 50KB chunks
const int QUEUE_REQUEST_THRESHOLD = 8;
const unsigned int COMMAND_REQUEST_INTERVAL_US = 200000;
// --- SPSC Lock-Free Queue Definition ---
volatile uint8_t struct_queue[STRUCT_QUEUE_LEN][STRUCT_SIZE];
volatile uint8_t queueHead = 0;
volatile uint8_t queueTail = 0;
volatile bool shouldRequestData = false;

volatile uint8_t log_queue[LOG_QUEUE_SIZE];
volatile uint32_t log_head = 0;
volatile uint32_t log_tail = 0;
volatile bool data_ready_to_send = false;
volatile bool log_queue_overflow = false;
const unsigned int LOGGING_DATA_INTERVAL_US = 100000;
// ========================================================================
// NEW TIMING VARIABLES
// ========================================================================
volatile uint32_t cycle_count_start, cycle_count_1, cycle_count_2, cycle_count_3, cycle_count_4, cycle_count_end;
volatile uint32_t duration_0_to_1, duration_1_to_2, duration_2_to_3, duration_3_to_4, duration_4_to_5, duration_total;

// --- Communication Protocol Structures ---
struct HeaderPacket {
    uint8_t packet_type;
    uint8_t num_blocks;
    uint16_t last_block_size;
};

// --- Networking Configuration ---
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 0, 177);
IPAddress server(192, 168, 0, 142);
int port = 65432;

// --- Communication Protocol Keys ---
// For requesting/receiving command data from Pi
const uint8_t REQUEST_COMMANDS_BYTE = 0xAA;
const uint8_t COMMAND_HEADER_PACKET = 0xBB;
const uint8_t CONFIRM_COMMANDS_RECEIPT_BYTE = 0xCC;
const uint8_t NO_COMMANDS_AVAILABLE_SIGNAL = 0xFF;
// For sending log data to Pi
const uint8_t LOG_DATA_CHUNK_HEADER = 0xDD; // "Start key" for a chunk of log data
const uint8_t LOG_DATA_FLUSH_HEADER = 0xEE; // "Start key" for the final flushed data
const uint8_t LINE_NUMBER_UPDATE_HEADER = 0x60; // "Start key" for a line number status update

// --- NEW: Configuration Mode Protocol Keys ---
const uint8_t CONFIG_PACKET_HEADER = 0xA0;        // Pi -> Teensy: Identifies the start of a configuration data packet.
const uint8_t CONFIG_CONFIRMATION_BYTE = 0xA1;    // Teensy -> Pi: Acknowledges successful receipt of the configuration.
const uint8_t START_EXECUTION_BYTE = 0xB0;        // Pi -> Teensy: Tells the Teensy to exit config mode and start the machine.




// --- Global State ---
volatile bool emergencyStop = false;
uint8_t* current_block = nullptr;

volatile uint32_t current_line_number = 0;
volatile uint32_t current_f_clock = 0;
volatile uint16_t i = 0;
volatile bool tool_run = false;
volatile bool program_reading = true; // Start in reading mode
volatile bool misc_bit = false;

// DDA state
volatile uint32_t currentQ = 0;

volatile int32_t x_targetPosition = 0;
volatile int32_t y_targetPosition = 0;
volatile int32_t z_targetPosition = 0;

volatile uint8_t direction_x = 0;
volatile uint8_t direction_y = 0;
volatile uint8_t direction_z = 0;

volatile int32_t x_currentPosition = 0;
volatile int32_t y_currentPosition = 0;
volatile int32_t z_currentPosition = 0;

volatile int32_t x_positionError= 0;
volatile int32_t y_positionError= 0;
volatile int32_t z_positionError= 0;

volatile float x_integralErrorSum = 0.0;
volatile float y_integralErrorSum = 0.0;
volatile float z_integralErrorSum = 0.0;

// Create an EthernetClient object to make connections
EthernetClient client;


IntervalTimer requestTimer;
//IntervalTimer statusTimer;
IntervalTimer controlTimer;
IntervalTimer dataCheckTimer;

Encoder x_motorEncoder(x_ENCODER_PHASE_A_PIN, x_ENCODER_PHASE_B_PIN);
Encoder z_motorEncoder(z_ENCODER_PHASE_A_PIN, z_ENCODER_PHASE_B_PIN);
Encoder y_motorEncoder(y_ENCODER_PHASE_A_PIN, y_ENCODER_PHASE_B_PIN);

// --- Status Printing ---
unsigned long lastPrintTime = 0;
const int PRINT_INTERVAL = 500;

// --- Forward Declarations ---
void handleSerialCommands();
void sendBatchData1();
void ethReceiver();
void sendDataChunkToPi();
void flushLogDataToServer();

// Queue Management & Helpers
bool isQueueEmpty();
bool isQueueFull();
uint8_t* current_struct();
bool dequeue_struct();
uint8_t getQueueSize();
void increment_i();
uint16_t swap16(uint16_t val);
int ddaStep();
void parse_and_start_run();
void buffer_reader();

// Interrupt Service Routines (ISRs)
void triggerDataRequest();
void statusPrinterISR();
void stopButtonISR();
void controlLoopISR();
void checkDataAndSetFlagISR();

// Networking
void getDataFromServer();
bool ensureConnection();

// Motor Control Functions
void moveXMotorForward(int speed);
void moveXMotorReverse(int speed);
void stopXMotor();
void moveYMotorForward(int speed);
void moveYMotorReverse(int speed);
void stopYMotor();
void moveZMotorForward(int speed);
void moveZMotorReverse(int speed);
void stopZMotor();

// =================================================================================
// SETUP FUNCTION
// =================================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) { }
    Serial.println("System Booting... Initializing Ethernet.");

    // --- BASIC ETHERNET INITIALIZATION ---
    Ethernet.begin(mac, ip);
    delay(1000);

    if (Ethernet.hardwareStatus() == EthernetNoHardware || Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet connection failed. Halting.");
        while (true) {}
    }
    Serial.print("Teensy IP address: ");
    Serial.println(Ethernet.localIP());

    Serial.print("Connecting to server at ");
    Serial.print(server);
    Serial.println("...");

    while (!client.connect(server, port)) {
        Serial.println("Connection failed. Retrying in 2 seconds...");
        delay(2000);
    }
    Serial.println("Connection successful! Entering Configuration Mode.");
    Serial.println("Waiting for configuration packet (Header 0xA0) from server...");

    // ==============================================================
    // --- PHASE 1: CONFIGURATION LOOP ---
    // The Teensy will wait here until it has been configured AND receives a start command.
    // ==============================================================
    bool isConfigured = false;
    bool startCommandReceived = false;

    while (!startCommandReceived) {
        if (client.available() > 0) {
            // Peek at the first byte to see what kind of command it is
            uint8_t header = client.peek();

            // --- BRANCH 1: Handle Configuration Packet ---
            if (header == CONFIG_PACKET_HEADER) {
                // Check if the *entire* packet has arrived yet.
                if (client.available() >= (1 + sizeof(ConfigurationPacket))) {
                    client.read(); // Consume the header byte we peeked at.

                    ConfigurationPacket newConfig;
                    // Read the packet bytes directly into our struct variable
                    client.readBytes((uint8_t*)&newConfig, sizeof(newConfig));


                    // --- APPLY THE NEW CONFIGURATION ---
                    Serial.println("Configuration packet received! Applying settings...");
                    // Apply values from the struct to our global volatile variables
                    CONTROL_LOOP_INTERVAL_US = newConfig.control_loop_interval_us;
                    ERROR_DEADBAND = newConfig.error_deadband;
                    INTEGRAL_ACTIVE_ZONE = newConfig.integral_active_zone;
                    INTEGRAL_MAX_SUM = newConfig.integral_max_sum;

                    KP_xf = newConfig.kp_xf;
                    KI_xf = newConfig.ki_xf;
                    FRICTION_KICK_xf = newConfig.friction_kick_xf;
                    KP_xr = newConfig.kp_xr;
                    KI_xr = newConfig.ki_xr;
                    FRICTION_KICK_xr = newConfig.friction_kick_xr;

                    KP_yf = newConfig.kp_yf;
                    KI_yf = newConfig.ki_yf;
                    FRICTION_KICK_yf = newConfig.friction_kick_yf;
                    KP_yr = newConfig.kp_yr;
                    KI_yr = newConfig.ki_yr;
                    FRICTION_KICK_yr = newConfig.friction_kick_yr;
                    
                    KP_zf = newConfig.kp_zf;
                    KI_zf = newConfig.ki_zf;
                    FRICTION_KICK_zf = newConfig.friction_kick_zf;
                    KP_zr = newConfig.kp_zr;
                    KI_zr = newConfig.ki_zr;
                    FRICTION_KICK_zr = newConfig.friction_kick_zr;
                    
                    // --- NEW: Apply Logging Settings ---
                    // Note: enable_data_logging is a uint8_t (0 or 1) in the packet for platform compatibility.
                    DataLoggingMode = (newConfig.enable_data_logging == 1);
                    logging_frequency_multiplier = newConfig.logging_frequency_multiplier;
                    BATCH_DATA_INTERVAL_US = newConfig.batch_data_interval_us;

                    // Safety check for multiplier to prevent division by zero or hangs.
                    if (logging_frequency_multiplier == 0) {
                        logging_frequency_multiplier = 1;
                    }
                    
                    // --- RECALCULATE DEPENDENT VARIABLES ---
                    // It's critical to update these values now that the interval has changed.
                    CONTROL_LOOP_INTERVAL_S = CONTROL_LOOP_INTERVAL_US / 1000000.0f;
                    frequencyThreshold = 1000000 / CONTROL_LOOP_INTERVAL_US;

                    Serial.print("New control loop interval (us): ");
                    Serial.println(CONTROL_LOOP_INTERVAL_US);
                    Serial.println("Settings applied. Sending confirmation to server...");
                    
                    // Send a confirmation byte back to the Pi
                    client.write(CONFIG_CONFIRMATION_BYTE);
                    isConfigured = true;
                    Serial.println("Waiting for start command (Header 0xB0)...");
                }
            }
            // --- BRANCH 2: Handle Start Command ---
            else if (header == START_EXECUTION_BYTE) {
                client.read(); // Consume the start byte
                if (isConfigured) {
                    Serial.println("Start command received. Exiting configuration mode.");
                    startCommandReceived = true; // This will break the while loop
                } else {
                    Serial.println("Start command received, but system is not configured yet! Ignoring.");
                }
            }
            // --- BRANCH 3: Handle unexpected data ---
            else {
                Serial.print("Unexpected byte received in config mode: 0x");
                Serial.println(client.read(), HEX); // Read and discard
            }
        }
        delay(10); // Small delay to prevent the loop from running too fast
    }

    // ==============================================================
    // --- PHASE 2: NORMAL SETUP CONTINUES ---
    // Now we use the configured values to initialize the rest of the system.
    // ==============================================================
    Serial.println("Initializing hardware with new configuration...");
    analogWriteResolution(PWM_RESOLUTION);
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    // Initialize all your hardware pins
    pinMode(x_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(x_MOTOR_REVERSE_PIN, OUTPUT);
    pinMode(y_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(y_MOTOR_REVERSE_PIN, OUTPUT);
    pinMode(z_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(z_MOTOR_REVERSE_PIN, OUTPUT);
    pinMode(x_LIMIT_SWITCH_LEFT_PIN, INPUT_PULLUP);
    pinMode(x_LIMIT_SWITCH_RIGHT_PIN, INPUT_PULLUP);
    pinMode(z_LIMIT_SWITCH_LEFT_PIN, INPUT_PULLUP);
    pinMode(z_LIMIT_SWITCH_RIGHT_PIN, INPUT_PULLUP);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);


    stopXMotor();
    stopYMotor();
    stopZMotor();

    delay(100);

    x_motorEncoder.write(0);
    y_motorEncoder.write(0);
    z_motorEncoder.write(0);

    x_home();

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), stopButtonISR, FALLING);

    // --- CRITICAL STEP: Initialize timers with the NEW value ---
    Serial.print("Starting control loop with configured interval: ");
    Serial.print(CONTROL_LOOP_INTERVAL_US);
    Serial.println(" us");
    controlTimer.begin(controlLoopISR, CONTROL_LOOP_INTERVAL_US);
    
    // Initialize other timers
    requestTimer.begin(triggerDataRequest, COMMAND_REQUEST_INTERVAL_US); // 200ms
    dataCheckTimer.begin(checkDataAndSetFlagISR, LOGGING_DATA_INTERVAL_US); // Use configured value
    
    Serial.println("Setup complete. System is active.");
}

void loop() {
  ethReceiver();
  handleSerialCommands();
  // if (send_batchdata_1) { 
  //     sendBatchData1();   
  // }  
  if (micros() - lastStatusPrintMicros >= STATUS_PRINT_INTERVAL_US) {
        lastStatusPrintMicros = micros(); // Reset the timer
        statusPrinterISR();                    // Call the print function
    }
  if (shouldRequestData) {
    shouldRequestData = false;
    if (STRUCT_QUEUE_LEN - 1 - getQueueSize() >= QUEUE_REQUEST_THRESHOLD) {
      unsigned long StartTime = micros();
      getDataFromServer();
      unsigned long Duration = micros() - StartTime;
    }
  }
  if (data_ready_to_send) {
    sendDataChunkToPi();
  }
}

void controlLoopISR() {
  cycle_count_start = ARM_DWT_CYCCNT;
  if (emergencyStop) {
    stopXMotor();
    stopYMotor();
    stopZMotor();
    return;
  }

  if (program_reading) {
    buffer_reader();
  }
  cycle_count_1 = ARM_DWT_CYCCNT; // Time 1
  x_currentPosition = x_motorEncoder.read();
  y_currentPosition = y_motorEncoder.read();
  z_currentPosition = z_motorEncoder.read();

  cycle_count_2 = ARM_DWT_CYCCNT; // Time 2

  // --- Lower-frequency velocity calculation setup ---
  static int velocity_update_counter = 0;
  const int VELOCITY_UPDATE_INTERVAL = 100; // Update velocity every 100 cycles

  // Position tracking for each axis
  static long x_position_at_last_velocity_calc = 0;
  static long y_position_at_last_velocity_calc = 0;
  static long z_position_at_last_velocity_calc = 0;

  // Last calculated velocity for each axis
  static long x_velocity = 0;
  static long y_velocity = 0;
  static long z_velocity = 0;

  // --- Error Calculation ---
  x_positionError = x_targetPosition - x_currentPosition;
  y_positionError = y_targetPosition - y_currentPosition;
  z_positionError = z_targetPosition - z_currentPosition;

  // --- Update velocity for all axes periodically ---
  if (++velocity_update_counter >= VELOCITY_UPDATE_INTERVAL) {
      // Calculate velocity as the change in position since the last update
      x_velocity = x_currentPosition - x_position_at_last_velocity_calc;
      y_velocity = y_currentPosition - y_position_at_last_velocity_calc;
      z_velocity = z_currentPosition - z_position_at_last_velocity_calc;

      // Store the current position for the next calculation
      x_position_at_last_velocity_calc = x_currentPosition;
      y_position_at_last_velocity_calc = y_currentPosition;
      z_position_at_last_velocity_calc = z_currentPosition;

      // Reset the counter
      velocity_update_counter = 0;
  }


  if (abs(x_positionError) < INTEGRAL_ACTIVE_ZONE) {
    // Integral term with anti-windup zone
    if (abs(x_positionError) > ERROR_DEADBAND) {
      x_integralErrorSum += x_positionError * CONTROL_LOOP_INTERVAL_S;
      // Integral clamping to prevent excessive buildup
      x_integralErrorSum = constrain(x_integralErrorSum, -INTEGRAL_MAX_SUM, INTEGRAL_MAX_SUM);
    } else {
      x_integralErrorSum = 0; // Reset integral
    }
  }
  if (abs(y_positionError) < INTEGRAL_ACTIVE_ZONE) {
    // Integral term with anti-windup zone
    if (abs(y_positionError) > ERROR_DEADBAND) {
      y_integralErrorSum += y_positionError * CONTROL_LOOP_INTERVAL_S;
      // Integral clamping to prevent excessive buildup
      y_integralErrorSum = constrain(y_integralErrorSum, -INTEGRAL_MAX_SUM, INTEGRAL_MAX_SUM);
    } else {
      y_integralErrorSum = 0; // Reset integral
    }
  }
  if (abs(z_positionError) < INTEGRAL_ACTIVE_ZONE) {
    // Integral term with anti-windup zone
    if (abs(z_positionError) > ERROR_DEADBAND) {
      z_integralErrorSum += z_positionError * CONTROL_LOOP_INTERVAL_S;
      // Integral clamping to prevent excessive buildup
      z_integralErrorSum = constrain(z_integralErrorSum, -INTEGRAL_MAX_SUM, INTEGRAL_MAX_SUM);
    } else {
      z_integralErrorSum = 0; // Reset integral
    }
  }
  cycle_count_3 = ARM_DWT_CYCCNT; // Time 3
  // --- X-AXIS CONTROL ---
  int friction_kick_to_apply_x = 0;
  if (x_velocity > 0) {
      // If moving forward, apply forward kick
      friction_kick_to_apply_x = FRICTION_KICK_xf;
  } else if (x_velocity < 0) {
      // If moving in reverse, apply reverse kick
      friction_kick_to_apply_x = -FRICTION_KICK_xr;
  } else { // x_velocity is zero, motor is stopped
      // Apply kick in the direction the controller WANTS to go
      if (x_positionError > ERROR_DEADBAND) {
          // If error is positive, we need to move forward
          friction_kick_to_apply_x = FRICTION_KICK_xf;
      } else if (x_positionError < -ERROR_DEADBAND) {
          // If error is negative, we need to move in reverse
          friction_kick_to_apply_x = -FRICTION_KICK_xr;
      }
  }

  float x_output;
  if (x_positionError >= 0) {
      // Error is positive, use forward gains
      x_output = (KP_xf * x_positionError) + (KI_xf * x_integralErrorSum) + friction_kick_to_apply_x;
  } else {
      // Error is negative, use reverse gains
      x_output = (KP_xr * x_positionError) + (KI_xr * x_integralErrorSum) + friction_kick_to_apply_x;
  }

  int x_pwmSpeed = constrain(abs(x_output), 0, MAX_PWM_VALUE);

  if (x_output >= 0) {
      moveXMotorForward(x_pwmSpeed);
  } else {
      moveXMotorReverse(x_pwmSpeed);
  }
  // // --- X-AXIS CONTROL ---
  // if (x_positionError >= 0) {
  //     // Error is positive, move forward
  //     float x_output = (KP_xf * x_positionError) + (KI_xf * x_integralErrorSum);
  //     int x_pwmSpeed = constrain(abs(x_output) + FRICTION_KICK_xf, 0, MAX_PWM_VALUE);
  //     moveXMotorForward(x_pwmSpeed);
  // }
  // else {
  //     // Error is negative, move in reverse
  //     float x_output = (KP_xr * x_positionError) + (KI_xr * x_integralErrorSum);
  //     int x_pwmSpeed = constrain(abs(x_output) + FRICTION_KICK_xr, 0, MAX_PWM_VALUE);
  //     moveXMotorReverse(x_pwmSpeed);
  // }

  // --- Y-AXIS CONTROL ---
  int friction_kick_to_apply_y = 0;
  if (y_velocity > 0) {
      // If moving forward, apply forward kick
      friction_kick_to_apply_y = FRICTION_KICK_yf;
  } else if (y_velocity < 0) {
      // If moving in reverse, apply reverse kick
      friction_kick_to_apply_y = -FRICTION_KICK_yr;
  } else { // y_velocity is zero, motor is stopped
      // Apply kick in the direction the controller WANTS to go
      if (y_positionError > ERROR_DEADBAND) {
          // If error is positive, we need to move forward
          friction_kick_to_apply_y = FRICTION_KICK_yf;
      } else if (y_positionError < -ERROR_DEADBAND) {
          // If error is negative, we need to move in reverse
          friction_kick_to_apply_y = -FRICTION_KICK_yr;
      }
  }

  float y_output;
  if (y_positionError >= 0) {
      // Error is positive, use forward gains
      y_output = (KP_yf * y_positionError) + (KI_yf * y_integralErrorSum) + friction_kick_to_apply_y;
  } else {
      // Error is negative, use reverse gains
      y_output = (KP_yr * y_positionError) + (KI_yr * y_integralErrorSum) + friction_kick_to_apply_y;
  }

  int y_pwmSpeed = constrain(abs(y_output), 0, MAX_PWM_VALUE);

  if (y_output >= 0) {
      moveYMotorForward(y_pwmSpeed);
  } else {
      moveYMotorReverse(y_pwmSpeed);
  }

  // --- Z-AXIS CONTROL (Corrected) ---
  int friction_kick_to_apply_z = 0;
    if (z_velocity > 0) {
        // If moving forward, apply forward kick
        friction_kick_to_apply_z = FRICTION_KICK_zf;
    } else if (z_velocity < 0) {
        // If moving in reverse, apply reverse kick
        friction_kick_to_apply_z = -FRICTION_KICK_zr;
    } else { // x_velocity is zero, motor is stopped
        // Apply kick in the direction the controller WANTS to go
        if (z_positionError > ERROR_DEADBAND) {
            // If error is positive, we need to move forward
            friction_kick_to_apply_z = FRICTION_KICK_zf;
        } else if (z_positionError < -ERROR_DEADBAND) {
            // If error is negative, we need to move in reverse
            friction_kick_to_apply_z = -FRICTION_KICK_zr;
        }
    }
  float z_output;
  if (z_positionError >= 0) {
      // Error is positive, use forward gains
      z_output = (KP_zf * z_positionError) + (KI_zf * z_integralErrorSum) + friction_kick_to_apply_z;
  } else {
      z_output = (KP_zr * z_positionError) + (KI_zr * z_integralErrorSum) + friction_kick_to_apply_z;
  }
  int z_pwmSpeed = constrain(abs(z_output), 0, MAX_PWM_VALUE);

  if (z_output >= 0){
    moveZMotorForward(z_pwmSpeed);
  }
  else{
    moveZMotorReverse(z_pwmSpeed);
  }
  // if (z_positionError >= 0) {
  //     // Error is positive, move "forward" (e.g., up)
  //     float z_output = (KP_zf * z_positionError) + (KI_zf * z_integralErrorSum);
  //     int z_pwmSpeed = constrain(abs(z_output) + FRICTION_KICK_zf, 0, MAX_PWM_VALUE);
  //     moveZMotorForward(z_pwmSpeed);
  // }
  // else {
  //     // Error is negative, move "reverse" (e.g., down)
  //     float z_output = (KP_zr * z_positionError) + (KI_zr * z_integralErrorSum);
  //     int z_pwmSpeed = constrain(abs(z_output) + FRICTION_KICK_zr, 0, MAX_PWM_VALUE);
  //     moveZMotorReverse(z_pwmSpeed);
  // }
  cycle_count_4 = ARM_DWT_CYCCNT; // Time 4
  // --- Data Logging ---
  if (DataLoggingMode) {
    if (++logging_counter >= logging_frequency_multiplier) {
      // Each log entry will be 6 variables * 4 bytes/variable = 24 bytes.
      uint32_t next_head = (log_head + 16) % LOG_QUEUE_SIZE;
      if (next_head == log_tail) {
          log_queue_overflow = true;
          DataLoggingMode = false; // Stop logging to prevent data loss
          Serial.println("FULLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL");
          return;
      }
      // Correctly copy 4 bytes for each variable at the correct offset.
      memcpy((void*)&log_queue[log_head + 0],  (const void*)&x_targetPosition, 4);
      memcpy((void*)&log_queue[log_head + 4],  (const void*)&x_currentPosition, 4);
      //memcpy((void*)&log_queue[log_head + 8],  (const void*)&y_targetPosition, 4);
      //memcpy((void*)&log_queue[log_head + 12], (const void*)&y_currentPosition, 4);
      memcpy((void*)&log_queue[log_head + 8], (const void*)&z_targetPosition, 4);
      memcpy((void*)&log_queue[log_head + 12], (const void*)&z_currentPosition, 4);
      // --- NEW: Add the two constant values ---
      //memcpy((void*)&log_queue[log_head + 24], (const void*)&LOG_CONSTANT_1, 4);
      //memcpy((void*)&log_queue[log_head + 28], (const void*)&LOG_CONSTANT_2, 4);

      log_head = next_head;
  }
  }
  cycle_count_end = ARM_DWT_CYCCNT; // Time 5 (End time)
  // --- Calculate Durations ---
  duration_0_to_1 = cycle_count_1 - cycle_count_start;
  duration_1_to_2 = cycle_count_2 - cycle_count_1;
  duration_2_to_3 = cycle_count_3 - cycle_count_2;
  duration_3_to_4 = cycle_count_4 - cycle_count_3;
  duration_4_to_5 = cycle_count_end - cycle_count_4;
  duration_total  = cycle_count_end - cycle_count_start;
}

void checkDataAndSetFlagISR() {
    static uint8_t isr_call_counter = 0;
    if (++isr_call_counter >= BATCH_DATA_INTERVAL_US/LOGGING_DATA_INTERVAL_US) {
        send_batchdata_1 = true; // Set the flag for the main loop
        isr_call_counter = 0;             // Reset the counter
    }
    if (data_ready_to_send) return;
    uint32_t head = log_head;
    uint32_t tail = log_tail;
    uint32_t queue_size = (head >= tail) ? (head - tail) : (LOG_QUEUE_SIZE - tail + head);
    if (queue_size >= SEND_CHUNK_SIZE) {
        data_ready_to_send = true;
    }
}

/**
 * Sends a standard-sized chunk of log data to the server.
 *
 * This function is called when the log buffer has filled up to the SEND_CHUNK_SIZE threshold.
 * It wraps the data chunk in a packet with a specific header (LOG_DATA_CHUNK_HEADER) and
 * the size of the payload, then sends it over the persistent TCP connection.
 */
void sendDataChunkToPi() {
    // First, ensure the connection to the server is active.
    if (!ensureConnection()) {
        Serial.println("[ERROR] Cannot send data chunk, no connection!");
        return;
    }

    // 1. Send the packet header to identify this as a standard log data chunk.
    client.write(LOG_DATA_CHUNK_HEADER);

    // 2. Send the size of the payload so the server knows exactly how many bytes to read.
    uint32_t bytesToSend = SEND_CHUNK_SIZE;
    client.write((uint8_t*)&bytesToSend, sizeof(bytesToSend));

    // 3. Send the actual data payload in efficient mini-chunks.
    //    (Your original, efficient memory copy logic is used here).
    const int MINI_CHUNK_SIZE = 1024; // Send data in 1KB pieces
    uint8_t temp_buffer[MINI_CHUNK_SIZE];

    for (int i = 0; i < SEND_CHUNK_SIZE; i += MINI_CHUNK_SIZE) {
        // This logic correctly handles the wrap-around case in the circular buffer.
        if (log_tail + i + MINI_CHUNK_SIZE > LOG_QUEUE_SIZE) {
            uint32_t bytes_to_end = LOG_QUEUE_SIZE - (log_tail + i);
            memcpy(temp_buffer, (const void*)&log_queue[log_tail + i], bytes_to_end);
            uint32_t bytes_from_start = MINI_CHUNK_SIZE - bytes_to_end;
            memcpy(temp_buffer + bytes_to_end, (const void*)&log_queue[0], bytes_from_start);
        } else {
            memcpy(temp_buffer, (const void*)&log_queue[log_tail + i], MINI_CHUNK_SIZE);
        }
        client.write(temp_buffer, MINI_CHUNK_SIZE);
    }

    // 4. Advance the tail of the circular buffer and reset the flag.
    log_tail = (log_tail + SEND_CHUNK_SIZE) % LOG_QUEUE_SIZE;
    data_ready_to_send = false;
}

/**
 * Sends any remaining log data from the buffer to the server.
 *
 * This function is called at the end of a process to ensure no data is lost.
 * It calculates the exact number of bytes remaining in the log buffer, wraps them
 * in a packet with a specific "flush" header, and sends them.
 * It does NOT close the client connection.
 */
void flushLogDataToServer() {
    // Stop the controller and logging before flushing.
    DataLoggingMode = false;
    Serial.println("Flushing remaining log data...");

    if (!ensureConnection()) {
        Serial.println("[ERROR] Cannot flush data, no connection!");
        return;
    }

    // Calculate the exact number of bytes left in the circular buffer.
    uint32_t head = log_head;
    uint32_t tail = log_tail;
    uint32_t remaining_bytes = (head >= tail) ? (head - tail) : (LOG_QUEUE_SIZE - tail + head);

    if (remaining_bytes > 0) {
        // 1. Send the packet header to identify this as the FINAL flushed data.
        client.write(LOG_DATA_FLUSH_HEADER);

        // 2. Send the size of the payload.
        client.write((uint8_t*)&remaining_bytes, sizeof(remaining_bytes));

        // 3. Send the remaining data payload.
        //    (Your original logic for sending variable-sized data is used here).
        const int MINI_CHUNK_SIZE = 1024;
        uint8_t temp_buffer[MINI_CHUNK_SIZE];
        uint32_t bytes_sent = 0;

        while (bytes_sent < remaining_bytes) {
            uint32_t bytes_to_send = remaining_bytes - bytes_sent;
            if (bytes_to_send > MINI_CHUNK_SIZE) {
                bytes_to_send = MINI_CHUNK_SIZE;
            }

            // This logic correctly handles the wrap-around case for the final chunk.
            uint32_t current_tail_pos = (tail + bytes_sent) % LOG_QUEUE_SIZE;
            if (current_tail_pos + bytes_to_send > LOG_QUEUE_SIZE) {
                uint32_t bytes_to_end = LOG_QUEUE_SIZE - current_tail_pos;
                memcpy(temp_buffer, (const void*)&log_queue[current_tail_pos], bytes_to_end);
                uint32_t bytes_from_start = bytes_to_send - bytes_to_end;
                memcpy(temp_buffer + bytes_to_end, (const void*)&log_queue[0], bytes_from_start);
            } else {
                memcpy(temp_buffer, (const void*)&log_queue[current_tail_pos], bytes_to_send);
            }

            client.write(temp_buffer, bytes_to_send);
            bytes_sent += bytes_to_send;
        }

        client.flush(); // Ensure all data is sent from the Teensy's buffer.
        Serial.print(remaining_bytes);
        Serial.println(" bytes flushed to Pi.");

    } else {
        Serial.println("No remaining data to flush.");
    }

    // Reset the buffer state for the next run.
    log_head = 0;
    log_tail = 0;
    data_ready_to_send = false;
    Serial.println("Data flush complete. Ready for next operation.");
}

// =================================================================================
// QUEUE MANAGEMENT & HELPER FUNCTIONS
// =================================================================================

bool isQueueEmpty() { return queueHead == queueTail; }
bool isQueueFull() { return ((queueHead + 1) & (STRUCT_QUEUE_LEN - 1)) == queueTail; }
uint8_t* current_struct() { return isQueueEmpty() ? nullptr : (uint8_t*)struct_queue[queueTail]; }
bool dequeue_struct() { if(isQueueEmpty()) return false; queueTail = (queueTail + 1) & (STRUCT_QUEUE_LEN - 1); return true; }
uint8_t getQueueSize() { uint8_t h = queueHead; uint8_t t = queueTail; return h >= t ? h - t : STRUCT_QUEUE_LEN - (t - h); }
void increment_i() { if (++i >= STRUCT_SIZE) { i = 0; dequeue_struct(); current_block = current_struct(); } }

uint16_t swap16(uint16_t val) {
  return (val << 8) | (val >> 8);
}

int ddaStep() {
  currentQ += current_f_clock;
  if (currentQ >= frequencyThreshold) {
    currentQ -= frequencyThreshold;
    return 1;
  }
  return 0;
}

void parse_and_start_run() {
  // Step 1: Increment and read first byte for current_f_clock
  increment_i();
  uint8_t byte1 = current_block[i];

  // Step 2: Increment and read second byte
  increment_i();
  uint8_t byte2 = current_block[i];

  // Step 3: Increment and read third byte
  increment_i();
  uint8_t byte3 = current_block[i];

  // Step 4: Compute current_f_clock using the three bytes
  current_f_clock = ((uint32_t)byte1 << 16) | ((uint32_t)byte2 << 8) | byte3;

  // Step 5: Re-initialize the DDA
  currentQ = 0;
  // Step 6: Increment and read direction byte
  increment_i();
  uint8_t direction_byte = current_block[i];
  direction_x = (direction_byte >> 2) & 1;
  direction_y = (direction_byte >> 1) & 1;
  direction_z = (direction_byte >> 0) & 1;

  // Step 7: Set tool_run true
  tool_run = true;

  // Step 8: Final increment after tool_run
  increment_i();
}

void buffer_reader() {
    if (isQueueEmpty()){
      return;
    }
    if (!misc_bit) {
        if (current_block == nullptr) {
            current_block = current_struct();
            if (current_block) i = 0;
        }

        if (current_block != nullptr) {
            if (!tool_run) {
                uint8_t marker = current_block[i];
                if (marker == 0x0D) {
                    program_reading = false;
                }
                while (marker == 0x0C) {
                    current_line_number++;
                    increment_i();
                    marker = current_block[i];
                }
                if (marker == 0x09) { 
                    current_line_number++;
                    increment_i(); 
                    marker = current_block[i]; }

                if (marker == 0x0A) { 
                  parse_and_start_run(); 
                  } 
                else if (marker == 0x0B) { 
                  //misc_bit = true; 
                  increment_i();
                  increment_i();
                  increment_i();
                  increment_i(); 
                  }
            }

            if (tool_run) {
              if (ddaStep()){
                uint8_t cmd = current_block[i];
                if ((cmd >> 2) & 1) { x_targetPosition += (direction_x == 0) ? 1 : -1; }
                if ((cmd >> 1) & 1) { y_targetPosition += (direction_y == 0) ? 1 : -1; }
                if ((cmd >> 0) & 1) { z_targetPosition += (direction_z == 0) ? 1 : -1; }
                increment_i();
                if (!current_block) { tool_run = false; } 
                else if (current_block[i] >= 0x09 && current_block[i] <= 0x0C) { tool_run = false; }
                else if (current_block[i] == 0x0D) {
                    program_reading = false;
                }
              }
            }
        }
    }
}

// This function will be called by the timer every 5 seconds
void triggerDataRequest() {
  // Just set the flag. Keep ISRs (Interrupt Service Routines) short!
  shouldRequestData = true;
}

void statusPrinterISR() {
    // --- Calculate Log Queue Usage Directly ---
    // Create local copies of volatile variables for safe calculation
    uint32_t head = log_head;
    uint32_t tail = log_tail;
    uint32_t log_used_bytes;

    if (head >= tail) {
        log_used_bytes = head - tail;
    } else {
        log_used_bytes = LOG_QUEUE_SIZE - (tail - head);
    }
    float log_percent_used = ((float)log_used_bytes / LOG_QUEUE_SIZE) * 100.0f;

    // --- Print all status information ---
    Serial.print("[STATUS] CmdQ: ");
    Serial.print(getQueueSize());
    
    Serial.print(" | Log Usage: ");
    Serial.print(log_percent_used, 1); // Print with one decimal place
    Serial.print("%");

    Serial.print(" | Target(mm): X=");
    Serial.print(x_targetPosition / 1000.0f, 3);
    Serial.print(" Y=");
    Serial.print(y_targetPosition / 1000.0f, 3);
    Serial.print(" Z=");
    Serial.println(z_targetPosition / 1000.0f, 3);
    // --- NEW: Descriptive Timing Information Printout ---
    // Convert cycles to microseconds by dividing by CPU speed in MHz (600)
    //Serial.print(" | ISR (us): Total=");
    //Serial.print(duration_total / 600.0f, 2); // Print with 2 decimal places
    // Serial.print(" (Buf:");
    // Serial.print(duration_0_to_1 / 600.0f, 2);
    // Serial.print(", Enc:");
    // Serial.print(duration_1_to_2 / 600.0f, 2);
    // Serial.print(", Int:");
    // Serial.print(duration_2_to_3 / 600.0f, 2);
    // Serial.print(", Svo:");
    // Serial.print(duration_3_to_4 / 600.0f, 2);
    // Serial.print(", Log:");
    // Serial.print(duration_4_to_5 / 600.0f, 2);
    // Serial.println(")");
}

void stopButtonISR() {
    emergencyStop = true;
}

void getDataFromServer() {
  // Step 1: Make sure we have a connection.
  if (!ensureConnection()) {
      return; // Can't do anything if we're not connected.
  }

  // Step 2: Send the byte to request data (same as before)
  client.write(REQUEST_COMMANDS_BYTE);
  
  // 2. Wait for and read the HeaderPacket
  HeaderPacket header;
  unsigned long headerStartTime = micros();
  while(client.available() < sizeof(HeaderPacket)) {
    if (micros() - headerStartTime > 5000000) { // 5 second timeout for header
      Serial.println("[ERROR] Timed out waiting for header packet.");
      return;
    }
  }

  // Read the raw bytes of the header
  client.read((uint8_t*)&header, sizeof(HeaderPacket));
  header.last_block_size = swap16(header.last_block_size);

  // 3. Check what the server sent
  if (header.packet_type == NO_COMMANDS_AVAILABLE_SIGNAL) {
      //Serial.println("[INFO] Server has no new data available.");
      return;
  }
  
  if (header.packet_type != COMMAND_HEADER_PACKET) {
      Serial.print("[ERROR] Invalid header packet type received: 0x");
      Serial.println(header.packet_type, HEX);
      return;
  }

  //Serial.print("[STATUS] Header received. Expecting ");
  //Serial.print(header.num_blocks);
  //Serial.println(" data blocks.");

  // 4. Loop to receive each data block
  for (uint8_t blockNum = 0; blockNum < header.num_blocks; blockNum++) {
    // Check if our local queue is full before accepting a new block
    if (isQueueFull()) {
      Serial.println("[ERROR] Queue is full! Cannot receive more data. Aborting.");
      // We need to tell the server to stop sending, but for now we just close connection.
      return;
    }

    uint16_t bytesToReceive = (blockNum == header.num_blocks - 1) ? header.last_block_size : STRUCT_SIZE;
    uint8_t* targetBuffer = (uint8_t*)struct_queue[queueHead];
    
    //Serial.print("  -> Receiving block #");
    //Serial.print(blockNum + 1);
    //Serial.print(" (");
    //Serial.print(bytesToReceive);
    //Serial.print(" bytes) into queue slot ");
    //Serial.println(queueHead);

    // Receive one full block of data
    unsigned int bytesReceived = 0;
    unsigned long blockStartTime = micros();
    while (bytesReceived < bytesToReceive) {
        if (client.available()) {
            int bytesRead = client.read(targetBuffer + bytesReceived, bytesToReceive - bytesReceived);
            if (bytesRead > 0) {
                bytesReceived += bytesRead;
            }
        }
        if (micros() - blockStartTime > 10000000) { // 10 sec timeout per block
            Serial.println("\n[ERROR] Timed out receiving data block.");
            return;
        }
    }
    
    // Successfully received a block, so advance the queue head
    queueHead = (queueHead + 1) & (STRUCT_QUEUE_LEN - 1);
  }

  //Serial.println("[STATUS] All blocks received successfully.");

  // 5. Send confirmation receipt to the server
  client.write(CONFIRM_COMMANDS_RECEIPT_BYTE);
  //Serial.println("[STATUS] Sent confirmation receipt (0xCC).");
}

bool ensureConnection() {
    // If we are already connected, we're good.
    if (client.connected()) {
        return true;
    }

    // If not, try to connect.
    Serial.println("[NETWORK] Connection lost. Attempting to reconnect...");
    if (client.connect(server, port)) {
        Serial.println("[NETWORK] Reconnected successfully.");
        return true;
    }

    // If the connection failed, report it.
    Serial.println("[ERROR] Reconnection failed!");
    return false;
}

void x_home() {
  Serial.println("Starting full homing sequence...");

  // Step 1: Move left until left limit switch is pressed
  while (digitalRead(x_LIMIT_SWITCH_LEFT_PIN) == HIGH) {
    moveXMotorReverse(180);
    delay(10);
  }
  stopXMotor();
  delay(100);

  // Step 2: Zero encoder
  x_motorEncoder.write(0);
  Serial.println("Left limit reached. Position zeroed.");
  moveXMotorForward(180);
  delay(500);
  stopXMotor();
  delay(100);
  // Step 3: Move right until right limit switch is pressed
  // while (digitalRead(x_LIMIT_SWITCH_RIGHT_PIN) == HIGH) {
  //   moveXMotorForward(180);
  //   delay(10);
  // }
  // stopXMotor();
  // delay(100);

  // // Step 4: Get travel distance
  // long travel = x_motorEncoder.read();
  // long center = travel / 2;

  // Serial.print("Right limit reached. Travel: ");
  // Serial.println(travel);

  // // Step 5: Move to center
  // while (x_motorEncoder.read() > center + ERROR_DEADBAND) {
  //   moveXMotorReverse(180);
  //   delay(10);
  // }
  // stopXMotor();
  // delay(100);

  // Step 6: Final zero
  x_motorEncoder.write(0);
  x_targetPosition = 0;
  Serial.println("X-axis homing complete. Centered and zeroed.");

  while (digitalRead(z_LIMIT_SWITCH_LEFT_PIN) == HIGH) {
    moveZMotorReverse(180);
    delay(10);
  }
  stopZMotor();
  delay(100);

  // Step 2: Zero encoder
  z_motorEncoder.write(0);
  Serial.println("Left limit reached. Position zeroed.");
  moveZMotorForward(180);
  delay(500);
  stopZMotor();
  delay(100);
  // Step 3: Move right until right limit switch is pressed
  // while (digitalRead(x_LIMIT_SWITCH_RIGHT_PIN) == HIGH) {
  //   moveXMotorForward(180);
  //   delay(10);
  // }
  // stopXMotor();
  // delay(100);

  // // Step 4: Get travel distance
  // long travel = x_motorEncoder.read();
  // long center = travel / 2;

  // Serial.print("Right limit reached. Travel: ");
  // Serial.println(travel);

  // // Step 5: Move to center
  // while (x_motorEncoder.read() > center + ERROR_DEADBAND) {
  //   moveXMotorReverse(180);
  //   delay(10);
  // }
  // stopXMotor();
  // delay(100);

  // Step 6: Final zero
  z_motorEncoder.write(0);
  z_targetPosition = 0;
  Serial.println("Z-axis homing complete. Centered and zeroed.");
}
// These functions accept a speed value to pass to analogWrite.
// X Axis
void moveXMotorForward(int speed) {
  analogWrite(x_MOTOR_REVERSE_PIN, 0);
  analogWrite(x_MOTOR_FORWARD_PIN, speed);
}

void moveXMotorReverse(int speed) {
  analogWrite(x_MOTOR_FORWARD_PIN, 0);
  analogWrite(x_MOTOR_REVERSE_PIN, speed);
}

void stopXMotor() {
  analogWrite(x_MOTOR_FORWARD_PIN, 0);
  analogWrite(x_MOTOR_REVERSE_PIN, 0);
}
// Y Axis
void moveYMotorForward(int speed) {
  analogWrite(y_MOTOR_REVERSE_PIN, 0);
  analogWrite(y_MOTOR_FORWARD_PIN, speed);
}

void moveYMotorReverse(int speed) {
  analogWrite(y_MOTOR_FORWARD_PIN, 0);
  analogWrite(y_MOTOR_REVERSE_PIN, speed);
}

void stopYMotor() {
  analogWrite(y_MOTOR_FORWARD_PIN, 0);
  analogWrite(y_MOTOR_REVERSE_PIN, 0);
}
// Z Axis
void moveZMotorForward(int speed) {
  analogWrite(z_MOTOR_REVERSE_PIN, 0);
  analogWrite(z_MOTOR_FORWARD_PIN, speed);
}

void moveZMotorReverse(int speed) {
  analogWrite(z_MOTOR_FORWARD_PIN, 0);
  analogWrite(z_MOTOR_REVERSE_PIN, speed);
}
void stopZMotor() {
  analogWrite(z_MOTOR_FORWARD_PIN, 0);
  analogWrite(z_MOTOR_REVERSE_PIN, 0);
}

void ethReceiver() {
    // We need at least 2 bytes for any valid command (Op-Code + State)
    if (!client.connected() || client.available() < 2) {
        return;
    }

    // Check if the first byte in the buffer is our logging command Op-Code
    if (client.peek() == 0x50) {
        client.read(); // Consume the 0x50 Op-Code
        uint8_t state = client.read(); // Consume the state byte

        // --- Logic Branch 1: Turn Logging ON ---
        if (state == 0x01) {
            // Check if the 2-byte multiplier has also arrived
            if (client.available() < 2) {
                // Incomplete packet. Return and wait for the rest of the data.
                // The Op-Code and state byte have been consumed, so we won't re-process them.
                return;
            }

            // Read the two bytes for the multiplier
            uint8_t high_byte = client.read();
            uint8_t low_byte = client.read();

            // Combine bytes into a 16-bit integer (assumes Big Endian from Pi)
            logging_frequency_multiplier = (high_byte << 8) | low_byte;

            // Safety check: multiplier must be at least 1
            if (logging_frequency_multiplier == 0) {
                logging_frequency_multiplier = 1;
            }

            // Only reset and print if the mode is actually changing
            if (!DataLoggingMode) {
                log_head = 0;
                log_tail = 0;
                log_queue_overflow = false;
                DataLoggingMode = true;
                Serial.print("[ETH_CMD] Logging ENABLED. Freq Multiplier: ");
                Serial.println(logging_frequency_multiplier);
            }
        }
        // --- Logic Branch 2: Turn Logging OFF ---
        else if (state == 0x00) {
            if (DataLoggingMode) {
                Serial.println("[ETH_CMD] Logging DISABLED. Flushing data...");
                flushLogDataToServer(); // This sets DataLoggingMode to false and flushes
            }
        }
    }
}

/**
 * @brief Sends Batch Data 1 (current line number) as a status update.
 *
 * This function is triggered by the send_batchdata_1 flag from an ISR and
 * sends a small, 5-byte packet containing a unique header and the 4-byte
 * line number.
 */
void sendBatchData1() {
    if (!ensureConnection()) {
        Serial.println("[ERROR] Cannot send Batch Data 1, no connection!");
        send_batchdata_1 = false; // Reset flag even on failure
        return;
    }

    // 1. Send the unique packet header.
    client.write(LINE_NUMBER_UPDATE_HEADER);

    // 2. Create a local, non-volatile copy to send safely.
    uint32_t lineNumberCopy = current_line_number;

    // 3. Send the 4-byte payload.
    client.write((uint8_t*)&lineNumberCopy, sizeof(lineNumberCopy));

    // 4. Reset the flag now that the data has been sent.
    send_batchdata_1 = false;
}

void handleSerialCommands() {
    // Check if there is any data available to read from the serial port.
    if (Serial.available() > 0) {
        // Read the incoming string until a newline character is received.
        String input = Serial.readStringUntil('\n');
        input.trim(); // Remove any whitespace

        // --- Command to turn ON data logging ---
        if (input.equalsIgnoreCase("L1")) {
            if (!DataLoggingMode) {
                log_head = 0; // Reset buffer pointers for a fresh start
                log_tail = 0;
                log_queue_overflow = false;
                DataLoggingMode = true;
                Serial.println("[CMD] Data logging ENABLED.");
            } else {
                Serial.println("[CMD] Data logging is already ON.");
            }
        }

        // --- Command to turn OFF data logging and flush data ---
        else if (input.equalsIgnoreCase("L0")) {
            if (DataLoggingMode) {
                Serial.println("[CMD] Data logging DISABLED. Flushing data...");
                // The flushLogDataToServer() function already sets DataLoggingMode to false.
                flushLogDataToServer();
            } else {
                Serial.println("[CMD] Data logging is already OFF.");
            }
        }

        // --- Unknown command ---
        else {
            Serial.print("Unknown command: ");
            Serial.println(input);
        }
    }
}
