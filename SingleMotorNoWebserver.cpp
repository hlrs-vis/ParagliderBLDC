/*
  PlatformIO Configuration:
  [env:esp32dev]
  platform = https://github.com/pioarduino/platform-espressif32/releases/download/51.03.04/platform-espressif32.zip
  board = esp32dev
  framework = arduino
  lib_deps =
      askuric/Simple FOC @ 2.4.0
      madhephaestus/ESP32Encoder @ ^0.11.7
  monitor_speed = 115200
  upload_port = COM17
*/

#include <ESP32Encoder.h>
#include <SimpleFOC.h>
#include <Wire.h>

#define PINA 15
#define PINB 4
#define PINI 2

#define UNDERVOLTAGE_THRES 11.1

void board_check();
float get_vin_Volt();
void board_init();

// ============================================================================
// HARDWARE CURRENT LIMITS & SAFETY
// ============================================================================
const float FFB_CURRENT_LIMIT     = 4.20f; // [Amps] Safe continuous current ceiling (5A Driver limit)

// ============================================================================
// PHASE 1: REEL-IN AT SPECIFIC CURRENT & STALL DETECTION (PURE TORQUE MODE)
// ============================================================================
float REEL_IN_CURRENT           = 1.8f;  // [Amps] Specific torque current used to pull handle home
const float STALL_VEL_THRESHOLD = 3.0f;  // [rad/s] Velocity threshold confirming mechanical stop
const uint32_t STALL_TIME_MS    = 200;   // [ms] Duration velocity must remain low to confirm home

// ============================================================================
// PHASE 2: DYNAMIC TORQUE RESISTANCE TUNING (DIGRESSIVE / SLOW-GROWTH MODE)
// ============================================================================
float I_rest                      = 0.50f; // [Amps] Baseline tension at home stop
float k_spring                    = 5.25f; // [Amps] Force scaling factor for pull displacement
float k_exponent                  = 0.65f; // [< 1.0] Digressive curve: force increases slower as displacement grows
float d_velocity                  = 0.08f; // [Amps/(rad/s)] Dynamic current added to resist speed

// Working variables
float current_angle1 = 0;
float spring_start_angle = 0;
bool windMotor = false;
uint32_t stall_timer = 0;
uint32_t reel_print_timer = 0; // Timer for non-blocking serial outputs during reel-in
uint32_t ffb_print_timer = 0;  // Timer for non-blocking serial outputs during FFB mode

bool flag_under_voltage = false;
uint32_t prev_millis_board;

// Hardware Objects
ESP32Encoder centreEncoder;
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1);

InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, 35, 34);
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor1, cmd); }
void doReelCurrent(char* cmd) { command.scalar(&REEL_IN_CURRENT, cmd); }
void doSpring(char* cmd) { command.scalar(&k_spring, cmd); }
void doExponent(char* cmd) { command.scalar(&k_exponent, cmd); }
void doDamp(char* cmd) { command.scalar(&d_velocity, cmd); }
void doRest(char* cmd) { command.scalar(&I_rest, cmd); }

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("===== BOOTING PARAGLIDER FFB (PURE CURRENT MODE) =====");

  board_init();

  pinMode(PINA, INPUT_PULLUP);
  pinMode(PINB, INPUT_PULLUP);
  centreEncoder.attachFullQuad(PINA, PINB);
  centreEncoder.setCount(0);

  // Setup AS5600 Encoder
  I2Ctwo.begin(23, 5, 400000UL); // Fast 400kHz I2C
  sensor1.init(&I2Ctwo);
  motor1.linkSensor(&sensor1);

  // Driver Setup
  driver1.voltage_power_supply = get_vin_Volt();
  driver1.pwm_frequency = 25000; // 25kHz PWM suppresses low-inductance noise
  driver1.init();

  // Current Sense Setup
  current_sense.linkDriver(&driver1);
  if (!current_sense.init()) {
    Serial.println("FATAL: Current sense init failed!");
    return;
  }

  motor1.linkDriver(&driver1);
  motor1.linkCurrentSense(&current_sense);
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // --------------------------------------------------------------------------
  // PERMANENT CURRENT / TORQUE CONTROL CONFIGURATION
  // --------------------------------------------------------------------------
  motor1.torque_controller = TorqueControlType::foc_current;
  motor1.controller = MotionControlType::torque; // Permanent Pure Current Mode

  // Inner Current Loop PID Tuning (Softened to reduce ESP32 ADC noise hunting)
  motor1.PID_current_q.P = 0.08f;
  motor1.PID_current_q.I = 2.0f;
  motor1.PID_current_d.P = 0.08f;
  motor1.PID_current_d.I = 2.0f;

  // Filtering (Smoothes out current noise and AS5600 velocity jitter)
  motor1.LPF_current_q.Tf = 0.03f; 
  motor1.LPF_current_d.Tf = 0.03f;
  motor1.LPF_velocity.Tf  = 0.020f; // Smoothes AS5600 velocity spikes

  // Driver & Voltage Boundaries
  motor1.voltage_limit = 9.00f;        // Voltage headroom for current loop
  motor1.voltage_sensor_align = 0.35f; // Anti-brownout calibration limit
  motor1.updateCurrentLimit(FFB_CURRENT_LIMIT); 
  motor1.phase_resistance = 0.08f;

  motor1.init();
  motor1.initFOC();

  // Serial commands for Live Tuning
  command.add('M', doMotor, "motor");
  command.add('W', doReelCurrent, "reel-in amperage (A)");
  command.add('S', doSpring, "spring current gain (A)");
  command.add('E', doExponent, "ramp exponent (<1.0 for slow taper)");
  command.add('D', doDamp, "velocity resistance (A/(rad/s))");
  command.add('I', doRest, "resting baseline current (A)");

  Serial.printf("Motor aligned. Continuous Current Mode ACTIVE. Reeling in at %.2f A...\n", REEL_IN_CURRENT);
}

void loop() {
  // High-frequency FOC calculation
  motor1.loopFOC();

  current_angle1 = sensor1.getAngle();

  // --------------------------------------------------------------------------
  // PHASE 1: TORQUE REEL-IN & STALL DETECTION AT SPECIFIC CURRENT
  // --------------------------------------------------------------------------
  if (!windMotor) {
    // Command the fixed specific reel-in current
    motor1.move(-REEL_IN_CURRENT);

    // Read current rotation speed
    float current_vel = fabs(sensor1.getVelocity());

    // ONLY PRINT VELOCITY AND THRESHOLD STATUS DURING WINDING SCENARIO
    if (millis() - reel_print_timer >= 200) {
      reel_print_timer = millis();
      bool threshold_met = (current_vel < STALL_VEL_THRESHOLD);
      Serial.printf("[WINDING] Vel: %.2f rad/s | Threshold Met: %s\n", 
                    current_vel, threshold_met ? "YES" : "NO");
    }

    // Check if the motor stopped spinning after hitting the end stop
    if (current_vel < STALL_VEL_THRESHOLD) {
      if (stall_timer == 0) {
        stall_timer = millis();
      } else if (millis() - stall_timer >= STALL_TIME_MS) {
        // Handle stalled at the hard stop under specified reel current
        spring_start_angle = current_angle1;
        windMotor = true;

        Serial.printf("\n>>> STALL DETECTED at hard stop! <<<\n");
        Serial.printf("Home position locked at %.2f rad. Dynamic FFB ACTIVE.\n\n", spring_start_angle);
      }
    } else {
      stall_timer = 0; // Still rotating, reset stall timer
    }

  // --------------------------------------------------------------------------
  // PHASE 2: DYNAMIC FORCE FEEDBACK CURRENT CONTROL (SLOW GROWING TORQUE)
  // --------------------------------------------------------------------------
  } else { 
    // Distance pulled away from home stop (positive when pulled out)
    float displacement = current_angle1 - spring_start_angle;
    
    // Line pull velocity (positive when pulling out)
    float velocity = sensor1.getVelocity();

    float target_current = I_rest; // Start at baseline holding current

    if (displacement > 0.0f) {
      // Slow-growing torque curve using fractional power exponent (k_exponent < 1.0):
      // Force increases progressively slower the further away the handle is pulled.
      target_current += k_spring * powf(displacement, k_exponent);

      // Dynamic current increase to resist speed/movement
      if (velocity > 0.0f) {
        target_current += d_velocity * velocity;
      }
    }

    // Safely clamp the current command within continuous threshold
    target_current = constrain(target_current, 0.0f, FFB_CURRENT_LIMIT);

    // PRINT TARGET AND ACTUAL CURRENT PERIODICALLY IN FFB MODE (EVERY 200 MS)
    if (millis() - ffb_print_timer >= 200) {
      ffb_print_timer = millis();
      float actual_current = fabs(motor1.current.q);
      Serial.printf("[FFB MODE] Target Current: %.2f A | Actual Current: %.2f A\n", 
                    target_current, actual_current);
    }

    // Direct negative q-axis current command to pull back towards home
    motor1.move(-target_current);
  }

  // Safety checks & Serial updates
  board_check();
  if (!flag_under_voltage) command.run();
}

void board_init() {
  analogReadResolution(12);

  float VIN_Volt = get_vin_Volt();
  while (VIN_Volt <= UNDERVOLTAGE_THRES) {
    VIN_Volt = get_vin_Volt();
    delay(100);
    Serial.printf("Waiting for power on... Voltage: %.2f V\n", VIN_Volt);
  }
  Serial.printf("Power rail OK: %.2f V\n", VIN_Volt);
}

float get_vin_Volt() { 
  return analogReadMilliVolts(13) * 8.5 / 1000.0; 
}

void board_check() {
  uint32_t curr_millis = millis();
  static uint8_t enableState = 1;

  if (curr_millis - prev_millis_board >= 500) {
    float vin_Volt = get_vin_Volt();

    if (vin_Volt < UNDERVOLTAGE_THRES) {
      flag_under_voltage = true;
      motor1.disable();
      enableState = 0;
      Serial.printf("[SAFETY ALERT] Undervoltage detected: %.2f V\n", vin_Volt);
    } else if (enableState == 0) {
      flag_under_voltage = false;
      enableState = 1;
      motor1.enable();
    }
    prev_millis_board = curr_millis;
  }
}
