#include <ESP32Encoder.h>
#include <SimpleFOC.h>
#include <Wire.h>

// actual wiring currently has PINB on IO4 and PINA on IO15
#define PINA 15
#define PINB 4
#define PINI 2

// Setting the alarm voltage
#define UNDERVOLTAGE_THRES 11.1

// --- ADDED: Filter for velocity ---
LowPassFilter velFilter = LowPassFilter(0.05f); 

void board_check();
float get_vin_Volt();
void board_init();

// --- MODIFIED: Softened haptic variables to stop violent shaking ---
float spring_constant = 0.1;       // Dropped from 0.8
float non_linear_stiffness = 0.0;  // Dropped from 0.2
float damping1 = 0.0;              // Dropped from 0.01 to test I2C noise
float handle_weight_force = 0.15;  
float angle_error1 = 0;
float current_angle1 = 0;
float torque_input1 = 0;
float curr_Velocity1 = 0;

// Wind spool on startup variables
float spring_start_angle = 0;
float windUpRad = 50; 
bool windMotor = false;

// Board safety check variables
bool flag_under_voltage = false;
uint32_t prev_millis_board;

// Telemetry timer
uint32_t prev_millis_limits = 0;

// Objects
ESP32Encoder centreEncoder;
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1);

InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, 35, 34);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor1, cmd); }
void doSpring(char* cmd) { command.scalar(&spring_constant, cmd); }

void setup() {
  Serial.begin(115200);
  Serial.println("===== BOOT =====");

  board_init();

  pinMode(PINA, INPUT_PULLUP);
  pinMode(PINB, INPUT_PULLUP);
  centreEncoder.attachFullQuad(PINA, PINB);
  centreEncoder.setCount(0);

  delay(500);

  I2Ctwo.begin(23, 5, 400000UL);  
  sensor1.init(&I2Ctwo);
  motor1.linkSensor(&sensor1);

  driver1.voltage_power_supply = get_vin_Volt();
  driver1.init();

  current_sense.linkDriver(&driver1);
  current_sense.init();

  motor1.linkDriver(&driver1);
  motor1.linkCurrentSense(&current_sense);
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.useMonitoring(Serial); 

  windMotor = false;
  motor1.torque_controller = TorqueControlType::voltage;

  // Direct Voltage Limits (Bypassing estimated current mode)
  motor1.voltage_limit = 1.0; 
  
  // --- ADDED: Drastically lower PID gains for RS2205 windup ---
  motor1.PID_velocity.P = 0.05; 
  motor1.PID_velocity.I = 1.0;  
  motor1.P_angle.P = 2.0;       
  motor1.velocity_limit = 5.0; // Slightly faster limit for smooth windup

  motor1.init();
  motor1.initFOC();

  command.add('M', doMotor, "motor");
  command.add('S', doSpring, "spring");
  command.add('N', doSpring, "non_linear_stiffness");

  Serial.println(F("Motor ready."));
  motor1.controller = MotionControlType::angle;
}

float computeTorqueCommand(float angle, float velocity, float damping) {
  // Deadband to prevent buzzing at exact zero
  if (fabs(angle) < 0.005f) { 
    return 0.0f;
  }

  float nonlinear_force = non_linear_stiffness * angle * fabs(angle);
  float torque = (-spring_constant * angle) - nonlinear_force - (damping * velocity) + handle_weight_force;
  return torque;
}

// Diagnostic Monitor
void check_motor_limits() {
  uint32_t curr_millis = millis();
  
  // Check every 100ms to avoid I2C/Serial congestion
  if (curr_millis - prev_millis_limits >= 100) { 
    prev_millis_limits = curr_millis;

    // Check Direct Voltage Cap
    if (fabs(torque_input1) >= motor1.voltage_limit) {
      Serial.print("[CAP LIMIT] Voltage target saturated! Commanded: ");
      Serial.print(fabs(torque_input1));
      Serial.print(" | Max Allowed: ");
      Serial.println(motor1.voltage_limit);
    }

    // Check Velocity Cap (Triggers warning at 95% of limit)
    if (fabs(curr_Velocity1) >= (motor1.velocity_limit * 0.95f)) { 
      Serial.print("[CAP LIMIT] Velocity maxed out! Actual: ");
      Serial.print(fabs(curr_Velocity1));
      Serial.print(" rad/s | Limit: ");
      Serial.println(motor1.velocity_limit);
    }
  }
}

bool correctAngleOnFirstLoop = false;
void loop() {
  motor1.loopFOC();
  
  current_angle1 = sensor1.getAngle();
  angle_error1 = current_angle1 - spring_start_angle;
  
  // Filtered velocity for damping math
  curr_Velocity1 = velFilter(sensor1.getVelocity()); 

  if (!correctAngleOnFirstLoop) {
    while (angle_error1 > PI) angle_error1 -= 2 * PI;
    while (angle_error1 < -PI) angle_error1 += 2 * PI;
    correctAngleOnFirstLoop = true;
  }

  if (!windMotor) {
    motor1.move(windUpRad);
    if (fabs(current_angle1 - windUpRad) < 0.1) {
      spring_start_angle = current_angle1;
      motor1.controller = MotionControlType::torque;
      windMotor = true;
    }
  } else {
    torque_input1 = computeTorqueCommand(angle_error1, curr_Velocity1, damping1);
    motor1.move(torque_input1);
  }

  // Run the limit check
  if (windMotor) { 
      check_motor_limits(); 
  }

  board_check();
  if (!flag_under_voltage) command.run();
}

void board_init() {
  analogReadResolution(12);
  float VIN_Volt = get_vin_Volt();
  while (VIN_Volt <= UNDERVOLTAGE_THRES) {
    VIN_Volt = get_vin_Volt();
    delay(100);
  }
}

float get_vin_Volt() { return analogReadMilliVolts(13) * 8.5 / 1000; }

void board_check() {
  uint32_t curr_millis = millis();
  static uint8_t enableState = 0;
  if (curr_millis - prev_millis_board >= 1000) {
    float vin_Volt = get_vin_Volt();
    flag_under_voltage = (vin_Volt < UNDERVOLTAGE_THRES);
    if (flag_under_voltage) motor1.disable();
    else if (0 == enableState) { enableState = 1; motor1.enable(); }
    prev_millis_board = curr_millis;
  }
}