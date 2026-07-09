/* The platform.ini file that works with this code is as follows. If you get errors relating to needing updated versions,   
delete the .pio folder and do a full clean. 

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

/* This current version has the wind spool on startup working properly even with the weight of the handle/string. 
The spring does not work as it once did. It is significantly weaker and acts more like a yoyo than a spring.
Current and voltage limits were increased for enough torque to wind up the spool, but estimated current mode isn't
yielding the same magnitude of corrective torque as voltage mode

  */
#include <ESP32Encoder.h>
#include <SimpleFOC.h>
#include <Wire.h>

// actual wiring currently has PINB on IO4 and PINA on IO15
#define PINA 15
#define PINB 4
#define PINI 2

// encoder instantiation: pina, pinb, index?
ESP32Encoder centreEncoder;

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);

// magnetic rotary encoder that senses internal motor position
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// Motor parameters
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

// Command settings
float target_angle = 0;
float spring_constant = 0.25;
float angle_error = 0;
float angle_error1 = 0;
float current_angle = 0;
float current_angle1 = 0;
float torque_input = 0;
float torque_input1 = 0;
float damping = 0.3;
float damping1 = 0.05;
float curr_Velocity = 0;
float curr_Velocity1 = 0;
uint32_t prev_millis_board;

// Setting the alarm voltage
#define UNDERVOLTAGE_THRES 11.1

Commander command = Commander(Serial);

void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }
void doSpring(char* cmd) { command.scalar(&spring_constant, cmd); }

void board_check();
float get_vin_Volt();
void board_init();

bool flag_under_voltage = false;

void setup() {
  Serial.begin(115200);

  Serial.println("===== BOOT =====");

  board_init();

  pinMode(PINA, INPUT_PULLUP);
  pinMode(PINB, INPUT_PULLUP);

  // ESP32Encoder::useInternalWeakPullResistors = puType::up;
  centreEncoder.attachFullQuad(PINA, PINB);

  centreEncoder.setCount(0);

  delay(500);

  // I2Cone.begin(19, 18, 400000UL);  // AS5600_M0
  I2Ctwo.begin(23, 5, 400000UL);  // AS5600_M1

  Serial.println("Scanning...");

  for (int addr = 1; addr < 127; addr++) {
    I2Ctwo.beginTransmission(addr);

    if (I2Ctwo.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
    }

    delay(5);
  }

  Serial.println("Done");
  // sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);

  // Connect the motor object and the sensor object
  // motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  // Supply voltage setting [V]
  // driver.voltage_power_supply = get_vin_Volt();
  // driver.init();

  driver1.voltage_power_supply = get_vin_Volt();
  // driver1.pwm_frequency = 1000;  higher frequency eliminates noise
  // apparently?
  driver1.init();

  // Connect the motor and driver objects
  // motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);

  // FOC model selection
  // motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // real time output of motor variables to serial terminal
  // motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  //-------Torque Control and Variables Setup------

  // foc_current wont work bc rs2205 motor doesn't have current sensors
  // motor.torque_controller = TorqueControlType::voltage;
  motor1.torque_controller = TorqueControlType::estimated_current;

  // [V] Please modify and check this value carefully, excessive voltage
  // and current may cause the driver board to burn out!!!
  // motor.voltage_limit = 0.3;  // Maximum voltage [V]
  motor1.voltage_limit = 5.0;      // Maximum voltage [V]
  motor1.updateCurrentLimit(4.5);  // max current limit
  motor1.phase_resistance = 0.1f;

  // manually adjust PID values, but do we also need LPF?
  motor1.PID_velocity.P = 0.5;
  motor1.PID_velocity.I = 4.0;
  motor1.LPF_velocity.Tf = 0.01;

  // Set a maximum speed limit
  // this is speed at which motor responds
  // motor.velocity_limit = 10;
  motor1.velocity_limit = 2;

  // Initialize the motor
  // motor.init();
  motor1.init();
  // Initialize FOC
  // motor.initFOC();
  motor1.initFOC();

  // creating command (command id, function pointer, command label)
  command.add('T', doTarget, "target angle");
  command.add('M', doMotor, "motor");
  command.add('S', doSpring, "spring");

  Serial.println(F("Motor ready."));
  Serial.println(
      F("Set the target velocity, voltage, and virtual spring constant using "
        "serial terminal:"));

  //-------Winding String Upon Startup--------
  motor1.controller = MotionControlType::angle;
}

void loop() {
  static bool windMotor = false;

  static float zeroAngleAfterWinding;
  // polling continuously I2C encoders
  // motor.loopFOC();
  motor1.loopFOC();

  // torque = spring constant * (target angle - current angle)
  // current_angle = sensor.getAngle();
  current_angle1 = sensor1.getAngle();

  // curr_Velocity = sensor.getVelocity();
  curr_Velocity1 = sensor1.getVelocity();

  // still need to wound, then wound
  if (!windMotor) {
    static float windUpRevolution = -48;

    motor1.move(windUpRevolution);
    if (fabs(current_angle1 - windUpRevolution) < 0.1) {
      windMotor = true;
      zeroAngleAfterWinding = current_angle1;
      motor1.controller = MotionControlType::torque;
    }
  } else {
    current_angle1 = sensor1.getAngle();
    // angle_error = (target_angle - current_angle);
    angle_error1 = (zeroAngleAfterWinding - current_angle1);
    curr_Velocity1 = sensor1.getVelocity();

    // torque_input = spring_constant * angle_error - curr_Velocity * damping;
    torque_input1 = spring_constant * angle_error1 - curr_Velocity1 * damping1;
    // instead of motor.move for motion control, need to generate PWM signals
    // motor.move(torque_input);
    motor1.move(torque_input1);
  }

  //----------Centre Encoder Readings----------
  long count = centreEncoder.getCount();
  Serial.print("Encoder Count: ");
  Serial.println(count);

  // When the voltage is lower than the set value, the motor will be disabled.
  board_check();

  // User Communications
  if (!flag_under_voltage) command.run();
}

void board_init() {
  pinMode(32, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);

  analogReadResolution(12);  // 12bit

  float VIN_Volt = get_vin_Volt();
  while (VIN_Volt <= UNDERVOLTAGE_THRES) {
    VIN_Volt = get_vin_Volt();
    delay(100);
    Serial.printf("Waiting for power on, current voltage%.2f\n", VIN_Volt);
  }
  Serial.printf("Calibrating motor...Current voltage%.2f\n", VIN_Volt);
}

// helper function for board_check
float get_vin_Volt() { return analogReadMilliVolts(13) * 8.5 / 1000; }

// making sure VIN voltage on board isn't overshooting
void board_check() {
  uint32_t curr_millis = millis();
  static uint8_t enableState = 0;

  if (curr_millis - prev_millis_board >= 1000) {
    float vin_Volt = get_vin_Volt();

    if (vin_Volt < UNDERVOLTAGE_THRES) {
      flag_under_voltage = true;
      enableState = 0;
      uint8_t count = 5;
      while (count--) {
        vin_Volt = get_vin_Volt();
        if (vin_Volt > UNDERVOLTAGE_THRES) {
          flag_under_voltage = false;
          break;
        }
      }
    } else {
      flag_under_voltage = false;
    }
    if (flag_under_voltage) {
      // motor.disable();
      motor1.disable();
    } else if (0 == enableState && flag_under_voltage == false) {
      enableState = 1;
      // motor.enable();
      motor1.enable();
    }
    prev_millis_board = curr_millis;
  }
}
