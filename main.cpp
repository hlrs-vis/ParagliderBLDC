// MKS ESP32 FOC V2.0 | Open Loop Position Example | Library:SimpleFOC 2.2.1 |
// Hardware:MKS ESP32 FOC V2.0 & MKS AS5600

// !!!Notice!!!
// ①Enter "T+number" in the serial port to set the position of the two motors.
// For example, if you want the motor to rotate to 180°, enter "T3.14" (180° in
// radians) ②When using your own motor, be sure to modify the default number of
// pole pairs, that is, the value in BLDCMotor(7), to the number of pole pairs
// of your own motor. ③Please set the correct voltage_limit value according to
// the selected motor. It is recommended to set it between 0.5 and 1.0 for the
// aircraft model motor and below 4 for the gimbal motor. Excessive voltage and
// current may burn out the driver board! ④The pid parameters of this routine
// can control the 2808 model aircraft motor. If you want to achieve better
// results or use other motors, please adjust the pid parameters yourself.

// Current implementation: Modelling the motors as springs with damping factor
/*
.ini platform note: use following versions
  [env:esp32dev]
  platform = espressif32@6.3.2
  board = esp32dev
  framework = arduino
  monitor_speed = 115200

  lib_deps =
      askuric/Simple FOC @ 2.2.1

  upload_port = COM15

*/

#include <SimpleFOC.h>

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
float spring_constant = 0.3;
float angle_error, angle_error1;
float current_angle, current_angle1;
float torque_input, torque_input1;
float damping = 0.3;
float damping1 = 0;
float currVelocity, currVelocity1;
uint32_t prev_millis;

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
  board_init();

  I2Cone.begin(19, 18, 400000UL);  // AS5600_M0
  I2Ctwo.begin(23, 5, 400000UL);   // AS5600_M1
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);

  // Connect the motor object and the sensor object
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  // Supply voltage setting [V]
  driver.voltage_power_supply = get_vin_Volt();
  driver.init();

  driver1.voltage_power_supply = get_vin_Volt();
  driver1.init();

  // Connect the motor and driver objects
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);

  // FOC model selection
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // real time output of motor variables to serial terminal
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  current_angle = sensor.getAngle();
  current_angle1 = sensor1.getAngle();

  // angle restricted 0 - 2pi
  angle_error = (target_angle - current_angle);
  torque_input = spring_constant * angle_error;
  torque_input1 = spring_constant * angle_error;

  // foc_current wont work bc rs2205 motor doesn't have current sensors
  motor.torque_controller = TorqueControlType::voltage;
  motor1.torque_controller = TorqueControlType::voltage;

  // Motion control mode settings
  motor.controller = MotionControlType::torque;
  motor1.controller = MotionControlType::torque;

  // [V] Please modify and check this value carefully, excessive voltage
  // and current may cause the driver board to burn out!!!
  motor.voltage_limit = 0.5;   // Maximum voltage [V]
  motor1.voltage_limit = 0.5;  // Maximum voltage [V]

  // Set a maximum speed limit
  // this is speed at which motor responds
  motor.velocity_limit = 20;
  motor1.velocity_limit = 20;

  // Initialize the motor
  motor.init();
  motor1.init();
  // Initialize FOC
  motor.initFOC();
  motor1.initFOC();

  // creating command (command id, function pointer, command label)
  command.add('T', doTarget, "target angle");
  command.add('M', doMotor, "motor");
  command.add('S', doSpring, "spring");

  Serial.println(F("Motor ready."));
  Serial.println(
      F("Set the target velocity, voltage, and virtual spring constant using "
        "serial terminal:"));
}

void loop() {
  // get up to date velocity
  sensor.update();
  sensor1.update();

  Serial.println(sensor.getVelocity());

  // torque = spring constant * (target angle - current angle)
  current_angle = sensor.getAngle();
  current_angle1 = sensor1.getAngle();

  angle_error = (target_angle - current_angle);
  angle_error1 = (target_angle - current_angle1);

  currVelocity = sensor.getVelocity();
  currVelocity1 = sensor1.getVelocity();

  torque_input = spring_constant * angle_error - currVelocity * damping;
  torque_input1 = spring_constant * angle_error1;

  // polling continuously I2C encoders
  motor.loopFOC();
  motor1.loopFOC();

  // instead of motor.move for motion control, need to generate PWM signals
  motor.move(torque_input);
  motor1.move(torque_input1);

  // When the voltage is lower than the set value, the motor will be disabled.
  board_check();

  // User Communications
  if (!flag_under_voltage) command.run();

  // Serial.print(sensor.getAngle());
  // Serial.print(" - ");
  // Serial.print(sensor1.getAngle());
  // Serial.println();
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

  if (curr_millis - prev_millis >= 1000) {
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
      motor.disable();
      motor1.disable();
    } else if (0 == enableState && flag_under_voltage == false) {
      enableState = 1;
      motor.enable();
      motor1.enable();
    }
    prev_millis = curr_millis;
  }
}
