/* The platform.ini file that works with this code is as follows. If you get
errors relating to needing updated versions, delete the .pio folder and do a
full clean.

[env:esp32dev]
platform =
https://github.com/pioarduino/platform-espressif32/releases/download/51.03.04/platform-espressif32.zip
board = esp32dev
framework = arduino

lib_deps =
    askuric/Simple FOC @ 2.4.0
    madhephaestus/ESP32Encoder @ ^0.11.7

monitor_speed = 115200
upload_port = COM17


*/

/* This current version has the wind spool on startup working properly even with
the weight of the handle/string. The spring does not work as it once did. It is
significantly weaker and acts more like a yoyo than a spring. Current and
voltage limits were increased for enough torque to wind up the spool, but still
need to tune PID variables in FOC current mode 

  */

#include <ESP32Encoder.h>
#include <SimpleFOC.h>
#include <Wire.h>

// actual wiring currently has PINB on IO4 and PINA on IO15
#define PINA 15
#define PINB 4
#define PINI 2

// Setting the alarm voltage
#define UNDERVOLTAGE_THRES 11.1

void board_check();
float get_vin_Volt();
void board_init();

// Torque calculation variables
float spring_constant = 5;
float spring_gain = 0.5;  // for extra stiffness the more you pull
float damping = 0.3;
float damping1 = 0.1;
float angle_error = 0;
float angle_error1 = 0;
float current_angle = 0;
float current_angle1 = 0;
float torque_input = 0;
float torque_input1 = 0;
float curr_Velocity = 0;
float curr_Velocity1 = 0;

// Wind spool on startup variables
float spring_start_angle = 0;
float windUpRad = -48;
bool windMotor = false;

// Board safety check variables
bool flag_under_voltage = false;
uint32_t prev_millis_board;

// Objects (encoder, sensors, motors, drivers)
ESP32Encoder centreEncoder;

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);

// magnetic rotary encoder that senses internal motor position
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// InlineCurrentSensor constructor
//  - shunt_resistor  - shunt resistor value
//  - gain  - current-sense op-amp gain
//  - phA   - A phase adc pin
//  - phB   - B phase adc pin
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, 35, 34);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }
void doSpring(char* cmd) { command.scalar(&spring_constant, cmd); }

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
  I2Ctwo.begin(23, 5, 100000UL);  // AS5600_M1

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

  current_sense.linkDriver(&driver1);

  if (current_sense.init()) {
    Serial.println("Current sense init success!");
  } else {
    Serial.println("Current sense init failed!");
    return;
  }

  // Connect the motor and driver objects
  // motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);

  motor1.linkCurrentSense(&current_sense);

  // FOC model selection
  // motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // real time output of motor variables to serial terminal
  // motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  //-------Torque Control and Parameters Setup------

  windMotor = false;
  zeroAngleAfterWinding = 0;

  // motor.torque_controller = TorqueControlType::voltage;
  motor1.torque_controller = TorqueControlType::foc_current;

  // foc current control parameters
  motor1.PID_current_q.P = 0.5;
  motor1.PID_current_q.I = 0.005;
  motor1.PID_current_d.P = 0.5;
  motor1.PID_current_d.I = 0.005;
  motor1.LPF_current_q.Tf = 0.01;
  motor1.LPF_current_d.Tf = 0.01;

  // [V] Please modify and check this value carefully, excessive voltage
  // and current may cause the driver board to burn out!!!
  // motor.voltage_limit = 0.3;  // Maximum voltage [V]
  motor1.voltage_limit = 0.5;      // Maximum voltage [V]
  motor1.updateCurrentLimit(1.0);  // max current limit
  motor1.phase_resistance = 0.05f;

  // motor.velocity_limit = 10;
  motor1.velocity_limit = 3;

  motor1.PID_velocity.P = 0.2;  // higher than 0.2 = vibrations
  motor1.PID_velocity.I = 0.5;
  motor1.LPF_velocity.Tf = 0.15;

  motor1.init();
  // Initialize FOC
  // motor.initFOC();
  motor1.initFOC();

  delay(1000);

  motor1.voltage_limit = 4.5;  // Maximum voltage [V]
  motor1.updateCurrentLimit(2.5);

  // creating commands (command id, function pointer, command label)
  char motor_id = 'M';
  command.add(motor_id, doMotor, "motor");
  command.add('S', doSpring, "spring");

  motor1.monitor_start_char = motor_id;
  motor1.monitor_end_char = motor_id;

  Serial.println(F("Motor ready."));
  Serial.println(
      F("Set the target velocity, voltage, and virtual spring constant using "
        "serial terminal:"));

  //-------Winding String Upon Startup--------
  motor1.controller = MotionControlType::angle;
}

float computeTorqueIntoFOCCurrent(float angle, float velocity, float damping) {
  float torque = -spring_constant * angle;
  // float focCurrent = torque / 0.00415;
  return torque;
}

int count;
bool correctAngleOnFirstLoop = false;
void loop() {
  motor1.loopFOC();
  motor1.monitor();

  current_angle1 = sensor1.getAngle();
  angle_error1 = current_angle1 - spring_start_angle;
  curr_Velocity1 = sensor1.getVelocity();

  // wrap angle because start angle and current are WAY OFF
  if (!correctAngleOnFirstLoop) {
    while (angle_error1 > PI) {
      angle_error1 -= 2 * PI;
    }

    while (angle_error1 < -PI) {
      angle_error1 += 2 * PI;
    }
  }
  correctAngleOnFirstLoop = true;

  /*
  count++;
  if (count >= 500) {
    Serial.printf("Torque: ");
    Serial.println(torque_input1);
    Serial.print("Spring Start Angle: ");
    Serial.println(spring_start_angle);
    Serial.print("Angle: ");
    Serial.println(current_angle1);
    Serial.print("Current velocity: ");
    Serial.println(curr_Velocity1);

    count = 0;
  }*/

  // still need to wind, then wound
  if (!windMotor) {
    motor1.move(windUpRad);
    if (fabs(current_angle1 - windUpRad) < 0.1) {
      motor1.move(0);
      delay(200);

      spring_start_angle = current_angle1;

      // switch to torque mode now for "spring" behaviour
      motor1.PID_velocity.reset();
      motor1.controller = MotionControlType::torque;

      windMotor = true;
    }
  } else {  //----ENTERING TORQUE MODE----
    torque_input1 =
        computeTorqueIntoFOCCurrent(angle_error1, curr_Velocity1, damping1);

    motor1.move(torque_input1);
  }

  // When the voltage is lower than the set value, the motor will be disabled.
  board_check();

  // User Communications
  if (!flag_under_voltage) command.run();
}

void board_init() {
  pinMode(32, INPUT_PULLUP);
  digitalWrite(32, LOW);
  pinMode(33, INPUT_PULLUP);
  digitalWrite(33, LOW);
  pinMode(25, INPUT_PULLUP);
  digitalWrite(25, LOW);
  pinMode(26, INPUT_PULLUP);
  digitalWrite(26, LOW);
  pinMode(27, INPUT_PULLUP);
  digitalWrite(27, LOW);
  pinMode(14, INPUT_PULLUP);
  digitalWrite(14, LOW);

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
