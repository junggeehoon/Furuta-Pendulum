#include <SimpleFOC.h>

// ----- Hardware Objects -----
BLDCMotor motor = BLDCMotor(7, 15.2, 34.5);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, 39, 36);

const int encoderPinA = 23; // A channel
const int encoderPinB = 5;  // B channel

// Set the encoder cycles per revolution
const int encoderCPR = 1000; 

// Set Torque constant Nm/A
const float torque_constant = 0.277;

volatile long encoderCount = 0;

// Control parameters and state estimates
float k1, k2, k3, k4;
float a_e, b_e, c_e, d_e;
float lambda_e;
float x_e1, x_e2;
float torque_max;
unsigned long lastUpdate = 0; 

// Assign parameters and initialize state estimates.
void assignParameters() {
  lambda_e = 50;
  a_e = -lambda_e;
  b_e = -lambda_e * lambda_e;
  c_e = 1;
  d_e = lambda_e;

  k1 = -0.0707;
  k2 = 3.9537;
  k3 = -0.0873;
  k4 = 0.3890;

  torque_max = 0.2;

  // Initialize state estimates
  x_e1 = 0;
  x_e2 = 0;
}

void initSensor() {
  I2Cone.begin(19, 18, 400000UL);
  sensor.init(&I2Cone);
  motor.linkSensor(&sensor);
}

void initDriver() {
  driver.voltage_power_supply = 20;
  driver.init();
  motor.linkDriver(&driver);
  motor.current_limit = 1;
}

void initCurrentSensor() {
  current_sense.linkDriver(&driver);
  current_sense.init();
  motor.linkCurrentSense(&current_sense);
}

void configureMotor() {
  // Sensor configuration
  motor.sensor_direction = Direction::CCW;
  motor.zero_electric_angle = 3.7214;
  
  // Control mode and controller
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  
  // Q axis
  motor.PID_current_q.P = 0.4;
  motor.PID_current_q.I = 50;
  
  // D axis
  motor.PID_current_d.P = 0.4;
  motor.PID_current_d.I = 50;
  
  // Low-pass filter settings for current sensor
  motor.LPF_current_q.Tf = 0.002;
  motor.LPF_current_d.Tf = 0.002;
}

// Interrupt Service Routine for encoder channel A
// Using RISING edge simplifies the logic since the ISR is triggered only on a LOW-to-HIGH transition.
void IRAM_ATTR encoderISR() {
  // Determine direction by reading channel B
  if (digitalRead(encoderPinB) == LOW) {
    encoderCount++;  // Clockwise: increment count
  } else {
    encoderCount--;  // Counterclockwise: decrement count
  }
}

// Modified readAngle() function that safely reads the encoder count
float readAngle() {
  noInterrupts();           // Disable interrupts to ensure a consistent read
  long count = encoderCount;
  interrupts();             // Re-enable interrupts
  float angle = (count * 2.0 * _PI) / encoderCPR;
  return angle;
}

void initMotorSystem() {
  motor.init();
  motor.initFOC(); // Align sensor and start FOC
}

void setup() {
  Serial.begin(115200);
  lastUpdate = millis();

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Assign parameters
  assignParameters();

  // Initialize sensor, driver, current sensor, and motor parameters
  initSensor();
  initDriver();
  initCurrentSensor();
  configureMotor();

  // Initialize motor control system
  initMotorSystem();

  // Attach interrupt for encoder channel A (rising edge)
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);

  Serial.println(F("Motor ready."));
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();

  // Run the FOC algorithm
  motor.loopFOC();

  sensor.update();
  float y1 = sensor.getAngle(); // Motor angle
  float y2 = readAngle() - _PI; // Pendulum angle

  float dt = (currentTime - lastUpdate) * 1e-3; 
  lastUpdate = currentTime;  

  float y_dothat1 = c_e * x_e1 + d_e * y1;
  float y_dothat2 = c_e * x_e2 + d_e * y2;

  // Update state estimates using Euler integration
  x_e1 = x_e1 + dt * (a_e * x_e1 + b_e * y1);
  x_e2 = x_e2 + dt * (a_e * x_e2 + b_e * y2);

  float tau;
  if (abs(y2) < 0.2) {
    tau = -k1 * y1 - k2 * y2 - k3 * y_dothat1 - k4 * y_dothat2;
    Serial.print("Control Mode ");
    Serial.println(tau * torque_constant);
  } else {
    Serial.println(y2);
    tau = 0;
  }

  // u = min(max(u, -torque_max), torque_max);
  float u = tau * torque_constant;
  u = min(max(u, -0.7f), 0.7f);
  motor.move(u);
}