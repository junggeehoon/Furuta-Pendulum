#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);                
BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,22);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

float x0 = 0;
static float x_e1 = 0.0f;    // internal state
const float lambda_e = 40.0f;  // "bandwidth" of differentiator

unsigned long lastUpdate = 0;  // time of last loop (ms)
unsigned long lastPrint = 0;   // time we last printed (ms)
unsigned long startTime;

// current sensor
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, 39, 36);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }

// Define a delay duration for the step input (in milliseconds)
unsigned long stepDelay = 3000;

void setup() { 
  I2Cone.begin(19,18, 400000UL); 
  sensor.init(&I2Cone);
  motor.linkSensor(&sensor);

  // driver configuration
  driver.voltage_power_supply = 20;
  driver.init();
  motor.linkDriver(&driver);
  current_sense.linkDriver(&driver);

  // current sensor initialization
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  motor.sensor_direction = Direction::CW;
  motor.zero_electric_angle = 3.7460;

  // set torque mode and controller type
  motor.torque_controller = TorqueControlType::foc_current; 
  motor.controller = MotionControlType::torque;


  // Debug
  motor.useMonitoring(Serial);
  SimpleFOCDebug::enable(&Serial);

  // FOC current control parameters for Q and D axes
  motor.PID_current_q.P = 0.4;
  motor.PID_current_q.I = 50;

  motor.PID_current_d.P = 0.4;
  motor.PID_current_d.I = 50;
  
  // Low pass filter parameters for current control
  motor.LPF_current_q.Tf = 0.002; 
  motor.LPF_current_d.Tf = 0.002; 

  Serial.begin(115200);
  motor.useMonitoring(Serial);
  SimpleFOCDebug::enable(&Serial);

  // Initialize motor and FOC
  motor.init();
  motor.initFOC();

  // Initially, set the target current to 0 A (so no immediate step)
  motor.target = 0.0;

  delay(500);

  lastPrint = millis();
  lastUpdate = millis();
  startTime = millis();
  lastUpdate = 0; 
  sensor.update();
  x0 = sensor.getAngle();
  x_e1 = -lambda_e * x0; 
  delay(1000);
}

void loop() {

  unsigned long currentTime = millis() - startTime;

  if (currentTime > 2000 && currentTime < 7000) {
    motor.target = 0.1;
  } else {
    motor.target = 0;
  }

  // Run the main FOC and motion control functions
  motor.loopFOC();
  motor.move();
  sensor.update();

  float x = sensor.getAngle();  // Get sensor angle


  float dt = (currentTime - lastUpdate) * 1e-3;  // convert ms to s
  lastUpdate = currentTime;  

  // Update the internal state using Euler integration
  if (dt > 1e-9) {
    float dx_e1 = -lambda_e * x_e1 - (lambda_e * lambda_e) * x;
    x_e1 += dx_e1 * dt;
  }

  // Compute the approximated derivative
  float y_dot_hat = x_e1 + lambda_e * x;

  // Print the approximate derivative every 10ms
  if ((currentTime - lastPrint) >= 10) {
    Serial.print(currentTime * 1e-3, 3);
    Serial.print(",");
    Serial.println(y_dot_hat);
    lastPrint = currentTime;
  }
}