#include <SimpleFOC.h>
#include <WiFi.h>
#include <AsyncTCP.h>  
#include <ESPAsyncWebServer.h>

const char* ssid = "WIFI";
const char* password = "PASSWORD";

AsyncWebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Furuta Pendulum Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin: 20px;}
    .btn { display: inline-block; padding: 15px 30px; font-size: 24px; cursor: pointer; text-align: center;
           text-decoration: none; outline: none; color: #fff; background-color: #4CAF50; border: none;
           border-radius: 15px; box-shadow: 0 9px #999; }
    .btn:hover {background-color: #3e8e41}
    .btn:active { background-color: #3e8e41; box-shadow: 0 5px #666; transform: translateY(4px); }
    .btn-stop {background-color: #f44336;}
    .btn-stop:hover {background-color: #da190b;}
  </style>
</head>
<body>
  <h1>Furuta Pendulum Control</h1>
  <p><button onclick="sendCommand('s')" class="btn">Start</button></p>
  <p><button onclick="sendCommand('q')" class="btn btn-stop">Stop</button></p>
  <p>Status: <span id="status">--</span></p>
<script>
function sendCommand(command) {
  var xhr = new XMLHttpRequest();
  var url = "/" + command;
  xhr.open("GET", url, true);
  xhr.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("status").innerHTML = this.responseText;
    }
  };
  xhr.send();
}
</script>
</body>
</html>
)rawliteral";

// Motor & Driver
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(26,27,14,12);

// Sensors
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, 35, 34);
Encoder pendulum = Encoder(5, 23, 1000);

// Encoder Interrupt handlers
void doA(){pendulum.handleA();}
void doB(){pendulum.handleB();}

const float torque_constant = 0.277;;
const float torque_max = 0.2;

const float J2 = 0.000022803;
const float m = 0.0128;
const float g = 9.80665;
const float l = 0.035682;

// "catch" controller
const float k1_1 = -0.01;
const float k2_1 = 1.56220586;
const float k3_1 = -0.00996822;
const float k4_1 = 0.04673844;

// "balance" controller
const float k1 = -0.0230;
const float k2 = 1.5627;
const float k3 = -0.0166;
const float k4 = 0.0467;
const float k5 = -0.0087;

float initial_position = 0;
unsigned long last_micros = 0;
int count = 0;
bool balance = false;
unsigned long timeEnteredTightZone = 0;
static float sigma = 0.0f;

// Low Pass Filters
LowPassFilter LPF_Motor = LowPassFilter(0.08);
LowPassFilter LPF_Pendulum = LowPassFilter(0.08);

bool isMotorEnabled = true;

float constrainAngle(float x) {
    x = fmod(x + _PI, _2PI);
    if (x < 0) x += _2PI;
    return x - _PI;
}

void resetExperiment() {
  motor.move(0);
  sigma = 0;
  initial_position = sensor.getAngle();
}

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // --- Initialize WiFi ---
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // --- Web Server Routes ---
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/s", HTTP_GET, [](AsyncWebServerRequest *request){
    motor.PID_current_q.reset();
    motor.PID_current_d.reset();
    motor.enable();
    isMotorEnabled = true;
    Serial.println("Motor ENABLED via Web.");
    request->send(200, "text/plain", "Motor Started");
  });

  server.on("/q", HTTP_GET, [](AsyncWebServerRequest *request){
    motor.disable();
    isMotorEnabled = false;
    Serial.println("Motor DISABLED via Web. Type 's' to re-enable.");
    request->send(200, "text/plain", "Motor Stopped");
  });

  server.begin();

  pendulum.init();
  pendulum.enableInterrupts(doA, doB);

  delay(500);
  
  I2Cone.begin(19, 18, 400000UL);
  sensor.init(&I2Cone);

  initial_position = sensor.getAngle();

  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 20;
  delay(250);
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_limit = 20;
  motor.current_limit = 1;
  motor.velocity_limit = 25;

  current_sense.linkDriver(&driver);
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  motor.PID_current_q.P = 2;
  motor.PID_current_q.I = 80;
  motor.PID_current_d.P = 2;
  motor.PID_current_d.I = 80;
  motor.LPF_current_q.Tf = 0.08;
  motor.LPF_current_d.Tf = 0.08;

  motor.init();
  motor.zero_electric_angle = 0.5262;
  motor.sensor_direction = Direction::CCW;
  motor.initFOC();

  motor.PID_current_q.reset();
  motor.PID_current_d.reset();

  last_micros = micros();
  delay(250);
  Serial.println("Furuta Pendulum Ready");
}

void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi dropped, reconnecting…");
    WiFi.reconnect();
  }

  unsigned long now = micros();
  float dt = (now - last_micros) / 1.0e6f; // dt in seconds
  last_micros = now;

  sensor.update();
  pendulum.update();
  motor.loopFOC();

  float y1 = sensor.getAngle();
  float y2 = constrainAngle(pendulum.getAngle() + _PI);
  float y1_dot = LPF_Motor(-motor.shaft_velocity);
  float y2_dot = LPF_Pendulum(pendulum.getVelocity());
  float tau = 0.0f;
  float tau_raw = 0.0f;
  float energy = 0.0f;
  float sign = 0.0f;

  if (isMotorEnabled) {

    if (abs(motor.shaft_velocity) > 35) {
      isMotorEnabled = false;
      motor.disable();
      resetExperiment();
    }


    // CONDITION 1: Balance mode
    if (abs(y2) < 0.5f) {
      if (abs(y2) < 0.2f) {
        if (timeEnteredTightZone == 0) {
          timeEnteredTightZone = millis(); // Start timer
        } else if (millis() - timeEnteredTightZone > 1000) {
          balance = true;
        }
      } else {
        timeEnteredTightZone = 0;
        balance = false;
      }

      if (balance) {
        // ---- FINAL BALANCE CONTROLLER (with integral) ----
        sigma += dt * fmod(y1, _2PI);
        tau_raw = -k1 * fmod(y1, _2PI) + k2 * y2 - k3 * y1_dot + k4 * y2_dot - k5 * sigma;
        tau = constrain(tau_raw, -torque_max, torque_max);
        motor.move(-tau / torque_constant);
      } else {
        sigma = 0;
        tau_raw = -k1_1 * fmod(y1, _2PI) + k2_1 * y2 - k3_1 * y1_dot + k4_1 * y2_dot;
        tau = constrain(tau_raw, -torque_max, torque_max);
        motor.move(-tau / torque_constant);
      }

    } 
    // CONDITION 2: Pendulum is resting at the bottom
    else if (abs(y2_dot) < 0.2 || abs(y2) > (_PI - 0.2)) {
      motor.move(0);
      sigma = 0;
      balance = false;
      timeEnteredTightZone = 0;
    }
    // CONDITION 3: Pendulum has been pushed, start swinging
    else { 
      sigma = 0;
      balance = false;
      timeEnteredTightZone = 0;
      float energy = 0.5 * J2 * y2_dot * y2_dot + m * g * l * (cos(y2) - 1);
      float sign = (y2_dot * cos(y2) >= 0) ? -1.0f : 1.0f;
      tau_raw = 12 * energy * sign;
      tau = constrain(tau_raw, -torque_max, torque_max);
      motor.move(tau / torque_constant);
    }

  } else {
    resetExperiment();
    balance = false;
  }

  // if (count++ >= 500) {
  //     Serial.print(y1);
  //     Serial.print(", ");
  //     Serial.println(sigma);
  //     count = 0;
  // }

  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'q') {
      motor.disable();
      isMotorEnabled = false;
      Serial.println("Motor DISABLED. Type 's' to re-enable.");
    } 
    else if (command == 's') {
      motor.PID_current_q.reset();
      motor.PID_current_d.reset();
      motor.enable();
      isMotorEnabled = true;
      Serial.println("Motor ENABLED.");
    }
  }
}

