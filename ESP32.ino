#include <SimpleFOC.h>
#include "BluetoothSerial.h"
#include <math.h>
#include <cmath>  // For fmod() and M_PI

BluetoothSerial SerialBT;
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(15, 33, 27, 4);

const int outputPin = 26;
const int freq = 978;
const int resolution = 10;

const int signalPin = 25;
volatile int pulseCount = 0;
unsigned long lastPulseTime = 0;

volatile float signalAngle = 0;
static String receivedText = "";

float target = 0;

int slide = 0;
int lastSlide = 0;

float numClutch = 4;
float stepSize = 6.28 / numClutch;
char control_mode = 'V';

unsigned long alttabtime = 0;
int alttab = 1;
int sendapp = 0;
float lastanlge = 0;

void doTarget(const char* cmd) {
  float value = atof(cmd + 2);

  switch (cmd[0]) {
    case 'A':
      motor.controller = MotionControlType::angle;
      if (control_mode != 'A') {
        SerialBT.println("Switched to angle control mode.");
        control_mode = 'A';
      }
      target = value;
      break;
    case 'S':
      motor.controller = MotionControlType::velocity;
      if (control_mode != 'S') {
        SerialBT.println("Switched to velocity control mode.");
        control_mode = 'S';
      }
      target = value;
      break;
    case 'V':
      motor.controller = MotionControlType::torque;
      if (control_mode != 'V') {
        SerialBT.println("Switched to torque (voltage) control mode.");
        control_mode = 'V';
      }
      target = value;
      break;
    case 'C':
      motor.controller = MotionControlType::angle;
      motor.P_angle.P = 20;
      if (control_mode != 'C') {
        SerialBT.println("Switched to clutch mode.");
        control_mode = 'C';
        target = round(sensor.getAngle() / 6.28) * 6.28;
      }
      break;
    default:
      SerialBT.println("Unknown command.");
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Smart Knob");
  //Serial.println("The device started, now you can pair it with Bluetooth!");

  pinMode(signalPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(signalPin), SignalCounter, CHANGE);

  ledcAttach(outputPin, freq, resolution);

  SimpleFOCDebug::enable(&Serial);
  sensor.init();
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 1.2f;
  motor.PID_velocity.D = 0;
  motor.voltage_limit = 6;
  motor.LPF_velocity.Tf = 0.005f;
  motor.P_angle.P = 18;
  motor.velocity_limit = 20;
  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();

  //Serial.println(F("Motor ready."));
  //Serial.println(F("Set the target and control mode using serial terminal:"));
  //Serial.println(F("A[value] for angle, S[value] for speed, V[value] for voltage."));
  _delay(1000);
}

void loop() {
  motor.loopFOC();
  motor.move(target);

  if (control_mode == 'C') {
    if (sensor.getAngle() > (target + stepSize / 2)) {
      target = target + stepSize;
    }
    if (sensor.getAngle() < (target - stepSize / 2)) {
      target = target - stepSize;
    }
  }
  if (pulseCount > 0 && (millis() - lastPulseTime > 188)) {  // 200 ms is the total expected time of the longest pattern
    if (pulseCount == 2) {
      SerialBT.println("Swipe Left");
      slide++;
    } else if (pulseCount == 3) {
      SerialBT.println("Swipe Right");
      slide--;
    } else if (pulseCount == 4) {
      SerialBT.println("Swipe Up");
    } else if (pulseCount == 5) {
      SerialBT.println("Swipe Down");
    }
    pulseCount = 0;  // Reset pulse count for the next pattern
  }

  if (slide == 0) {
    doTarget("V 0");
    lastSlide = 0;
  }

  if (slide == 1) {
    doTarget("A 0");
    lastSlide = 1;
  }

  if (slide == 2) {
    if (sensor.getAngle() > 0 && sensor.getAngle() < 1) {
      doTarget("A 0.525");
    }
    if (sensor.getAngle() < 0 && sensor.getAngle() > -1) {
      doTarget("A -0.525");
    }
    lastSlide = 2;
  }

  if (slide == 3) {
    //motor.P_angle.P = 5;
    if (sensor.getAngle() > 1.57) {
      doTarget("V -1");
    }
    else if (sensor.getAngle() < -1.57) {
      doTarget("V 1");
    }
    else {
      doTarget("V 0");
    }
    lastSlide = 3;
    }

  if (slide == 4) {
    if (lastSlide != 4){
      doTarget("C");
    }
    lastSlide = 4;
  }

  if (slide == 5) {
    if (sensor.getVelocity() > 4){
      doTarget("V 1.5");
    }
    else if (sensor.getVelocity() < -4){
      doTarget("V -1.5");
    }
    else if (abs(sensor.getVelocity()) < 3){
      doTarget("V 0");
    }
    lastSlide = 5;
  }
  

  if (slide == 6) {
    if (sensor.getAngle() > 6.28) {
      doTarget("V -1");
    }
    else if (sensor.getAngle() < 0) {
      doTarget("V 1");
    }
    else {
      doTarget("V 0");
    }
    int percent = int(sensor.getAngle()/6.28*100);
    if (percent != lastanlge) {
      SerialBT.println("Volume"+String(percent));
    }
    lastanlge = percent;
    lastSlide = 6;
  }

  if (slide == 7) {
    doTarget("A 0");
    float angle = sensor.getSensorAngle();  // Store the sensor angle in a variable

    if (angle >= 0.45 && angle <= 0.47 && millis() - alttabtime > 500) {
        alttab = 1;
        alttabtime = millis();
        SerialBT.println("AltTab");
    } 
    else if (angle >= 6 && angle <= 6.28) {
        alttab = 0;
    }
    
    lastSlide = 7;
  }

  if (slide == 8) {
    if (sensor.getVelocity() > 4){
      doTarget("V 1.5");
      if (sendapp == 0){
        SerialBT.println("Illustrator");
        sendapp = 1;
      }
    }
    else if (sensor.getVelocity() < -4){
      doTarget("V -1.5");
      sendapp = 0;
    }
    else if (abs(sensor.getVelocity()) < 3){
      doTarget("V 0");
    }
    lastSlide = 8;
  }

  if (slide == 9) {
    slide = 0;
    lastSlide = 9;
  }


  readBluetooth();
  updateAngle();
  Serial.println(sensor.getAngle());
}

void readBluetooth() {
  if (SerialBT.available()) {
    char inChar = (char)SerialBT.read();
    //Serial.write(inChar);

    if (inChar == '\n' || inChar == '\r') {
      if (receivedText.length() > 0) {
        SerialBT.println("Received: " + receivedText);
        doTarget(receivedText.c_str());  // Call doTarget with the correct C string
        receivedText = "";
      }
    } else {
      receivedText += inChar;
    }
  }
}

void updateAngle() {
  signalAngle = fmod(sensor.getAngle(), 6.28);
  if (signalAngle < 0) {
    signalAngle += 6.28;
  }
  volatile int output = int((signalAngle / 6.28 * 360) + 5);
  //Serial.println(output);
  ledcWrite(outputPin, output);
}

void SignalCounter() {
  pulseCount++;
  //Serial.println(pulseCount);
  //SerialBT.println(pulseCount);
  lastPulseTime = millis();
}
