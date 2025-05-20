//#####################################################################

#include <PID_v1.h>
#include <driver/ledc.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//#####################################################################
Adafruit_PWMServoDriver pwmm = Adafruit_PWMServoDriver();

const int inputParams[][5] = {
  { 0, 60, 83, 160, 90 }, { 0, 80, 83, 170, 90 }, 
  { 0, 80, 83, 170, 90 }, { 0, 80, 83, 160, 70 }, 
  { 0, 80, 83, 170, 75 }, { 0, 80, 83, 170, 125 }, 
  { 0, 80, 95, 160, 180 }, { 0, 70, 83, 170, 180 }, 
  { 0, 75, 83, 165, 180 }, { 0, 80, 95, 160, 0 }, 
  { 0, 80, 95, 180, 0 }, { 0, 55, 90, 180, 0 }, 
  { -50, 110, 150, 100, 0 }, { -50, 110, 160, 120, 0 }, 
  { -50, 110, 155, 140, 0 }, { -50, 75, 155, 90, 0 }, 
  { 0, 75, 90, 90, 0 }, { 0, 40, 180, 180, 90 }, 
  { 0, 20, 90, 90, 90 }, { 0, 10, 180, 180, 90 }, 
  { 0, 0, 180, 180, 90 }
};
int currentParamSet = 0;
int paramSetCount = sizeof(inputParams) / sizeof(inputParams[0]);

unsigned long cycleStartTime;
const unsigned long cycleDelay = 2000;  // 2 sec between cycles
bool isRunning = false;                 // Wait for serial input to start

enum ProgramState {
  WAITING_FOR_START,
  RUNNING,
  COMPLETED
};
ProgramState programState = WAITING_FOR_START;


//#####################################################################


//#####################################################################

// Servo settings
#define SERVO_MIN 125  // Minimum pulse length
#define SERVO_MAX 575  // Maximum pulse length
#define SERVO_FREQ 50  // 50Hz for analog servos
#define NUM_SERVOS 3
volatile int servo1_target = 90;
volatile int servo2_target = 90;
volatile int servo3_target = 90;
// Current servo positions (for smooth movement)
int servo1_current = 90;
int servo2_current = 90;
int servo3_current = 90;
const int SERVO_SPEED = 2;

//#####################################################################

// Last update time
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 20;  // ms

// I2C pins for ESP
#define I2C_SDA 21  // ESP32 default
#define I2C_SCL 22  // ESP32 default

//#####################################################################

// Motor 1 Configuration
#define Mot1Enable 5
#define Mot1Fwd 4
#define Mot1Rev 18
#define encoder1Pin1 35
#define encoder1Pin2 23
const int PPR1 = 4000;
double kp1 = 2.5, ki1 = 0.1, kd1 = 0.06;

// Motor 2 Configuration
#define Mot2Enable 25
#define Mot2Fwd 19
#define Mot2Rev 33
#define encoder2Pin1 2
#define encoder2Pin2 15
const int PPR2 = 1880;
double kp2 = 1.8, ki2 = 0.1, kd2 = 0.07;

//#####################################################################

// Encoder Variables
volatile long encoder1Value = 0;
volatile long encoder2Value = 0;
volatile int lastEncoded1 = 0;
volatile int lastEncoded2 = 0;

//#####################################################################

// PID Controllers
double input1 = 0, output1 = 0, setpoint1 = 0;
PID pid1(&input1, &output1, &setpoint1, kp1, ki1, kd1, DIRECT);

double input2 = 0, output2 = 0, setpoint2 = 0;
PID pid2(&input2, &output2, &setpoint2, kp2, ki2, kd2, DIRECT);

//#####################################################################

// System Variables
String readString;
unsigned long lastPlotTime = 0;
const unsigned long plotInterval = 50;
bool newData = false;

// PWM Setup
const int pwmChannel1 = LEDC_CHANNEL_0;
const int pwmChannel2 = LEDC_CHANNEL_1;
const int pwmFreq = 5000;
const int pwmResolution = 8;

//#####################################################################

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;  // Wait for serial connection

  Wire.begin(I2C_SDA, I2C_SCL);  // Explicitly set pins

  if (!pwmm.begin()) {
    Serial.println("PCA9685 not found! Check wiring.");
    while (1)
      ;  // Halt if I2C fails
  }
  Serial.println("PCA9685 initialized!");
  pwmm.setPWMFreq(SERVO_FREQ);

  // Initialize servos to center position
  pwmm.setPWM(0, 0, angleToPulse(90));
  pwmm.setPWM(1, 0, angleToPulse(90));
  pwmm.setPWM(2, 0, angleToPulse(90));

  // Motor 1 Setup
  pinMode(Mot1Fwd, OUTPUT);
  pinMode(Mot1Rev, OUTPUT);
  ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
  ledcAttachPin(Mot1Enable, pwmChannel1);

  // Motor 2 Setup
  pinMode(Mot2Fwd, OUTPUT);
  pinMode(Mot2Rev, OUTPUT);
  ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
  ledcAttachPin(Mot2Enable, pwmChannel2);

  // Encoder Setup
  pinMode(encoder1Pin1, INPUT_PULLUP);
  pinMode(encoder1Pin2, INPUT_PULLUP);
  pinMode(encoder2Pin1, INPUT_PULLUP);
  pinMode(encoder2Pin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1Pin1), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1Pin2), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2Pin1), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2Pin2), updateEncoder2, CHANGE);

  // PID Setup
  pid1.SetMode(AUTOMATIC);
  pid1.SetSampleTime(10);
  pid1.SetOutputLimits(-255, 255);

  pid2.SetMode(AUTOMATIC);
  pid2.SetSampleTime(10);
  pid2.SetOutputLimits(-255, 255);
}

void loop() {

  switch (programState) {
    case WAITING_FOR_START:
      if (Serial.available() > 0) {
        Serial.read();  // Clear serial buffer
        Serial.println("Starting program...");
        currentParamSet = 0;  // Reset to first parameter set
        programState = RUNNING;
        cycleStartTime = millis();
      }
      break;

    case RUNNING:
      // Run all 3 functions sequentially WITHOUT delays
      processInput(
        inputParams[currentParamSet][0],
        inputParams[currentParamSet][1],
        inputParams[currentParamSet][2],
        inputParams[currentParamSet][3],
        inputParams[currentParamSet][4]);
      controlMotors();
      plotMotorData();

      // Move to next parameter set after delay
      if (millis() - cycleStartTime >= cycleDelay) {
        currentParamSet++;

        if (currentParamSet >= paramSetCount) {
          Serial.println("Program completed. Send any character to restart.");
          programState = COMPLETED;
        } else {
          cycleStartTime = millis();  // Reset timer for next cycle
        }
      }
      break;

    case COMPLETED:
      if (Serial.available() > 0) {
        Serial.read();                     // Clear serial buffer
        programState = WAITING_FOR_START;  // Will restart on next loop
      }
      break;
  }
}

void processInput(int a, int b, int x, int d, int e) {
  int parts[5] = { a, b, x, d, e };
  // Set motor targets
  setpoint1 = map(parts[0], 0, 360, 0, PPR1);
  setpoint2 = map(parts[1], 0, 360, 0, PPR2);
  servo1_target = constrain(parts[2], 0, 180);
  servo2_target = constrain(parts[3], 0, 180);
  servo3_target = constrain(parts[4], 0, 180);

  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentMillis;

    // Smooth movement for Servo 1
    if (servo1_current != servo1_target) {
      servo1_current += (servo1_target > servo1_current) ? SERVO_SPEED : -SERVO_SPEED;
      servo1_current = constrain(servo1_current, 0, 180);
      pwmm.setPWM(0, 0, angleToPulse(servo1_current));
    }

    // Smooth movement for Servo 2
    if (servo2_current != servo2_target) {
      servo2_current += (servo2_target > servo2_current) ? SERVO_SPEED : -SERVO_SPEED;
      servo2_current = constrain(servo2_current, 0, 180);
      pwmm.setPWM(1, 0, angleToPulse(servo2_current));
    }

    // Smooth movement for Servo 3
    if (servo3_current != servo3_target) {
      servo3_current += (servo3_target > servo3_current) ? SERVO_SPEED : -SERVO_SPEED;
      servo3_current = constrain(servo3_current, 0, 180);
      pwmm.setPWM(2, 0, angleToPulse(servo3_current));
    }
  }

  Serial.print("New Targets - M1:");
  Serial.print(parts[0]);
  Serial.print("° M2:");
  Serial.print(parts[1]);
  Serial.print("° S1:");
  Serial.print(parts[2]);
  Serial.print("° S2:");
  Serial.print(parts[3]);
  Serial.print(" S3:");
  Serial.print(parts[4]);
  Serial.println("°");

  readString = "";
}
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void controlMotors() {
  // Motor 1 Control
  input1 = encoder1Value;
  pid1.Compute();
  motorOut(Mot1Enable, pwmChannel1, Mot1Fwd, Mot1Rev, output1);

  // Motor 2 Control
  input2 = encoder2Value;
  pid2.Compute();
  motorOut(Mot2Enable, pwmChannel2, Mot2Fwd, Mot2Rev, output2);
}

void motorOut(int enablePin, int channel, int fwdPin, int revPin, int out) {
  int pwmValue = constrain(abs(out), 0, 255);
  ledcWrite(channel, pwmValue);

  if (out > 5) {
    digitalWrite(fwdPin, HIGH);
    digitalWrite(revPin, LOW);
  } else if (out < -5) {
    digitalWrite(fwdPin, LOW);
    digitalWrite(revPin, HIGH);
  } else {
    digitalWrite(fwdPin, LOW);
    digitalWrite(revPin, LOW);
    ledcWrite(channel, 0);
  }
}

void plotMotorData() {
  if (millis() - lastPlotTime >= plotInterval) {
    lastPlotTime = millis();

    Serial.print("M1_Cur:");
    Serial.print((float)encoder1Value / PPR1 * 360.0);
    Serial.print("° M1_Tar:");
    Serial.print((float)setpoint1 / PPR1 * 360.0);
    Serial.print("° M2_Cur:");
    Serial.print((float)encoder2Value / PPR2 * 360.0);
    Serial.print("° M2_Tar:");
    Serial.print((float)setpoint2 / PPR2 * 360.0);
    Serial.print("° M1_Out:");
    Serial.print(output1);
    Serial.print(" M2_Out:");
    Serial.println(output2);
  }
}

void updateEncoder1() {
  int MSB = digitalRead(encoder1Pin1);
  int LSB = digitalRead(encoder1Pin2);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded1 << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder1Value++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder1Value--;
  lastEncoded1 = encoded;
}

void updateEncoder2() {
  int MSB = digitalRead(encoder2Pin1);
  int LSB = digitalRead(encoder2Pin2);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded2 << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder2Value++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder2Value--;
  lastEncoded2 = encoded;
}
