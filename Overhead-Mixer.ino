#include "Adafruit_LEDBackpack.h"

#define HALL_PIN 26
#define MOTOR_PIN 25
#define POT_PIN 34

const int TICS_PER_REV = 2;

const int PWM_FREQ = 500;
const int MIN_MICROS = 1200;
const int NEUTRAL_MICROS = 1500;
const int MAX_MICROS = 1800;

const double DUTYCYCLE_PER_MICROS = 255 * PWM_FREQ / 1E6;
const double MAX_RPM = 2300;
const double kV = (MAX_MICROS - NEUTRAL_MICROS) / MAX_RPM;

const double kP = 0;
const double kD = 0;
const double kI = 0.04;

double integral = 0;
const double INTEGRAL_MAX = 1000;

double prev_error = 0;

volatile long prev_tick_micros = 0;
volatile long tick_dt = 1;
volatile long prev_tick_dt = 1;

long prev_micros = 0;

const byte SMOOTH = 20;
int prev_desired_RPMs[SMOOTH];

Adafruit_7segment matrix = Adafruit_7segment();

double micros_to_rpm(int micros) {
  return 1 / (micros*1E-6) * 60 / TICS_PER_REV;
}

int avg(int* arr, int len) {
  int sum = 0;
  for (int i = 0; i < len; i++) {
    sum += arr[i];
  }
  return sum / len;
}

int PID_result(double current, double desired, double dt) {
  double error = desired - current;

  double derivative = (prev_error - error) / dt;
  prev_error = error;

  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  
  return kP*error + kD*derivative + kI*integral;
}

void IRAM_ATTR update_encoder_ticks() {
  if (micros() - prev_tick_micros < 10000) return;
  prev_tick_dt = tick_dt;
  tick_dt = micros() - prev_tick_micros;
  prev_tick_micros = micros();
}

void setup() {
  Serial.begin(115200);
  
  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);

  attachInterrupt(HALL_PIN, update_encoder_ticks, RISING);
  ledcAttach(MOTOR_PIN, PWM_FREQ, 8);
  prev_micros = micros();

  matrix.begin();

  int desired_RPM = map(analogRead(POT_PIN), 100, 4095, 0, MAX_RPM);
  desired_RPM = constrain(desired_RPM, 0, MAX_RPM);
  for (int i = 0; i < SMOOTH; i++) {
    prev_desired_RPMs[i] = desired_RPM;
  }
}

void loop() {
  double dt = (micros() - prev_micros) * 1E-6;
  prev_micros = micros();

  double rpm = (micros_to_rpm(tick_dt) + micros_to_rpm(prev_tick_dt)) / 2;

  int desired_RPM = map(analogRead(POT_PIN), 100, 4095, 0, MAX_RPM);
  desired_RPM = constrain(desired_RPM, 0, MAX_RPM);

  int micros = map(desired_RPM, 0, MAX_RPM, NEUTRAL_MICROS, MAX_MICROS);
  if (desired_RPM > 60) {
    micros += PID_result(rpm, desired_RPM, dt);
    micros = constrain(micros, NEUTRAL_MICROS, MAX_MICROS);
  }
  else {
    rpm = 0;
  }
  Serial.print(rpm);
  
  int dutycycle = DUTYCYCLE_PER_MICROS * micros;
  Serial.print("\t"); Serial.print(desired_RPM);
  Serial.print("\t"); Serial.println(digitalRead(HALL_PIN));
  ledcWrite(MOTOR_PIN, dutycycle);

  for (int i = SMOOTH-1; i > 0; i--) {
    prev_desired_RPMs[i] = prev_desired_RPMs[i-1];
  }
  prev_desired_RPMs[0] = desired_RPM;

  matrix.print((int) round( avg(prev_desired_RPMs, SMOOTH) / 10.0 ) * 10);
  matrix.writeDisplay();
  
  delay(20);
}
