#include <Servo.h>

// Arduino pin assignment (based on 11_example_1.ino)
#define PIN_LED   9    // LED active-low
#define PIN_TRIG  12   // sonar TRIGGER
#define PIN_ECHO  13   // sonar ECHO
#define PIN_SERVO 10   // servo signal

// Sonar configuration
#define SND_VEL 346.0
#define INTERVAL 25
#define PULSE_DURATION 10
#define _DIST_MIN 180.0  // mm = 18 cm
#define _DIST_MAX 360.0  // mm = 36 cm
#define TIMEOUT ((INTERVAL / 2) * 1000.0)
#define SCALE (0.001 * 0.5 * SND_VEL)

// Filters: median suppresses isolated spikes; EMA smooths motion
#define MEDIAN_SIZE 5
#define _EMA_ALPHA 0.3

// Tune pulse widths to the actual servo before recording the demo
#define _DUTY_MIN 1000
#define _DUTY_MAX 2000
#define _DUTY_NEU ((_DUTY_MIN + _DUTY_MAX) / 2)

Servo myservo;
float dist_ema = (_DIST_MIN + _DIST_MAX) / 2;
float dist_raw = 0;
float dist_samples[MEDIAN_SIZE];
int sample_count = 0;
int sample_index = 0;
bool ema_ready = false;
unsigned long last_sampling_time = 0;

float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, (unsigned long)TIMEOUT) * SCALE;
}

float medianFilter(float value) {
  dist_samples[sample_index] = value;
  sample_index = (sample_index + 1) % MEDIAN_SIZE;
  if (sample_count < MEDIAN_SIZE) sample_count++;

  float sorted[MEDIAN_SIZE];
  for (int i = 0; i < sample_count; i++) sorted[i] = dist_samples[i];
  for (int i = 0; i < sample_count - 1; i++) {
    for (int j = i + 1; j < sample_count; j++) {
      if (sorted[i] > sorted[j]) {
        float t = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = t;
      }
    }
  }
  return sorted[sample_count / 2];
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_LED, HIGH);  // active-low: OFF
  digitalWrite(PIN_TRIG, LOW);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);
  Serial.begin(57600);
  last_sampling_time = millis();
}

void loop() {
  if (millis() - last_sampling_time < INTERVAL) return;
  last_sampling_time += INTERVAL;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);
  bool echo_ok = (dist_raw > 0.0);
  bool in_range = echo_ok && dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX;

  // Active-low LED: ON only for valid measurements in 18..36 cm
  digitalWrite(PIN_LED, in_range ? LOW : HIGH);

  if (echo_ok) {
    // Range filter: actual distances below/above range map to endpoints.
    // A zero pulseIn result means no echo, not a distance of zero.
    float dist_filtered = constrain(dist_raw, _DIST_MIN, _DIST_MAX);

    // Median filter: suppress one-off jumps before EMA.
    float dist_median = medianFilter(dist_filtered);

    // EMA: smooth the remaining variations.
    if (!ema_ready) {
      dist_ema = dist_median;
      ema_ready = true;
    } else {
      dist_ema = _EMA_ALPHA * dist_median
               + (1.0 - _EMA_ALPHA) * dist_ema;
    }

    float angle = (dist_ema - _DIST_MIN) * 180.0 / (_DIST_MAX - _DIST_MIN);
    angle = constrain(angle, 0.0, 180.0);
    int duty = _DUTY_MIN + (int)((_DUTY_MAX - _DUTY_MIN) * angle / 180.0 + 0.5);
    myservo.writeMicroseconds(duty);
  }
  // Timeout: hold the last servo position and EMA value.

  Serial.print("Min:");   Serial.print(_DIST_MIN);
  Serial.print(",dist:"); Serial.print(dist_raw);
  Serial.print(",ema:");  Serial.print(dist_ema);
  Serial.print(",Servo:");Serial.print(myservo.read());
  Serial.print(",Max:");  Serial.print(_DIST_MAX);
  Serial.println();
}
