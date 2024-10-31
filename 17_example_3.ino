#include <Servo.h>

// Arduino pin assignment
#define PIN_IR    0         // IR sensor at Pin A0
#define PIN_LED   9
#define PIN_SERVO 10

#define _DUTY_MIN 500      // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1750     // servo neutral position (90 degree)
#define _DUTY_MAX 2750     // servo full counter-clockwise position (180 degree)

#define _DIST_MIN  100.0   // minimum distance 100mm (10cm)
#define _DIST_MAX  250.0   // maximum distance 250mm (25cm)

#define EMA_ALPHA  0.1     // EMA filter constant
#define LOOP_INTERVAL 20   // Loop Interval in milliseconds (20ms)

Servo myservo;
unsigned long last_loop_time = 0;  // Track the last loop time

float dist_prev = _DIST_MIN;
float dist_ema = _DIST_MIN;

void setup() {
  pinMode(PIN_LED, OUTPUT);  // Set LED pin as output

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);  // Set servo to neutral position

  Serial.begin(1000000);  // Initialize serial communication at 1,000,000 bps
}

void loop() {
  unsigned long time_curr = millis();
  int duty;
  float a_value, dist_raw;

  // Wait until the next loop interval
  if (time_curr < (last_loop_time + LOOP_INTERVAL)) return;
  last_loop_time += LOOP_INTERVAL;

  // Read the IR sensor value and calculate the raw distance
  a_value = analogRead(PIN_IR);
  dist_raw = ((6762.0 / (a_value - 9.0)) - 4.0) * 10.0 - 60.0;

  // Control LED based on distance range
  if (dist_raw <= _DIST_MAX) {
    digitalWrite(PIN_LED, HIGH);   // Turn on LED when distance is within range
  } else {
    digitalWrite(PIN_LED, LOW);    // Turn off LED when out of range
  }

  // Apply the EMA filter for stable distance reading
  dist_ema = EMA_ALPHA * dist_raw + (1 - EMA_ALPHA) * dist_prev;
  dist_prev = dist_ema;

  // Map the EMA distance to servo duty cycle without map() function
  duty = (int)((dist_ema - _DIST_MIN) / (_DIST_MAX - _DIST_MIN) * 
               (_DUTY_MAX - _DUTY_MIN) + _DUTY_MIN);

  // Move the servo to the calculated position
  myservo.writeMicroseconds(duty);

  // Print all relevant data for Serial Plotter
  Serial.print("IR:");        Serial.print(a_value);
  Serial.print(" dist_raw:"); Serial.print(dist_raw);   // Raw distance value for plot
  Serial.print(" ema:");      Serial.print(dist_ema);   // EMA filtered distance for plot
  Serial.print(" servo:");    Serial.print(duty);       // Servo position for plot
  Serial.println("");
}
