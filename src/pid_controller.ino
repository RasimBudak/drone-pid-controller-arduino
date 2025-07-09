// Basic PID controller for drone pitch axis
// Developed by Rasim Budak - 2025

float Kp = 1.2;
float Ki = 0.02;
float Kd = 0.8;

float setpoint = 0.0;         // Desired pitch angle
float input = 0.0;            // Current pitch angle (from IMU or simulated)
float output = 0.0;

float error, previous_error = 0;
float integral = 0;
float derivative = 0;

unsigned long lastTime = 0;
float dt;

void setup() {
  Serial.begin(9600);
}

void loop() {
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;  // convert to seconds
  lastTime = now;

  // Simulated IMU input (or replace with real sensor value)
  input = analogRead(A0) * (180.0 / 1023.0) - 90; // Map to -90 to +90 degrees

  error = setpoint - input;
  integral += error * dt;
  derivative = (error - previous_error) / dt;

  output = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;

  // Simulate motor response (or replace with real PWM output)
  analogWrite(9, constrain(map(output, -50, 50, 0, 255), 0, 255));

  // Debug output
  Serial.print("Input: "); Serial.print(input);
  Serial.print("  Output: "); Serial.println(output);

  delay(20);  // 50 Hz loop
}
