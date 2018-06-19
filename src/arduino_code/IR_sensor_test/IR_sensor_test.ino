/*  This code is for quickly testing the sensors to make sure they all work properly
 *  before adding them to the sensor array on the robot.
 */

// Sensor reading value
int16_t value;

void setup() {
  Serial.begin(9600);
  Serial.println("Sensor recording...");
}

void loop() {
  value = analogRead(A0);
  Serial.println(value);
  // Print at 10Hz
  delay(100);
}
