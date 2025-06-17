#include <NewPing.h>
#include <SoftwareSerial.h>

// Adjust the pin connections based on your setup
#define TRIGGER_PIN_1 2
#define ECHO_PIN_1 3
#define TRIGGER_PIN_2 4
#define ECHO_PIN_2 5
#define TRIGGER_PIN_3 6
#define ECHO_PIN_3 7

// Define the maximum distance (in centimeters) for obstacle detection
#define MAX_DISTANCE 200

// Define the bau
d rate for communication with ROS node
#define BAUD_RATE 115200

NewPing sonar_1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar_2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
NewPing sonar_3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE);

SoftwareSerial serial(10, 11);  // Adjust the RX and TX pin numbers based on your setup

void setup() {
  Serial.begin(BAUD_RATE);
  serial.begin(BAUD_RATE);
}

void loop() {
  unsigned int distance_1 = sonar_1.ping_cm();
  unsigned int distance_2 = sonar_2.ping_cm();
  unsigned int distance_3 = sonar_3.ping_cm();

  // Send the sensor readings to the ROS node
  serial.print("ULTRASONIC:");
  serial.print(distance_1);
  serial.print(",");
  serial.print(distance_2);
  serial.print(",");
  serial.println(distance_3);

  delay(100);  // Adjust the delay as needed
}