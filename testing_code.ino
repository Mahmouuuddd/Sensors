#include <NewPing.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#define TRIGGER_PIN_1 9
#define ECHO_PIN_1 8
#define TRIGGER_PIN_2 11
#define ECHO_PIN_2 10
#define TRIGGER_PIN_3 13
#define ECHO_PIN_3 12

#define MAX_DISTANCE 200

NewPing sensor_1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sensor_2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
NewPing sensor_3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE);

// Motor pins
const int leftMotorPin1 = 4;
const int leftMotorPin2 = 5;
const int rightMotorPin1 = 6;
const int rightMotorPin2 = 7;


SoftwareSerial ss(2, 3); // RX, TX
TinyGPSPlus gps;
Adafruit_HMC5883_Unified compass;

ros::NodeHandle nh;

std_msgs::Float32 ultrasonic_data_msg;
std_msgs::Float32 compass_data_msg;
std_msgs::String gps_data_msg;
geometry_msgs::Twist movement_cmd_msg;

ros::Publisher ultrasonic_pub("/ultrasonic_data", &ultrasonic_data_msg);
ros::Publisher compass_pub("/compass_data", &compass_data_msg);
ros::Publisher gps_pub("/gps_data", &gps_data_msg);
ros::Subscriber<geometry_msgs::Twist> movement_cmd_sub("/cmd_vel", movementCmdCallback);


void setup() {
  nh.initNode();
  nh.advertise(ultrasonic_pub);
  nh.advertise(compass_pub);
  nh.advertise(gps_pub);
  nh.subscribe(movement_cmd_sub);

  Wire.begin();
  ss.begin(9600);
  mag.begin();

  // Initialize serial communication
  Serial.begin(115200);
}

void loop() {
  // Read ultrasonic sensor data
  unsigned int distance_1 = sensor_1.ping_cm();
  unsigned int distance_2 = sensor_2.ping_cm();
  unsigned int distance_3 = sensor_3.ping_cm();

  // Publish ultrasonic data
  ultrasonic_data_msg.data = String(distance_1, 3) + "," + String(distance_2, 3) + "," + String(distance_3, 3);;
  ultrasonic_pub.publish(&ultrasonic_data_msg);

  // Read compass data
  sensors_event_t mag_data;
  mag.getEvent(&mag_data);

  float heading = atan2(mag_data.magnetic.y, mag_data.magnetic.x);
  if (heading < 0) {
    heading += 2 * PI;
  }
  float heading_degrees = heading * 180 / PI;
  // Publish compass data
  compass_data_msg.data = heading_degrees;
  compass_pub.publish(&compass_data_msg);

  // Read GPS data
  while (ss.available() > 0)
    if (gps.encode(ss.read())){
      float gpsLatitude = gps.location.lat();
      float gpsLongitude = gps.location.lng();
    }
  gps_data_msg.latitude = gpsLatitude;  // Replace with actual GPS latitude value
  gps_data_msg.longitude = gpsLongitude;  // Replace with actual GPS longitude value
  gps_pub.publish(&gps_data_msg);
  

  nh.spinOnce();

  // Handle movement commands
  moveRobot(movement_cmd_msg);

  delay(100);
}


void movementCmdCallback(const geometry_msgs::Twist& cmd) {
  // Store the incoming movement command
  movement_cmd_msg = cmd;
}

void moveRobot(const geometry_msgs::Twist& cmd) {
  // Perform appropriate actions based on the movement command

  // Extract linear and angular velocities from the command
  float linear_velocity = cmd.linear.x;
  float angular_velocity = cmd.angular.z;

  // Adjust these values based on your hardware and desired behavior
  float max_linear_velocity = 1.0;   // Maximum linear velocity (adjust as needed)
  float max_angular_velocity = 1.0;  // Maximum angular velocity (adjust as needed)

  // Adjust these values based on your motor controller commands
  byte left_motor_forward = 4;
  byte left_motor_backward = 5;
  byte right_motor_forward = 6;
  byte right_motor_backward = 7;

  // Calculate motor speeds based on the linear and angular velocities
  float left_motor_speed = linear_velocity - angular_velocity;
  float right_motor_speed = linear_velocity + angular_velocity;

  // Limit the motor speeds to the maximum values
  left_motor_speed = constrain(left_motor_speed, -max_linear_velocity, max_linear_velocity);
  right_motor_speed = constrain(right_motor_speed, -max_linear_velocity, max_linear_velocity);

  // Control the left motor
  if (left_motor_speed > 0) {
    digitalWrite(left_motor_forward, HIGH);
    digitalWrite(left_motor_backward, LOW);
  } else {
    digitalWrite(left_motor_forward, LOW);
    digitalWrite(left_motor_backward, HIGH);
  }
  //analogWrite(left_motor_speed_pin, abs(left_motor_speed));

  // Control the right motor
  if (right_motor_speed > 0) {
    digitalWrite(right_motor_forward, HIGH);
    digitalWrite(right_motor_backward, LOW);
  } else {
    digitalWrite(right_motor_forward, LOW);
    digitalWrite(right_motor_backward, HIGH);
  }
  //analogWrite(right_motor_speed_pin, abs(right_motor_speed));
}
