// Motor pins
const int leftMotorPin1 = 4;
const int leftMotorPin2 = 5;
const int rightMotorPin1 = 6;
const int rightMotorPin2 = 7;

void setup() {
  // Initialize the serial communication
  Serial.begin(115200);

  // Set the motor control pins as output
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
}

void loop() {
  // Read the serial data if available
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');

    // Parse the command and extract the motor speeds
    float leftSpeed, rightSpeed;
    sscanf(command.c_str(), "%f,%f", &leftSpeed, &rightSpeed);

    // Set the motor speeds based on the command
    setLeftMotorSpeed(leftSpeed);
    setRightMotorSpeed(rightSpeed);
  }
}

void setLeftMotorSpeed(float speed) {
  if (speed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  }

  //analogWrite(leftMotorPinPWM, abs(speed));
}

void setRightMotorSpeed(float speed) {
  if (speed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }

  //analogWrite(rightMotorPinPWM, abs(speed));
}
