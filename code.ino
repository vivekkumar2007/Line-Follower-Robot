// IR sensor pins
const int irLeft = A4;
const int irCenter = A3;
const int irRight = A5;

// Motor control pins
const int leftMotorPWM = 6;
const int leftMotorIN1 = 9;
const int leftMotorIN2 = 10;
const int rightMotorPWM = 5;
const int rightMotorIN1 = 7;
const int rightMotorIN2 = 8;

// PID constants
float kp = 120;
float ki = 0.0;
float kd = 70;

// PID variables
float currentError = 0;
float lastError = 0;
float errorSum = 0;
float errorChange = 0;
float pidOutput = 0;

// Motor base speed
int defaultSpeed = 190;  // Range: 0 to 255

// Sensor threshold
int blackWhiteThreshold = 500;

void setup() {
  pinMode(irLeft, INPUT);
  pinMode(irCenter, INPUT);
  pinMode(irRight, INPUT);

  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorIN1, OUTPUT);
  pinMode(leftMotorIN2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorIN1, OUTPUT);
  pinMode(rightMotorIN2, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  int leftSensor = analogRead(irLeft);
  int centerSensor = analogRead(irCenter);
  int rightSensor = analogRead(irRight);

  // Determine black or white
  int leftDetect = (leftSensor > blackWhiteThreshold) ? 1 : 0;
  int centerDetect = (centerSensor > blackWhiteThreshold) ? 1 : 0;
  int rightDetect = (rightSensor > blackWhiteThreshold) ? 1 : 0;

  // Calculate error based on sensor state
  if (leftDetect == 1 && centerDetect == 0 && rightDetect == 0) currentError = -2;
  else if (leftDetect == 1 && centerDetect == 1 && rightDetect == 0) currentError = -1;
  else if (leftDetect == 0 && centerDetect == 1 && rightDetect == 0) currentError = 0;
  else if (leftDetect == 0 && centerDetect == 1 && rightDetect == 1) currentError = 1;
  else if (leftDetect == 0 && centerDetect == 0 && rightDetect == 1) currentError = 2;
  else if (leftDetect == 1 && centerDetect == 1 && rightDetect == 1) currentError = 0;
  else currentError = lastError; // Fallback if line is lost

  // PID calculations
  errorSum += currentError;
  errorChange = currentError - lastError;
  pidOutput = kp * currentError + ki * errorSum + kd * errorChange;
  lastError = currentError;

  // Speed adjustment
  int leftSpeed = defaultSpeed - pidOutput ;
  int rightSpeed = defaultSpeed + pidOutput ;

  // Limit speed range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  moveMotors(leftSpeed, rightSpeed);
  delay(5);
}

void moveMotors(int leftSpeed, int rightSpeed) {
  // Move both motors forward
  digitalWrite(leftMotorIN1, HIGH);
  digitalWrite(leftMotorIN2, LOW);
  digitalWrite(rightMotorIN1, HIGH);
  digitalWrite(rightMotorIN2, LOW);

  analogWrite(leftMotorPWM, leftSpeed);
  analogWrite(rightMotorPWM, rightSpeed);
}
