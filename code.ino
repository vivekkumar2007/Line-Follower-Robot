// ==================== IR Sensor Pins ====================
const int irLeft = A4;     // Left IR sensor connected to analog pin A4
const int irCenter = A3;   // Center IR sensor connected to analog pin A3
const int irRight = A5;    // Right IR sensor connected to analog pin A5

// ==================== Motor Control Pins ====================
const int leftMotorPWM = 6;     // PWM pin to control left motor speed
const int leftMotorIN1 = 9;     // IN1 for left motor direction
const int leftMotorIN2 = 10;    // IN2 for left motor direction
const int rightMotorPWM = 5;    // PWM pin to control right motor speed
const int rightMotorIN1 = 7;    // IN3 for right motor direction
const int rightMotorIN2 = 8;    // IN4 for right motor direction

// ==================== PID Constants ====================
float kp = 120;   // Proportional gain
float ki = 0.0;   // Integral gain (set to 0 for simplicity)
float kd = 70;    // Derivative gain

// ==================== PID Variables ====================
float currentError = 0;    // Real-time error based on sensor reading
float lastError = 0;       // Stores the previous error for derivative calculation
float errorSum = 0;        // Accumulates error over time (used for integral)
float errorChange = 0;     // Difference between current and last error
float pidOutput = 0;       // Final PID output used to adjust motor speed

// ==================== Motor Base Speed ====================
int defaultSpeed = 190;    // Base speed of motors (range: 0 to 255)

// ==================== IR Sensor Threshold ====================
int blackWhiteThreshold = 500;  // Analog value threshold to detect black vs white

// ==================== Arduino Setup ====================
void setup() {
  // Configure sensor pins as input
  pinMode(irLeft, INPUT);
  pinMode(irCenter, INPUT);
  pinMode(irRight, INPUT);

  // Configure motor control pins as output
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorIN1, OUTPUT);
  pinMode(leftMotorIN2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorIN1, OUTPUT);
  pinMode(rightMotorIN2, OUTPUT);

  // Start serial communication for debugging
  Serial.begin(9600);
}

// ==================== Main Loop ====================
void loop() {
  // Read analog values from IR sensors
  int leftSensor = analogRead(irLeft);
  int centerSensor = analogRead(irCenter);
  int rightSensor = analogRead(irRight);

  // Convert analog readings into binary: 1 = black, 0 = white
  int leftDetect = (leftSensor > blackWhiteThreshold) ? 1 : 0;
  int centerDetect = (centerSensor > blackWhiteThreshold) ? 1 : 0;
  int rightDetect = (rightSensor > blackWhiteThreshold) ? 1 : 0;

  // Determine error based on sensor states
  if (leftDetect == 1 && centerDetect == 0 && rightDetect == 0) currentError = -2;
  else if (leftDetect == 1 && centerDetect == 1 && rightDetect == 0) currentError = -1;
  else if (leftDetect == 0 && centerDetect == 1 && rightDetect == 0) currentError = 0;
  else if (leftDetect == 0 && centerDetect == 1 && rightDetect == 1) currentError = 1;
  else if (leftDetect == 0 && centerDetect == 0 && rightDetect == 1) currentError = 2;
  else if (leftDetect == 1 && centerDetect == 1 && rightDetect == 1) currentError = 0; // All sensors on line. Here you can stop your robot
  else currentError = lastError; // If all are off line, maintain last direction

  // ========== PID Calculation ==========
  errorSum += currentError;  // For integral term (not used here as ki = 0)
  errorChange = currentError - lastError;  // For derivative term
  pidOutput = kp * currentError + ki * errorSum + kd * errorChange; // Final PID output
  lastError = currentError;  // Update last error for next loop

  // ========== Calculate Motor Speeds ==========
  int leftSpeed = defaultSpeed - pidOutput;   // Left motor slows down on turning right
  int rightSpeed = defaultSpeed + pidOutput;  // Right motor slows down on turning left

  // Limit speeds to range [0, 255]
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Move the motors with calculated speeds
  moveMotors(leftSpeed, rightSpeed);

  delay(5);  // Small delay for stability
}

// ==================== Motor Control Function ====================
void moveMotors(int leftSpeed, int rightSpeed) {
  // Set direction of both motors (forward)
  digitalWrite(leftMotorIN1, HIGH);
  digitalWrite(leftMotorIN2, LOW);
  digitalWrite(rightMotorIN1, HIGH);
  digitalWrite(rightMotorIN2, LOW);

  // Set speed of both motors using PWM
  analogWrite(leftMotorPWM, leftSpeed);
  analogWrite(rightMotorPWM, rightSpeed);
}
