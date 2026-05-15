#include <Pixy2I2C.h>
#include <Wire.h>

// -------------------- PIXY --------------------
Pixy2I2C pixy;

// -------------------- GYRO --------------------
#define ITG3205_ADDR 0x68

// -------------------- MOTOR PINS --------------------
#define ENA 5
#define ENB 6
#define IN1 A1
#define IN2 A0
#define IN3 A2
#define IN4 A3

// -------------------- PID CONSTANTS (TUNED FOR MAX SPEED) --------------------
float Kp = 6.0;   // Increased to react fast enough at maximum speed
float Kd = 3.8;   // Increased to prevent extreme high-speed weaving
float Ky = 0.06;  // Increased to lock straight lines during full throttle

// ===================== SETTINGS ===================== 
const int frameCenter = 39; 
const int laneShift   = 20; 

// -------------------- MAXIMUM SPEED CONFIGURATION --------------------
int maxStraightSpeed = 255; // ABSOLUTE LIMIT: 100% PWM Full Power  PWM (Pulse Width Modulation)
int minCornerSpeed   = 80; // High speed cornering floor to maintain momentum

// -------------------- STATE & MEMORY --------------------
int lastError = 0;
float filteredGyro = 0;
float gyroOffsetZ = 0;
float lastValidSteering = 0; 
int lostFrameCounter = 0;    

// Smooth Steering Memory
float smoothedSteering = 0;
const float steeringFilter = 0.45; // Faster filtering to allow quick high-speed turns

// -------------------- GYRO READ --------------------
int16_t readRawGyroZ() {
  Wire.beginTransmission(ITG3205_ADDR);
  Wire.write(0x21);
  Wire.endTransmission(false);
  Wire.requestFrom(ITG3205_ADDR, 2);

  if (Wire.available() >= 2) {
    return (Wire.read() << 8) | Wire.read();
  }
  return 0;
}

// -------------------- MOTOR CONTROL --------------------
void setMotor(int left, int right) {
  // Constrained precisely to the Arduino hardware limit of 255
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  digitalWrite(IN1, left >= 0 ? HIGH : LOW);
  digitalWrite(IN2, left >= 0 ? LOW : HIGH);

  digitalWrite(IN3, right >= 0 ? HIGH : LOW);
  digitalWrite(IN4, right >= 0 ? LOW : HIGH);

  analogWrite(ENA, abs(left));
  analogWrite(ENB, abs(right));
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pixy.init();
  pixy.changeProg("line");

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Wire.beginTransmission(ITG3205_ADDR);
  Wire.write(0x3E);
  Wire.write(0x00);
  Wire.endTransmission();

  Serial.println("Calibrating Gyro...");
  long sumZ = 0;
  for (int i = 0; i < 1000; i++) {
    sumZ += readRawGyroZ();
    delay(2);
  }
  gyroOffsetZ = sumZ / 1000.0;
  Serial.println("Gyro Ready");
}

// -------------------- LOOP --------------------
void loop() {
  // -------------------- GYRO PROCESSING --------------------
  float yawRate = (readRawGyroZ() - gyroOffsetZ) / 14.375;

  if (abs(yawRate) < 1.0) {
    yawRate = 0;
  }

  filteredGyro = 0.9 * filteredGyro + 0.1 * yawRate;
  float gyroCorrection = filteredGyro * Ky;

  // -------------------- PIXY FEATURE SELECTION --------------------
  pixy.line.getMainFeatures(); 
  int numVectors = pixy.line.numVectors;

  if (numVectors > 0) {
    lostFrameCounter = 0; 
    int calculatedTargetX = frameCenter;

    if (numVectors >= 2) {
      int line1_X = (pixy.line.vectors[0].m_x0 + pixy.line.vectors[0].m_x1) / 2;
      int line2_X = (pixy.line.vectors[1].m_x0 + pixy.line.vectors[1].m_x1) / 2;
      calculatedTargetX = (line1_X + line2_X) / 2; 
    } 
    else {
      int singleLineX = (pixy.line.vectors[0].m_x0 + pixy.line.vectors[0].m_x1) / 2;
      
      if (singleLineX > frameCenter) {
        calculatedTargetX = singleLineX - laneShift; 
      } else {
        calculatedTargetX = singleLineX + laneShift; 
      }
    }

    int error = calculatedTargetX - frameCenter; 
    error = constrain(error, -28, 28); // Widened error calculation window for ultra-fast response

    int curvature = error - lastError;
    lastError = error;

    // -------------------- EXPONENTIAL CORNER BRAKING --------------------
    // Rapidly drops power when a turn is detected to keep the vehicle on the track
    int currentBaseSpeed = maxStraightSpeed - (abs(error) * 5); 
    currentBaseSpeed = constrain(currentBaseSpeed, minCornerSpeed, maxStraightSpeed);

    // -------------------- PID STEERING --------------------
    float rawSteering = (error * Kp) + (curvature * Kd) + gyroCorrection;
    
    // Increased steering headroom up to 110 to force swift corrections at 255 top speed
    rawSteering = constrain(rawSteering, -110, 110); 

    // Smooth sudden adjustments to maintain traction
    smoothedSteering = (steeringFilter * rawSteering) + ((1.0 - steeringFilter) * smoothedSteering);
    
    lastValidSteering = smoothedSteering; 

    // -------------------- MOTOR DRIVE --------------------
    setMotor(currentBaseSpeed - smoothedSteering, currentBaseSpeed + smoothedSteering);

  } else {
    // -------------------- MEMORY FALLBACK MODE --------------------
    lostFrameCounter++;
    
    if (lostFrameCounter < 15) { // Shorter time frame because the robot covers more distance at high speeds
      setMotor(minCornerSpeed - lastValidSteering, minCornerSpeed + lastValidSteering);
    } 
    else {
      setMotor(0, 0);
      Serial.println("[CRITICAL LOST] Track Lost at Max Speed.");
    }
  }
  
  delay(4); // Reduced to 4ms for faster execution cycles
}
