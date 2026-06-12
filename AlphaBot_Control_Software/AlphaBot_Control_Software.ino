*******************************************************************************************************
          Hochschule Hamm-Lippstadt
 *******************************************************************************************************
  Modul           :   AlphaBot
  Date            :   26.05.2026
  Function        :   Lane detection with pixy2 camera and recording yaw rate and yaw angle.
  Hardware        :   UNO R4 WIFI, Pixy2 camera and Gy-85 sensor
  Implementation  :   Arduino IDE 2.3.86
  Author          :   Syed Muhammad Abis Rizvi 
 References       :   Waveshare AlphaBot demo code (www.waveshare.com/wiki/AlphaBot)
                      GY-85 IMU sensor  Arduino examples and documentation (https://docs.arduino.cc/)
                      Pixy2 examples and documentation (https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:arduino_api)
                      AI (conceptual assistance and code structuring)
 Last Modified   :    12.06.2026
*******************************************************************************************************/
#include <WiFiS3.h>
#include <Pixy2I2C.h>
#include <Wire.h>

// WiFi Credentials
char ssid[] = "AutonomeSysteme";
char pass[] = "Kennwort1";

// Networking Definitions
IPAddress local_IP(192, 168, 1, 23);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiServer server(5000); 
WiFiClient matlabClient;

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
float Kp = 7.2;   // Increased to react fast enough at maximum speed (without 6.0/PRISMA 6.6 )
float Kd = 4.0;   // Increased to prevent extreme high-speed weaving (without 3.8/PRISMA 4.0 )
float Ky = 0.06;  // Increased to lock straight lines during full throttle (without 0.06/PRISMA 0.06 )
float Kpsi = 0.05; //Increased to improve curve anticipation and maintain smooth trajectory at high speed

// ===================== SETTINGS ===================== 
const int frameCenter = 39; /
const int laneShift   = 20; // horizontal pixel offset use to calculate a virtual center line (without 22/PRISMA 20)

// -------------------- MAXIMUM SPEED CONFIGURATION --------------------
int maxStraightSpeed = 255; // Max speed PWM (Pulse Width Modulation)
int minCornerSpeed   = 80;  // min speed during turns

// -------------------- STATE & MEMORY --------------------
int lastError = 0;
float filteredGyro = 0;
float gyroOffsetZ = 0;
float lastValidSteering = 0; 
int lostFrameCounter = 0;    
float yawAngle = 0;         
float headingError = 0;
float integralError = 0;
float lineHeading = 0;  
unsigned long lastMicros = 0;  
unsigned long lastWiFiMillis = 0; //   
unsigned long sendCounter = 0;    //  

// Smooth Steering Memory
float smoothedSteering = 0;
const float steeringFilter = 0.45; // Faster filtering to allow quick high-speed turns

// -------------------- GYRO READ --------------------
int16_t readRawGyroZ() {
  Wire.beginTransmission(ITG3205_ADDR);
  Wire.write(0x21);
  Wire.endTransmission(false);
  Wire.requestFrom(ITG3205_ADDR, 2);

  if (Wire.available() >= 2) { // Requests 2 bytes of data (High byte and Low byte) from the gyro.
    return (Wire.read() << 8) | Wire.read(); // Combines the two 8-bit readings into a single signed 16-bit integer via bit-shifting (<< 8) and bitwise 
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

  // -------------------- WLAN INITIALIZATION --------------------
  // Configured inside setup() to prevent code blocking loops later
  WiFi.config(local_IP, gateway, subnet);
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) { 
    delay(500); 
    Serial.print(".");
  }
  
  server.begin();
  lastMicros = micros();
  lastWiFiMillis = millis();
  Serial.println("\nWireless TCP Server Ready at 192.168.1.23:5000");
}

// -------------------- MAIN LOOP --------------------
void loop() {
  // -------------------- TIME DELTA CALCULATION --------------------
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0; // Convert to seconds
  lastMicros = currentMicros;

  // -------------------- GYRO PROCESSING --------------------
  float yawRate = (readRawGyroZ() - gyroOffsetZ) / 14.375;

  if (abs(yawRate) < 1.0) {
    yawRate = 0;
  }

  filteredGyro = 0.9 * filteredGyro + 0.1 * yawRate; // Applies a Low-Pass filter (90% historical data, 10% new data) to eliminate vibration noise
  float gyroCorrection = filteredGyro * Ky; // steering correction
  
  // -------------------- YAW ANLGE --------------------
  yawAngle += filteredGyro * dt; // Integration calculation 
  
  // -------------------- MATLAB TCP SERVER --------------------
  // Check for newly connecting clients
  if (!matlabClient || !matlabClient.connected()) {
    matlabClient = server.accept(); //  Client assignment for UNO R4 WiFi
  }

  // Send telemetry packets at 50Hz (every 20ms)
  if (matlabClient && matlabClient.connected()) {
    if (millis() - lastWiFiMillis >= 20) { 
      lastWiFiMillis = millis();
      matlabClient.print(filteredGyro);
      matlabClient.print(",");
      matlabClient.print(yawAngle);
      matlabClient.print(",");
      matlabClient.println(sendCounter++); 
    }
  }

  // -------------------- PIXY FEATURE SELECTION --------------------
  pixy.line.getMainFeatures(); 
  int numVectors = pixy.line.numVectors;

  if (numVectors > 0) {
    lostFrameCounter = 0; 
       // -------------------- LINE HEADING --------------------
int x0 = pixy.line.vectors[0].m_x0;
int y0 = pixy.line.vectors[0].m_y0;
int x1 = pixy.line.vectors[0].m_x1;
int y1 = pixy.line.vectors[0].m_y1;

// Correct vector direction if Pixy flips the vector
if (y1 > y0) {
  int tempX = x0;
  x0 = x1;
  x1 = tempX;

  int tempY = y0;
  y0 = y1;
  y1 = tempY;
}

int dx = x1 - x0;
int dy = y1 - y0;

lineHeading = atan2(dy, dx) * 180.0 / PI;

// Desired straight-line heading in Pixy image coordinates
float desiredHeading = -90.0;

headingError = lineHeading - desiredHeading;

// Normalize to -180 ... 180 degrees
if (headingError > 180) headingError -= 360;
if (headingError < -180) headingError += 360;

    // -------------------- TARGET CENTER --------------------
    int calculatedTargetX = frameCenter;

    if (numVectors >= 2) {
      int line1_X = (pixy.line.vectors[0].m_x0 + pixy.line.vectors[0].m_x1) / 2;
      int line2_X = (pixy.line.vectors[1].m_x0 + pixy.line.vectors[1].m_x1) / 2;
      calculatedTargetX = (line1_X + line2_X) / 2;  // find midpoint of detercted line which calculates a precise trajectory right through the middle of the path.
    } 
    else {
      int singleLineX = (pixy.line.vectors[0].m_x0 + pixy.line.vectors[0].m_x1) / 2; // One line detected: Finds the midpoint of the single line. If it is on the right side of the screen, it shifts the target left by laneShift pixels. If it is on the left, it shifts it right.
      
      if (singleLineX > frameCenter) {
        calculatedTargetX = singleLineX - laneShift; 
      } else {
        calculatedTargetX = singleLineX + laneShift; 
      }
    }
  // -------------------- LANE OFFSET --------------------
   
    int error = calculatedTargetX - frameCenter; 
    error = constrain(error, -28, 28); // spatial error distance which limits the error window to a max 28 pixels left or right
   
    // -------------------- CURVATURE --------------------

    int curvature = error - lastError;
    lastError = error;

      // -------------------- HEADING ERROR --------------------

      //headingError = lineHeading - yawAngle;

    // -------------------- SPEED CONTROL WITH EXPONENTIAL CORNER BRAKING --------------------
       // Rapidly drops power when a turn is detected to keep the vehicle on the track
    int currentBaseSpeed = maxStraightSpeed - (abs(error) * 5); //  lowers the robot's base speed by 5 units for every pixel of tracking error.
    currentBaseSpeed = constrain(currentBaseSpeed, minCornerSpeed, maxStraightSpeed);

    // -------------------- PID STEERING --------------------
 float rawSteering =
  (error * Kp) + (headingError * Kpsi) +(filteredGyro * Ky) +(curvature * Kd);

    // Increased steering headroom up to 110 to force swift corrections at 255 top speed
    rawSteering = constrain(rawSteering, -110, 110); // max range of 110 to avoid over-steering spinouts.

    // Smooth sudden adjustments to maintain traction
    smoothedSteering = (steeringFilter * rawSteering) + ((1.0 - steeringFilter) * smoothedSteering);
    lastValidSteering = smoothedSteering; 

    // -------------------- MOTOR DRIVE --------------------
    setMotor(currentBaseSpeed - smoothedSteering, currentBaseSpeed + smoothedSteering);

  } else {
    // -------------------- MEMORY FALLBACK MODE --------------------
    lostFrameCounter++;
    
    if (lostFrameCounter < 15) {  // Shorter time frame because the robot covers more distance at high speeds
      setMotor(minCornerSpeed - lastValidSteering, minCornerSpeed + lastValidSteering);
    } 
    else {
      setMotor(0, 0);
      Serial.println("[CRITICAL LOST] Track Lost at Max Speed.");
    }
  }
  
  delay(4); // Reduced  for faster execution cycles
} 

