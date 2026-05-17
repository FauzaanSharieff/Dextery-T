#include "Wire.h"
#include "PCA9685.h"

const int switchPin = 12;     
const int NUM_JOINTS = 5;
String command = "idle,c";    // Start with a closed gripper

PCA9685 pwmController(Wire);  
PCA9685_ServoEval helperServo;

class ServoMotor{

public:
  uint8_t channel;
  float startAngle;
  float endAngle;
  float currentAngle;

  ServoMotor() : channel(0), startAngle(0), endAngle(0), currentAngle(0) {}
  
  ServoMotor(uint8_t ch) {
    channel = ch;
    startAngle = 0;
    currentAngle = startAngle;
    endAngle = startAngle;
  }
};

class Position{
  public:
  float x;
  float y;
  float z;
  float target_x;
  float target_y;
  float target_z;
  float current_x;
  float current_y;
  float current_z;
  bool gripperOpen;
  Position() : x(0), y(0), z(0), target_x(14.5), target_y(0), target_z(0), current_x(14.5), current_y(0), current_z(0), gripperOpen(false) {}
};

ServoMotor servoArray[NUM_JOINTS];
Position pos;

void setup() {
  
  pinMode(switchPin, INPUT_PULLUP);  

  Serial.begin(115200);             // Begin Serial and Wire interface
  Wire.begin();
  pwmController.init();             // Initialize module with default settings
  pwmController.setPWMFreqServo();  // Set to standard 50Hz (20ms servo cycle)

  // Constructor for setting PWM channels to object
  for (int i = 0; i < NUM_JOINTS; i++) {
    servoArray[i] = ServoMotor(i);
  }

  float initGlobalCoordinate = 0;     // Set all motors to 0 position
  int pwmVal = helperServo.pwmForAngle(initGlobalCoordinate);
  for(int i = 0; i < NUM_JOINTS; i++)
    pwmController.setChannelPWM(servoArray[i].channel, pwmVal);
  pwmController.setChannelPWM(5, pwmVal);

}

void loop() {

  if (digitalRead(switchPin) == HIGH) {
    Serial.println("Switch is off.");
    delay(1000);
    return;
  } 

  updateCommand(); 
  Serial.println(command.c_str());

  if (command != "idle,o" && command != "idle,c") {
    String moveCommand = command;   // copy current command
    move(moveCommand.c_str());
  }
 
  if(command == "idle,o"){
    if(!pos.gripperOpen)
      openGripper();
  }

  if(command == "idle,c"){
    if(pos.gripperOpen)
      closeGripper(); 
  }  
  delay(50); 
  
}


void move(const char* dir)
{
  calculateCartesianTarget(dir);    // Updates pos.target_x/y/z

  const int cartesianSteps = 10;

  float start_x = pos.current_x;
  float start_y = pos.current_y;
  float start_z = pos.current_z;

  float target_x = pos.target_x;
  float target_y = pos.target_y;
  float target_z = pos.target_z;

  float prev_x = start_x;
  float prev_y = start_y;
  float prev_z = start_z;

  float stepSpeed = 12;   // cm/s
  float stepDistance = 0; // distance moved along the one axis
  unsigned long stepTime = 0; // time it should take to complete one stepDistance

  for(int cartesianStep = 1; cartesianStep <= cartesianSteps; cartesianStep++) {
    float fraction = (float)cartesianStep / cartesianSteps;

    float x = start_x + fraction * (target_x - start_x);
    float y = start_y + fraction * (target_y - start_y);
    float z = start_z + fraction * (target_z - start_z);

    float dx = x - prev_x;
    float dy = y - prev_y;
    float dz = z - prev_z;

    stepDistance = sqrt(dx * dx + dy * dy + dz * dz);
    stepTime = (stepDistance / stepSpeed) * 1000;
    if(stepTime < 20){ 
      stepTime = 20;
    }

    moveCartesian(x, y, z);    // Updates .endAngle servo data member
    moveJoint(stepTime);       // Moves to .endAngle

    prev_x = x;
    prev_y = y;
    prev_z = z;

    if(haltOperation())
      return;

  }
}
 
void calculateCartesianTarget(const char* dir)
{
  // The robot has a 3D workspace that looks like a front-top half of a torus
  // X limits for Y = 0 (same for Y when X = 0 except Y has + and - counterparts)
  float xMinLimit = 7.0;
  float xMaxLimit = 22.0;

  // Torus parameters
  float R = (xMaxLimit + xMinLimit) / 2.0;   // centerline radius
  float r = (xMaxLimit - xMinLimit) / 2.0;   // cross-section radius / tube radius

  char sign = dir[0];  // '+' or '-'
  char axis = dir[1];  // 'x', 'y', or 'z'

  // Start by making target equal to current position.
  // Then only change the requested axis.
  pos.target_x = pos.current_x;
  pos.target_y = pos.current_y;
  pos.target_z = pos.current_z;

  float minLimit;
  float maxLimit;

  if(axis == 'x') {
    getXLimits(pos.current_y, pos.current_z, R, r, minLimit, maxLimit);
    if(sign == '+') {
      pos.target_x = maxLimit;
    }
    else if(sign == '-') {
      pos.target_x = minLimit;
    }
  }

  else if(axis == 'y') {
    getYLimits(pos.current_x, pos.current_y, pos.current_z, R, r, minLimit, maxLimit);
    if(sign == '+') {
      pos.target_y = maxLimit;
    }
    else if(sign == '-') {
      pos.target_y = minLimit;
    }
  }

  else if(axis == 'z') {
    getZLimits(pos.current_x, pos.current_y, R, r, minLimit, maxLimit);
    if(sign == '+') {
      pos.target_z = maxLimit;
    }
    else if(sign == '-') {
      pos.target_z = minLimit;
    }
  }
}

void moveCartesian(float x, float y, float z)
{
  // Link lengths in cm
  const float l[NUM_JOINTS] = {10.0, 10.0, 12.5, 0.0, 17.5};

  // Joint coordinates in radians
  float q[NUM_JOINTS];
  // Joint coordinates in degrees
  float qDeg[NUM_JOINTS];

  float T05[4][4] = {
    {0.0, 0.0, 1.0, x},
    {1.0, 0.0, 0.0, y},
    {0.0, 1.0, 0.0, z},
    {0.0, 0.0, 0.0, 1.0}
  };

  float v1[4] = {0.0, 0.0, 0.0, 1.0};
  float v2[4] = {0.0, 1.0, 0.0, 0.0};

  float T05_v1[4] = {0.0, 0.0, 0.0, 0.0};
  float T05_v2[4] = {0.0, 0.0, 0.0, 0.0};

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      T05_v1[i] += T05[i][j] * v1[j];
      T05_v2[i] += T05[i][j] * v2[j];
    }
  }

  float wrist[4];

  for (int i = 0; i < 4; i++) {
    wrist[i] = T05_v1[i] + l[4] * T05_v2[i];
  }

  float x_wrist = wrist[0];
  float y_wrist = wrist[1];
  float z_wrist = wrist[2];

  float li_xcomp = sqrt(x_wrist * x_wrist + y_wrist * y_wrist);
  float li_ycomp = z_wrist - l[0];
  float li = sqrt(li_xcomp * li_xcomp + li_ycomp * li_ycomp);
  float alphaArg = (li * li + l[2] * l[2] - l[1] * l[1]) / (2.0 * li * l[2]);
  float betaArg  = (li * li + l[1] * l[1] - l[2] * l[2]) / (2.0 * li * l[1]); 

  alphaArg = constrain(alphaArg, -1.0, 1.0);
  betaArg  = constrain(betaArg,  -1.0, 1.0);  

  float alpha1 = acos(alphaArg);
  float beta1  = acos(betaArg);
  float phi1   = atan2((z_wrist - l[0]), li_xcomp);

  q[0] = atan2(y_wrist, x_wrist);
  q[1] = PI / 2.0 - (phi1 + beta1);
  q[2] = -(PI / 2.0 - (alpha1 + beta1));
  q[3] = -(q[1] + q[2]);
  q[4] = q[0];

  // Convert radians to degrees
  for (int i = 0; i < NUM_JOINTS; i++) {
    if(i == 2)
       qDeg[i] = q[i] * -180.0 / PI;
    else
      qDeg[i] = q[i] * 180.0 / PI;
  }

  // Write target joint angles into global servoArray object
  for (int i = 0; i < NUM_JOINTS; i++) {
    servoArray[i].endAngle = qDeg[i];
  }

}

void moveJoint(unsigned long duration){
  // servoArray[i].endAngle already updated in another previous call

  const unsigned long stepDelay = 20; // milliseconds between steps
  const int steps = duration / stepDelay;

  for (int i = 0; i <= steps; i++) {
  float progress = (float)i / steps; // Normalize time [0,1]
 
  for (int j = 0; j < NUM_JOINTS; j++)
  { 
    float interpolatedAngle = servoArray[j].startAngle + progress * (servoArray[j].endAngle - servoArray[j].startAngle);

    float finalInterpolatedAngle = interpolatedAngle;
    if (j == 1) 
      finalInterpolatedAngle -= 20;

    if(finalInterpolatedAngle > 90)
      finalInterpolatedAngle = 90;

    else if(finalInterpolatedAngle < -90)
      finalInterpolatedAngle = -90;

    int pwmVal = helperServo.pwmForAngle(finalInterpolatedAngle);
    pwmController.setChannelPWM(servoArray[j].channel, pwmVal);

    servoArray[j].currentAngle = interpolatedAngle;
  }
  forwardKinematics();
  delay(stepDelay);

  if(haltOperation())
  {
    for(int k = 0; k < NUM_JOINTS; k++)
      servoArray[k].startAngle = servoArray[k].currentAngle;
    return;
  }
}

  for(int i = 0; i < NUM_JOINTS; i++)
    servoArray[i].startAngle = servoArray[i].currentAngle;

}

void forwardKinematics()
{
  const float l[NUM_JOINTS] = {10.0, 10.0, 12.5, 0.0, 17.5};

  float theta[NUM_JOINTS];

  // Convert current logical joint angles from degrees to radians.
  // Joint index 2 was inverted in IK, so invert it back here.
  for(int i = 0; i < NUM_JOINTS; i++) {
    if(i == 2) {
      theta[i] = -servoArray[i].currentAngle * PI / 180.0;
    }
    else {
      theta[i] = servoArray[i].currentAngle * PI / 180.0;
    }
  }

  float phi[NUM_JOINTS];
  float d[NUM_JOINTS];
  float a[NUM_JOINTS];
  float alpha[NUM_JOINTS];

  phi[0]   = theta[0];
  d[0]     = l[0];
  a[0]     = 0.0;
  alpha[0] = -PI / 2.0;

  phi[1]   = theta[1] - PI / 2.0;
  d[1]     = 0.0;
  a[1]     = l[1];
  alpha[1] = 0.0;

  phi[2]   = theta[2] + PI / 2.0;
  d[2]     = 0.0;
  a[2]     = l[2];
  alpha[2] = 0.0;

  phi[3]   = theta[3] + PI;
  d[3]     = 0.0;
  a[3]     = 0.0;
  alpha[3] = PI / 2.0;

  phi[4]   = theta[4] + PI / 2.0;
  d[4]     = l[4];
  a[4]     = 0.0;
  alpha[4] = -PI / 2.0;

  float T01[4][4];
  float T12[4][4];
  float T23[4][4];
  float T34[4][4];
  float T45[4][4];

  createDHMatrix(phi[0], d[0], a[0], alpha[0], T01);
  createDHMatrix(phi[1], d[1], a[1], alpha[1], T12);
  createDHMatrix(phi[2], d[2], a[2], alpha[2], T23);
  createDHMatrix(phi[3], d[3], a[3], alpha[3], T34);
  createDHMatrix(phi[4], d[4], a[4], alpha[4], T45);

  float T02[4][4];
  float T03[4][4];
  float T04[4][4];
  float T05[4][4];

  multiplyMatrix4x4(T01, T12, T02);
  multiplyMatrix4x4(T02, T23, T03);
  multiplyMatrix4x4(T03, T34, T04);
  multiplyMatrix4x4(T04, T45, T05);

  // Extract TCP position from T05
  pos.current_x = T05[0][3];
  pos.current_y = T05[1][3];
  pos.current_z = T05[2][3];
/* 
    Serial.print("Current position: ");
    Serial.print(pos.current_x);
    Serial.print(", ");
    Serial.print(pos.current_y);
    Serial.print(", ");
    Serial.println(pos.current_z);
    */
    
}

void cubicInterpolateServo(float startAngle, float endAngle, unsigned long durationMs, uint8_t channel) {

  const unsigned long stepDelay = 20; // milliseconds between steps
  const int steps = durationMs / stepDelay;

  // Interpolation coefficients for cubic (smooth start and stop)
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps; // Normalize time [0,1]
        
      // Cubic interpolation (Hermite basis: smooth start and stop)
      float h00 = 2 * t * t * t - 3 * t * t + 1;
      float h01 = -2 * t * t * t + 3 * t * t;

      // Interpolated angle based on the cubic interpolation formula
      float interpolatedAngle = h00 * startAngle + h01 * endAngle;

      // Convert the interpolated angle to PWM and move the servo
      int pwmVal = helperServo.pwmForAngle(interpolatedAngle);
     pwmController.setChannelPWM(channel, pwmVal);

     delay(stepDelay);
    }
}

void openGripper()
{
  unsigned long gripperDurationMs = 300;
  // Angle for closed gripper = 0 deg
  // Angle for open gripper = -45 deg
  cubicInterpolateServo(0, -45, gripperDurationMs, 5); 
  pos.gripperOpen = true;
  delay(300);
}

void closeGripper()
{
  unsigned long gripperDurationMs = 300;
  // Angle for closed gripper = 0 deg
  // Angle for open gripper = -45 deg
  cubicInterpolateServo(-45, 0, gripperDurationMs, 5);  
  pos.gripperOpen = false;
  delay(300);
}

void updateCommand()
{
  while (Serial.available() > 0) {
    String newCommand = Serial.readStringUntil('\n');
    newCommand.trim();

    if (newCommand.length() > 0) {
      command = newCommand;
    }
  }
}

bool haltOperation()
{
  updateCommand();
  if (command == "idle,o" || command == "idle,c" || digitalRead(switchPin) == HIGH) 
    return true;
  
  return false;
}

void getXLimits(float y, float z, float R, float r, float &xMin, float &xMax)
{
  // Workspace:
  // (sqrt(x^2 + y^2) - R)^2 + z^2 <= r^2
  // with x >= 0 and z >= 0

  float s = sqrt(r * r - z * z);

  float rhoMin = R - s;
  float rhoMax = R + s;

  float outerVal = rhoMax * rhoMax - y * y;
  xMax = sqrt(outerVal);

  float innerVal = rhoMin * rhoMin - y * y;

  if(innerVal > 0) {
    xMin = sqrt(innerVal);
  }
  else {
    xMin = 0;
  }

}

void getZLimits(float x, float y, float R, float r, float &zMin, float &zMax)
{
  float rho = sqrt(x * x + y * y);

  float zVal = r * r - (rho - R) * (rho - R);

  zMin = 0;
  zMax = sqrt(zVal);
}

void getYLimits(float x, float y, float z, float R, float r, float &yMin, float &yMax)
{
  float s = sqrt(r * r - z * z);

  float rhoMin = R - s;
  float rhoMax = R + s;

  float outerVal = rhoMax * rhoMax - x * x;
  float yOuter = sqrt(outerVal);

  float innerVal = rhoMin * rhoMin - x * x;

  // Case 1: no forbidden inner hole for this x,z slice.
  // Valid y range is continuous: -yOuter <= y <= +yOuter
  if(innerVal <= 0) {
    yMin = -yOuter;
    yMax =  yOuter;
  }

  // Case 2: inner hole exists.
  // Valid y regions are:
  // -yOuter <= y <= -yInner
  //  yInner <= y <=  yOuter
  else {
    float yInner = sqrt(innerVal);

    float yOuterNeg = -yOuter;
    float yInnerNeg = -yInner;
    float yInnerPos =  yInner;
    float yOuterPos =  yOuter;

    if(y < 0) {
      // Negative y branch.
      // Moving -y goes to outer negative boundary.
      // Moving +y goes to inner negative boundary.
      yMin = yOuterNeg;
      yMax = yInnerNeg;
    }
    else {
      // Positive y branch.
      // Moving -y goes to inner positive boundary.
      // Moving +y goes to outer positive boundary.
      yMin = yInnerPos;
      yMax = yOuterPos;
    }
  }
}

void multiplyMatrix4x4(float A[4][4], float B[4][4], float result[4][4])
{
  float temp[4][4];

  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      temp[i][j] = 0.0;

      for(int k = 0; k < 4; k++) {
        temp[i][j] += A[i][k] * B[k][j];
      }
    }
  }

  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      result[i][j] = temp[i][j];
    }
  }
}

void createDHMatrix(float phi, float d, float a, float alpha, float T[4][4])
{
  T[0][0] = cos(phi);
  T[0][1] = -cos(alpha) * sin(phi);
  T[0][2] =  sin(alpha) * sin(phi);
  T[0][3] = a * cos(phi);

  T[1][0] = sin(phi);
  T[1][1] = cos(alpha) * cos(phi);
  T[1][2] = -sin(alpha) * cos(phi);
  T[1][3] = a * sin(phi);

  T[2][0] = 0.0;
  T[2][1] = sin(alpha);
  T[2][2] = cos(alpha);
  T[2][3] = d;

  T[3][0] = 0.0;
  T[3][1] = 0.0;
  T[3][2] = 0.0;
  T[3][3] = 1.0;
}

