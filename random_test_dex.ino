#include "Wire.h"
#include "PCA9685.h"

const int switchPin = 12;
const int NUM_JOINTS = 5;
unsigned long durationMs = 1500;

PCA9685 pwmController(Wire);  // Use Wire (default I2C) @100kHz, default I2C address (0x40)
// Create a ServoEvaluator object for servo control (only helper object, not an entity!)
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

ServoMotor servoArray[NUM_JOINTS];


void setup() {
  
  pinMode(LED_BUILTIN, OUTPUT);  // Set onboard LED as output
  pinMode(switchPin, INPUT_PULLUP);  // Enable internal pull-up

  Serial.begin(115200);  // Begin Serial and Wire interface
  Serial.println("setup()");
  Wire.begin();

  // pwmController.resetDevices();       // Reset all PCA9685 devices on I2C line
  pwmController.init();             // Initialize module with default settings
  pwmController.setPWMFreqServo();  // Set to standard 50Hz (20ms servo cycle)

  for (int i = 0; i < NUM_JOINTS; i++) {
    servoArray[i] = ServoMotor(i);
  }

  float initJointCoordinates[NUM_JOINTS] = {0, 0, 0, 0, 0};
  float initGlobalCoordinates = 0;
  int pwmVal = helperServo.pwmForAngle(initGlobalCoordinates);

  for(int i = 0; i < NUM_JOINTS; i++)
  { 
    pwmController.setChannelPWM(servoArray[i].channel, pwmVal);
  }
  pwmController.setChannelPWM(5, pwmVal);

}

void loop() {


  int switchState = digitalRead(switchPin);
  if (switchState == HIGH) {
    // Switch off
    Serial.println("Switch is off.");
    delay(1000);
    return;
  } 

  Serial.println("Loop started.");


  moveset1();
  Serial.println("Loop ended");
  
  
}


void moveset1()
{

  float jointCoordinates[NUM_JOINTS];
  unsigned long moveset1Delay = 700;
  unsigned long prepDelay = 5000;


  // POSE 1 - Home position
  openGripper();
  jointCoordinates[0] = 0;
  jointCoordinates[1] = -45;
  jointCoordinates[2] = -90;
  jointCoordinates[3] = 45;
  jointCoordinates[4] = 0;
  move(jointCoordinates, servoArray, durationMs);
  delay(moveset1Delay);
  delay(prepDelay);

  // POSE 2 - Pick target pre-approach
  jointCoordinates[0] = 45;
  jointCoordinates[1] = 5;
  jointCoordinates[2] = -45;
  jointCoordinates[3] = 90;
  jointCoordinates[4] = 0;
  move(jointCoordinates, servoArray, durationMs);
  delay(moveset1Delay);


  // POSE 3 - Pick target approach and grip
  jointCoordinates[0] = 45;
  jointCoordinates[1] = 13;
  jointCoordinates[2] = -45;
  jointCoordinates[3] = 80;
  jointCoordinates[4] = 0;
  move(jointCoordinates, servoArray,500);
  delay(moveset1Delay);
  closeGripper();


  // POSE 4 - Pick target pre-approach
  jointCoordinates[0] = 45;
  jointCoordinates[1] = 5;
  jointCoordinates[2] = -45;
  jointCoordinates[3] = 90;
  jointCoordinates[4] = 0;
  move(jointCoordinates, servoArray, 500);
  delay(moveset1Delay);


  // POSE 5 - Drop target pre-approach
  jointCoordinates[0] = -45;
  jointCoordinates[1] = 5;
  jointCoordinates[2] = -45;
  jointCoordinates[3] = 90;
  jointCoordinates[4] = -60;
  move(jointCoordinates, servoArray, 2500);
  delay(moveset1Delay);


  // POSE 6 - Drop target approach and release
 jointCoordinates[0] = -45;
  jointCoordinates[1] = 13;
  jointCoordinates[2] = -45;
  jointCoordinates[3] = 80;
  jointCoordinates[4] = -60;
  move(jointCoordinates, servoArray, 500);
  delay(moveset1Delay);
  openGripper();


  // POSE 7 - Drop target pre-approach
  jointCoordinates[0] = -45;
  jointCoordinates[1] = 5;
  jointCoordinates[2] = -45;
  jointCoordinates[3] = 90;
  jointCoordinates[4] = -60;
  move(jointCoordinates, servoArray, 500);
  delay(moveset1Delay);


  // POSE 8 - Home position
  jointCoordinates[0] = 0;
  jointCoordinates[1] = -45;
  jointCoordinates[2] = -90;
  jointCoordinates[3] = 45;
  jointCoordinates[4] = 0;
  move(jointCoordinates, servoArray, durationMs);
  delay(moveset1Delay);
  delay(prepDelay);
  


}

void openGripper()
{
  unsigned long gripperDurationMs = 500;
  // Angle for closed gripper = 0 deg
  // Angle for open gripper = -45 deg
  cubicInterpolateServo(0, -45, gripperDurationMs, 5); 
  delay(300);
  
}

void closeGripper()
{
  unsigned long gripperDurationMs = 500;
  // Angle for closed gripper = 0 deg
  // Angle for open gripper = -45 deg
  cubicInterpolateServo(-45, 0, gripperDurationMs, 5);  
  delay(300);

}


void moveset0()
{

  float jointCoordinates[NUM_JOINTS];
  float globalCoordinates;

  // TARGET 1

    jointCoordinates[0] = 0;
    jointCoordinates[1] = -45;
    jointCoordinates[2] = -90;
    jointCoordinates[3] = 45;
    jointCoordinates[4] = 45;
  move(jointCoordinates, servoArray, durationMs);
  delay(1000);


  // TARGET 2
  globalCoordinates = 0;
  for(int i = 0; i < NUM_JOINTS; i++) {
    jointCoordinates[i] = globalCoordinates;
  }
  move(jointCoordinates, servoArray, durationMs);
  delay(1000);


  // TARGET 3
    jointCoordinates[0] = 0;
    jointCoordinates[1] = -45;
    jointCoordinates[2] = -90;
    jointCoordinates[3] = 45;
    jointCoordinates[4] = 45;
  move(jointCoordinates, servoArray, durationMs);
  delay(1000);



  // BACK TO ZERO
  globalCoordinates = 0;
  for(int i = 0; i < NUM_JOINTS; i++) {
    jointCoordinates[i] = globalCoordinates;
  }
  move(jointCoordinates, servoArray, durationMs);
  delay(1000);

}


void move(const float jointCoordinates[NUM_JOINTS], ServoMotor servoArray[NUM_JOINTS], unsigned long durationMs){
  
  // update end target angles
  for(int i = 0; i < NUM_JOINTS; i++)
  {
    servoArray[i].endAngle = jointCoordinates[i];
  }

  const unsigned long stepDelay = 20; // milliseconds between steps
  const int steps = durationMs / stepDelay;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps; // Normalize time [0,1]
        
      // Cubic interpolation (Hermite basis: smooth start and stop)
      float h00 = 2 * t * t * t - 3 * t * t + 1;
      float h01 = -2 * t * t * t + 3 * t * t;
    for(int j = 0; j < NUM_JOINTS; j++)
    { 
      // Interpolated angle based on the cubic interpolation formula
      float interpolatedAngle = h00 * servoArray[j].startAngle + h01 * servoArray[j].endAngle;

      // Convert the interpolated angle to PWM and move the servo
      int pwmVal = helperServo.pwmForAngle(interpolatedAngle);

      if(servoArray[j].channel == 0){ 
      Serial.print("Angle = ");
      Serial.print(interpolatedAngle);
      Serial.print(", PWM = ");
      Serial.print(pwmVal);
      Serial.println("");
      }
      pwmController.setChannelPWM(servoArray[j].channel, pwmVal);

      servoArray[j].currentAngle = interpolatedAngle;
    }
      delay(stepDelay);
    }


    for(int i = 0; i < NUM_JOINTS; i++)
  {
    servoArray[i].startAngle = servoArray[i].endAngle;
  }

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

      // Print the interpolation details for debugging
        Serial.print("t=");
        Serial.print(t, 2);
        Serial.print(" | angle=");
        Serial.print(interpolatedAngle, 2);
        Serial.print(" | PWM=");
        Serial.println(pwmVal);

      delay(stepDelay);
    }
}




// Function to move servo to a given angle (0-180 degrees) without interpolation
void moveServo1(int angle, uint8_t channel) {
  // Ensure angle is within valid range
  //angle = constrain(angle, 0, 180);

  // Convert the angle (0-180) to the corresponding PWM pulse width
  int pwmValue = helperServo.pwmForAngle(angle);
  Serial.print("PWM pulse = ");
  Serial.println(pwmValue);
  // Set the PWM value for servo on channel 0
  pwmController.setChannelPWM(channel, pwmValue);

  // Optionally print the PWM value for debugging
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" -> PWM: ");
  Serial.println(pwmValue);
}

/*


*/
