// Random Test sketch for Dextery-T
#include "Wire.h"
#include "PCA9685.h"

const int switchPin = 12;
const int ledPin = LED_BUILTIN;

PCA9685 pwmController(Wire);  // Use Wire (default I2C) @100kHz, default I2C address
int t1 = 1500;
// Create a ServoEvaluator object for servo control
PCA9685_ServoEval servo1;

void setup() {
   pinMode(LED_BUILTIN, OUTPUT);  // Set onboard LED as output

  pinMode(switchPin, INPUT_PULLUP);  // Enable internal pull-up
  pinMode(ledPin, OUTPUT);

  Serial.begin(115200);  // Begin Serial and Wire interface
  Serial.println("setup()");
  Wire.begin();
  Serial.println("Wire began");

 // pwmController.resetDevices();       // Reset all PCA9685 devices on I2C line
  pwmController.init();             // Initialize module with default settings
 pwmController.setPWMFreqServo();  // Set to standard 50Hz (20ms servo cycle)
  Serial.println("setup() done");
}

void loop() {
  int switchState = digitalRead(switchPin);
  if (switchState == HIGH) {
    // Switch off
    digitalWrite(ledPin, LOW);
    Serial.println("Switch is off.");
    delay(1000);
    return;
  } 


  Serial.println("loop()");
  digitalWrite(LED_BUILTIN, HIGH); // LED ON
  delay(500);                     // Wait 0.5s

  digitalWrite(LED_BUILTIN, LOW);  // LED OFF
  delay(500);                     // Wait 0.5s
  

  cubicInterpolateServo(-90, 90, t1, 0);  // Move from -45° to +45° over 2 seconds
    delay(1000);
    cubicInterpolateServo(90, -90, t1, 0);  // Move back over 2 seconds
    delay(1000);
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
      int pwmVal = servo1.pwmForAngle(interpolatedAngle);
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




// Function to move servo1 to a given angle (0-180 degrees) without interpolation
void moveServo1(int angle, uint8_t channel) {
  // Ensure angle is within valid range
  //angle = constrain(angle, 0, 180);

  // Convert the angle (0-180) to the corresponding PWM pulse width
  int pwmValue = servo1.pwmForAngle(angle);
  Serial.print("PWM pulse = ");
  Serial.println(pwmValue);
  // Set the PWM value for servo1 on channel 0
  pwmController.setChannelPWM(channel, pwmValue);

  // Optionally print the PWM value for debugging
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" -> PWM: ");
  Serial.println(pwmValue);
}


