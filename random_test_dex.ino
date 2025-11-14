// Random Test sketch for Dextery-T

#include "PCA9685.h"

PCA9685 pwmController(Wire);  // Use Wire (default I2C) @100kHz, default I2C address
uint8_t angle1 = 5;           // Example input angle (0-180 degrees)
uint8_t angle2 = 15;
int t1 = 2000;
// Create a ServoEvaluator object for servo control
PCA9685_ServoEval servo1;

void setup() {
  Serial.begin(115200);  // Begin Serial and Wire interface
  Wire.begin();

  //pwmController.resetDevices();       // Reset all PCA9685 devices on I2C line
  pwmController.init();             // Initialize module with default settings
  pwmController.setPWMFreqServo();  // Set to standard 50Hz (20ms servo cycle)
}

void loop() {
  
  /*
  moveServo1(angle1, 0);  // Move the servo to the desired angle
  delay(1000);            // Wait for 1 second before changing the angle again
  moveServo1(angle2, 0);
  delay(1000);
  Serial.println(" ");
  */
  cubicInterpolateServo(-90, 90, t1, 0);  // Move from -45° to +45° over 2 seconds
    delay(1000);
    cubicInterpolateServo(90, -90, t1, 0);  // Move back over 2 seconds
    delay(1000);
}

// Function to move servo1 to a given angle (0-180 degrees)
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




void cubicInterpolateServo(float startAngle, float endAngle, unsigned long durationMs, uint8_t channel) {
  float n = 3;
    const unsigned long stepDelay = 20; // milliseconds between steps
    const int steps = durationMs / stepDelay;

    // Interpolation coefficients for cubic (smooth start and stop)
    for (int i = 0; i <= steps; i++) {
        float t = (float)i / steps; // Normalize time [0,1]

        // Apply the power to t (adjust the interpolation speed)
        float t_pow = pow(t, n);  // t raised to the power of n
        
        // Cubic interpolation (Hermite basis: smooth start and stop)
        float h00 = 2 * t_pow * t_pow * t_pow - 3 * t_pow * t_pow + 1;
        float h01 = -2 * t_pow * t_pow * t_pow + 3 * t_pow * t_pow;

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
