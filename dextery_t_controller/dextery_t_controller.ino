#include "Wire.h"

// Variables for analog input readings and communication
int x;           
int y;           
int z;          
int upperThreshold = 900;
int lowerThreshold = 100;
bool gripperOpen = false;
int leftButton = 8;
int rightButton = 7;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Configure input pins for joystick analog readings
  pinMode(A0, INPUT); 
  pinMode(A3, INPUT); 
  pinMode(A2, INPUT); 
  pinMode(leftButton, INPUT_PULLUP);
  pinMode(rightButton, INPUT_PULLUP);
}

void loop() {

  z = analogRead(A2);      
  x = analogRead(A0);         
  y = analogRead(A3);  

  if(digitalRead(leftButton) == LOW)
    gripperOpen = true;
  else if(digitalRead(rightButton) == LOW)
    gripperOpen = false;
  
  if(x > upperThreshold)
    Serial.println("-x");
  else if(x < lowerThreshold)
    Serial.println("+x");
  else if(y > upperThreshold)
    Serial.println("-y");
  else if(y < lowerThreshold)
    Serial.println("+y");
  else if(z > upperThreshold)
    Serial.println("-z");
  else if(z < lowerThreshold)
    Serial.println("+z");
  else { 
    if(gripperOpen)
      Serial.println("idle,o");
    else
      Serial.println("idle,c"); 
  }
  
  // Delay for a short period to avoid spamming transmissions
  delay(50);
  
}


