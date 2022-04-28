#include "MyButton.h"

MyButton smallButton = MyButton(7);
MyButton greenButton = MyButton(6);
MyButton redButton = MyButton(4);
MyButton blueButton = MyButton(5);
MyButton yellowButton = MyButton(3);

int potBottom = A3;
int pottopRight = A2;
int pottopMiddle = A1;
int pottopLeft = A0;

void setup()
{
  Serial.begin(1000000);
  pinMode(potBottom, INPUT_PULLUP);
  pinMode(pottopLeft, INPUT_PULLUP);
  pinMode(pottopMiddle, INPUT_PULLUP);
  pinMode(pottopRight, INPUT_PULLUP);
}

int reversePotVal(int val) {
  return 1023-val;
}

void loop() 
{
  Serial.print(reversePotVal(analogRead(potBottom)));
  Serial.print("|");
  Serial.print(reversePotVal(analogRead(pottopLeft)));
  Serial.print("|");
  Serial.print(reversePotVal(analogRead(pottopMiddle)));
  Serial.print("|");
  Serial.print(reversePotVal(analogRead(pottopRight)));
  Serial.print("|");
  
  Serial.print(smallButton.updateButton());
  Serial.print("|");
  Serial.print(yellowButton.updateButton());
  Serial.print("|");
  Serial.print(redButton.updateButton());
  Serial.print("|");
  Serial.print(blueButton.updateButton());
  Serial.print("|");
  Serial.print(greenButton.updateButton());
  Serial.print("|");

  delay(75);
  //Serial.println(analogRead(potBottom));
  //delay(analogRead(potBottom));
}
