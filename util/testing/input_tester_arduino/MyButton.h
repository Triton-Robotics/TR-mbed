class MyButton {
  private:
    // Define variables
    int pin;
    int buttonReturn = 0;
    int toggleState = 0;
    int previousState1 = 0;
    int previousState2 = 1;

    unsigned long intialPress;
    
  public:
    MyButton(int pin) {
      this->pin = pin;
      pinMode(pin, INPUT_PULLUP);
    }

    int updateButton() {
      if (digitalRead(pin) == 0 && buttonReturn == 1) {
        toggleState = !toggleState;
        intialPress = millis();
        buttonReturn = 0;
      }

      if (digitalRead(pin) == 1)
        buttonReturn = 1;

      return !digitalRead(pin);
    }

    int getToggle() {
      if (toggleState == 1)
        return 1;
      else
        return 0;
    }

    int getInitialPress() {
      if (digitalRead(pin) == 0 && previousState1 == 1) {
        previousState1 = digitalRead(pin);
        return 1;
      }
      previousState1 = digitalRead(pin);
      return 0;
    }

    int getInitialRelease() {
      if (digitalRead(pin) == 1 && previousState2 == 0) {
        previousState2 = digitalRead(pin);
        return 1;
      }
      previousState2 = digitalRead(pin);
      return 0;
    }

    int getTimePressed() {
      updateButton();
      if (getInitialRelease() == 1) {
        return millis() - intialPress;
      }
      else
        return -1;
    }

};
