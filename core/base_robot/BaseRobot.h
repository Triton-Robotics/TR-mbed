#pragma once

class BaseRobot {
  public:
    BaseRobot() = default;
    virtual ~BaseRobot() = default;

    virtual void init() = 0;
    virtual void periodic(const unsigned long dt_us) = 0;
    virtual void end_of_loop() {};

    // default 1000hz main loop. Can be overriden
    virtual unsigned int main_loop_dt_ms() { return 1; };

    virtual void main_loop();
};