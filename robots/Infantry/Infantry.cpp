#include "base_robot/BaseRobot.h"

class Infantry : public BaseRobot {
  public:
    Infantry() : BaseRobot(config_) {}

    // default config
    const Config config_ = Config();

    ~Infantry() {}

    void init() override {}

    void periodic(unsigned long dt_us) override {}

    void end_of_loop() override {}

    // unsigned int main_loop_dt_ms() override {}
};

int main() {
    Infantry infantry;

    // blocking
    infantry.main_loop();
}