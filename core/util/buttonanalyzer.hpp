

class buttonanalyzer {
    private: 
        bool curVal = 0;
        bool lastVal = 0; 
        bool lastValInitial = 0;
        bool lastValRelease = 0;
        bool toggleVal = 0;


    public: 
    buttonanalyzer() {}

    void update(bool val) {
        curVal = val;

        if (curVal == 1 && lastVal == 0)
            toggleVal = !toggleVal;

        lastVal = curVal;
    }
    
    bool getStatus() {
        return curVal;
    }

    bool getToggle() {
        return toggleVal;
    }

    bool getInitialPress() {
        if (curVal == 1 && lastValInitial == 0) {
            lastValInitial = curVal;
            return 1;
        }
        lastValInitial = curVal;
        return 0;
    }

    bool getInitialRelease() {
        if (curVal == 0 && lastValRelease == 1) {
            lastValRelease = curVal;
            return 1;
        }
        lastValRelease = curVal;
        return 0;
    }



};