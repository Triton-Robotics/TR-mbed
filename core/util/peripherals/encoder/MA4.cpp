#include "MA4.h"

MA4::MA4(PinName p) : _p(p) {
    _p.rise(callback(this, &MA4::rise));
    _p.fall(callback(this, &MA4::fall));
    _period = 0.0;
    _pulsewidth = 0.0;
    _t.start();
}

float MA4::period() {
    return _period;
}

float MA4::pulsewidth() {
    return _pulsewidth;
}

float MA4::dutycycle() {
    if (_period == 0.0f) {
        return 0.0f;  // Return 0 until first valid PWM cycle
    }
    return _pulsewidth / _period;
}

void MA4::rise() {
    _period = chrono::duration_cast<chrono::microseconds>(_t.elapsed_time()).count() / 1000.0f;
    _t.reset();
}

void MA4::fall() {
    _pulsewidth = chrono::duration_cast<chrono::microseconds>(_t.elapsed_time()).count() / 1000.0f;
}

double MA4::getEncoderYawPosition() {
    static float filtered_yaw = 0.0f;
    float filter_alpha = 0.2f;  // 0.0-1.0: lower = more smoothing, higher = more responsive

    float duty_raw = dutycycle();
    float duty_min = 0.02943f;   // 2.943%
    float duty_max = 0.97058f;   // 97.058%

    //low pass filter 
    double yaw_position = (double)(abs(((duty_raw - duty_min) / (duty_max - duty_min)) * 360.0));
    filtered_yaw = filtered_yaw * (1.0f - filter_alpha) + yaw_position *  filter_alpha;
    // printf("%.2f\n",yaw_position);
    return filtered_yaw;
}

double MA4::encoderMovingAverage() {
    const int windowSize = 20;
    static double readings[windowSize] = {0};
    static int index = 0;
    static bool filled = false;

    double newReading = getEncoderYawPosition();
    if (newReading < 0) {
        return -1.0f; // Encoder not available
    }

    readings[index] = newReading;
    index = (index + 1) % windowSize;
    if (index == 0) {
        filled = true;
    }

    double sum = 0.0;
    double result = 0.0;

    if (filled) {
        for (int i = 0; i < windowSize; i++) {
            sum += readings[i];
        }
        result = sum / windowSize;
    } else {
        for (int i = 0; i < index; i++) {
            sum += readings[i];
        }
        result = sum / index;
    }
    // printf("%.2f\n",result);
    return result;

}