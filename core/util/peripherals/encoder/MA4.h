
#include "mbed.h"
#ifndef MA4_H
#define MA4_H

/** PwmIn class to read PWM inputs
 * 
 * Uses InterruptIn to measure the changes on the input
 * and record the time they occur
 *
 * @note uses InterruptIn, so not available on p19/p20
 */
class MA4 {
public:
    /** Create a PwmIn
     *
     * @param p The pwm input pin (must support InterruptIn)
     */ 
    MA4(PinName p);
    
    /** Read the current period
     *
     * @returns the period in seconds
     */
    float period();
    
    /** Read the current pulsewidth
     *
     * @returns the pulsewidth in seconds
     */
    float pulsewidth();
    
    /** Read the current dutycycle
     *
     * @returns the dutycycle as a percentage, represented between 0.0-1.0
     */
    float dutycycle();

    /**
     * Gets the yaw position from encoder (PWM) input in degrees (0-360)
     * @return yaw position in degrees, or -1 if encoder not available
     */
    double getEncoderYawPosition();

    /**
     * A helper method to calculate the moving average of the encoder readings for yaw position
     * @return the moving average of the encoder readings for yaw position
     */
    double encoderMovingAverage();
    

protected:        
    void rise();
    void fall();
    
    InterruptIn _p;
    Timer _t;
    float _pulsewidth, _period;
};

#endif
