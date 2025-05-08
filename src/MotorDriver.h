#pragma once
#include "Motor.h"

/**
 *  Dual‑H‑bridge driver (two DC motors).
 */
class MotorDriver {
public:
    MotorDriver(uint8_t m1Dir, uint8_t m1Pwm,
                uint8_t m2Dir, uint8_t m2Pwm,
                bool invert1 = false,
                bool invert2 = false)
    : m1(m1Dir, m1Pwm, invert1),
      m2(m2Dir, m2Pwm, invert2) {}

    void begin() { 
        m1.begin();  
        m2.begin(); 
    }

    void setMotor1Speed(int s) { 
        m1.setSpeed(s);
    }

    void setMotor2Speed(int s) { 
        m2.setSpeed(s); 
    }

    void stopAll()  { 
        m1.stop();
        m2.stop();  
    }

    void brakeAll() { 
        m1.brake();
        m2.brake(); 
    }

private:
    Motor m1, m2;
};
