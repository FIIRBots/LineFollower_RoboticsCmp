/*
    * Copyright (C) 2025 by Dan-Dominic Staicu
    * 331CA UNSTPB ACS CTI
*/
#pragma once
#include <Arduino.h>

/**
 *  One half‑bridge (one motor) – works with DRV8833
 *  dirPin  – logic HIGH = forward   (unless invertDir==true)
 *  pwmPin  – any timer‑backed PWM pin
 */
class Motor {
public:
    explicit Motor(uint8_t dirPin,
                   uint8_t pwmPin,
                   bool    invertDir = false)
    : dirPin_(dirPin),
      pwmPin_(pwmPin),
      inv_(invertDir) {}

    void begin() {
        pinMode(dirPin_, OUTPUT);
        pinMode(pwmPin_, OUTPUT);
        stop();
    }

    /** speedPercent  −100 … +100  (values outside are clipped) */
    void setSpeed(int speedPercent) {
        speedPercent = constrain(speedPercent, -100, 100);
        lastSpeed_   = speedPercent;

        const int speed = map(abs(speedPercent), 0, 100, 0, 255);
        const bool fwd = (speedPercent < 0);

        int pwm = 0;
        if (fwd) {
          pwm = 255 - speed;
        } else {
          pwm = speed;
        }

        digitalWrite(dirPin_, (fwd ^ inv_) ? LOW : HIGH);
        analogWrite (pwmPin_, pwm);
    }

    /** coast to stop (high‑Z on both FETs) */
    void stop() { 
        analogWrite(pwmPin_, 0); 
    }

    /** fast stop (both low‑side FETs ON) – HR8833 only */
    void brake() {
        digitalWrite(dirPin_, LOW ^ inv_);
        analogWrite(pwmPin_, 255);
    }

    int lastCommand() const { 
        return lastSpeed_; 
    }

private:
    uint8_t dirPin_, pwmPin_;
    bool    inv_;
    int     lastSpeed_ = 0;
};
