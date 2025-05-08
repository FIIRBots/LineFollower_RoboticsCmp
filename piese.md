# Line-Following Robot for 2024 Robochallenge

This project is a line-following robot, developed using an Arduino Portenta H7. The robot uses PID (Proportional-Integral-Derivative) control to adjust its motors based on feedback from a line sensor array. This implementation includes a web interface for tuning PID values and other parameters in real-time.

## Features

- **PID Control**: The robot uses PID to adjust motor speeds for smooth and precise line-following.
- **Web Interface**: Allows real-time tuning of key PID parameters and sensor weights via a web page hosted on the Arduino Portenta H7.
- **Sensor Array**: Utilizes a 16-sensor array for accurate line detection.
- **Motor Control**: Powered by a DRV8835 Motor Driver and custom motor pins.

## Components

### Hardware
1. **[Arduino Portenta H7](https://store.arduino.cc/products/portenta-h7?srsltid=AfmBOooMavyPkp-detYT58EfPobycQ1quM_hkinavoQe7r5-D1K5LYQy)** - Microcontroller used to process sensor data and control the motors.
2. **[Pololu DRV8835 Motor Driver](https://www.pololu.com/product/2135)** - Dual motor driver for controlling two motors independently.
3. ***[Hyperline Robotics 16 Sensors Array](https://hyperlinerobotics.com/products/16-sensors-array.html)** - High-precision sensor array for line detection.
4. **[Pololu DC Motors](https://www.pololu.com/product/999)** - Pololu 10:1 Micro Metal Gearmotor HP 6V.
5. **[Hyperline Robotics Impeller](https://hyperlinerobotics.com/products/impeller.html)** - high-performance downforce solution designed for robotics and other lightweight applications. Offering approximately 800 grams of thrust
6. **[Battery](https://hpi-racing.ro/li-po-2s-74v/acumulator-lipo-gens-ace-g-tech-soaring-450mah-74v-30c-2s1p-cu-jst-syp.html)** - Lipo Gens Ace Acumulator - G-Tech Soaring - 450mAh - 7.4V - 30C - 2S1P with JST-SYP
7. **Miscellaneous** - Jumper wires, connectors, screws, PCB and other hardware for assembly.
8.5. **Chassis and Mounts** - 3D-printed parts (see 3D Model section) for housing the sensors, motors, and microcontroller.

### Software Libraries
- **DRV8835MotorShield** - For motor control.
- **WiFi** - For web interface and real-time parameter adjustments.

## Circuit Diagram

Connect the components as follows:
- **DRV8835 Motor Driver**: Connect M1DIR, M1PWM, M2DIR, M2PWM pins to custom GPIO pins 19, 18, 16, and 17 respectively.
- **Sensor Array**: Connect each of the 13 sensors to GPIO pins as defined in the code.
- **Portenta H7 Power and Ground**: Ensure a stable power source is connected to the ESP32 and motors.

## 3D Model ToDo

The chassis and sensor mounts are designed to be 3D-printed for robustness and optimal sensor positioning. The STL files for 3D printing can be found in the `3d-models` folder of this repository.
