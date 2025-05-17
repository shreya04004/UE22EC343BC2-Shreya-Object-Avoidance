# Object Avoidance Robot using Ultrasonic Sensor and Servo Motor

This project demonstrates a simple object-avoidance mechanism using an Arduino Uno, ultrasonic sensor, and a servo motor. The system detects nearby obstacles and responds by turning the servo motor to redirect away from the object.

## Hardware Components Used

- Arduino Uno  
- HC-SR04 Ultrasonic Sensor  
- SG90 Servo Motor  
- Jumper Wires  
- Breadboard  
- USB Cable  

## Project Workflow

1. The ultrasonic sensor continuously measures the distance to the nearest object.  
2. When an obstacle is detected within a threshold (15 cm), the servo motor rotates 180 degrees away from the obstacle.  
3. This simulates an object-avoidance action by changing direction.  
4. Sensor data and servo angle are printed over the serial monitor for debugging or integration with visualization tools like RViz.  

## Features

- Real-time distance measurement using HC-SR04  
- Servo-based avoidance mechanism  
- Simple and effective control logic  
- Serial output format for further integration (e.g., RViz or ROS 2)  

## Future Improvements

- Full integration with RViz/ROS 2 for simulated feedback  
- Add movement control (DC motors or wheels)  
- Improve dynamic response with PID or fuzzy logic  
- Support multiple obstacle directions  
