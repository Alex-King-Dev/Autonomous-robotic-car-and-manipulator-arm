# Autonomous Robotic Vehicle with Manipulator Arm

Built for my Mechatronics module (Year 2, University of Sheffield) as part of a 4-person team. My focus was the navigation firmware and arm control logic which included sensor integration, motor control, and the arm pen drawing routine.

![Robot dimensions](media/4_5cm.png)

## What it does

The robot runs a fixed 18-state sequence to navigate a course: spins to position itself, follows walls using ultrasonic distance readings with an encoder-based proportional correction to stay straight (as one motor was off sync with the other), detects a coloured zone with a TCS3200 colour sensor, then stops at a whiteboard and uses the 2-DOF arm to draw a pre-programmed path with a pen before lifting it clear.

## Hardware

- Arduino Mega
- 2x FIT0450 DC gear motors with built-in encoders (drive wheels)
- MG996R servo (shoulder joint) + SG90 servo (elbow joint) for the arm
- HC-SR04 ultrasonic sensor (wall/obstacle distance)
- TCS3200 colour sensor (zone detection)
- MDF chassis, ball casters (front/back), 6x 1700mAh battery pack, breadboards for prototyping wiring

## Repo structure
