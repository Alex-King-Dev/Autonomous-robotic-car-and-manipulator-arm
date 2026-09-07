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

/firmware - MainCode.ino (Arduino sketch)
/docs - final report, tender design drawing
/media - dimensions photo, demo video

## How it works

`loop()` runs a state machine (see the `case` statements), each state is one leg of the course: turn, drive to a wall, correct heading, detect the coloured zone, stop, then hand off to the arm. Wheel correction is a simple proportional controller (`Kp`) comparing left/right encoder-derived angles to keep the robot driving straight between wall checks. The arm doesn't use inverse kinematics,  `thetaS1`/`thetaS2` are pre-computed waypoint arrays it steps through at a fixed interval (`waypointInterval`) to trace the drawing path, with `limitServoRate()` capping how fast each joint can move per cycle.

## Building it yourself

1. Wire per the component list in `/docs` (report Table 1/2 + Fig. 1 for pin layout).
2. Run `/firmware/MainCode.ino` via Arduino IDE (Mega target board).
3. Re-tune the ultrasonic thresholds (`firstWallThreshold`, etc.) and `Kp` for your own course dimensions, these were calibrated to our specific test track.

## What I'd change

The waypoint-array approach for the arm works but is brittle, any change to the course geometry means re-recording every point. Next time I'd implement actual inverse kinematics so the arm target could just be an (x, y) coordinate.
