# Sujet_4__Bob_le_nettoyeur

This project allows to control a robot from an Android application.
This robot is able to move in a space while he is cleaning a table.

There is two modes:
- "control" mode = manual mode
- "auto" mode = automatic mode

A little sensor is allowing to detect the end of the table to not fall out of the table.


## User Manual

First you need to be sure to have ROS kinetic installed and "dynamixel_motor" git repo pulled.


```bash
roscore
```

```bash
roslaunch bob_le_nettoyeur control_all_launch.launch
```

The command used for debugging (reassigned the enslavement) the servo is : 

```bash
rosrun dynamixel_driver set_servo_config.py 3 --cw-angle-limit=0 --ccw-angle-limit=0
```

The first parameter "3" is the servomotor id, then in the second and third parameters CW means clockwise and CCW means counterclockwise, it makes sense if we want the motor to fully rotate, then both CW and CCW limits should be 0.

ligne 59 du code master -> changer le port par défaut tty ACM0


