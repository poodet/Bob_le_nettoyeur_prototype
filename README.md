# Bob_le_nettoyeur : a table cleaning robot for pandemic times ! 

This project allows to control a cleaning robot from an Android application, or to let it operates autonomously.
This robot is able to move in a space while he is cleaning a table.
It has been developed by Guillaume BERNARD, Jules GRAEFF, Pauline ODET and Antoine PASSEMARD, robotics student in the CPE school, in Lyon.


There are two functionning modes:
- "control" mode = manual mode
- "auto" mode = automatic mode

A little ToF sensor in front of Bob able it to detect the end of the table to not fall out of the table. Some parts have been printed with a 3D printer, like the container for the hydroalcoolic liquid, and for the sensor support.

**The material and technologies used** :
- 3 Servomotors dynamixel AX-12 (2 for the wheels, 1 for the sponge)
- Proximity sensor ToF
- Esp32 T-display
- 5V Pump
- Raspberry Pi 2
- 12V Battery
- 2 wheels at the back, and 1 freewheel at the front

and

- ROS
- Arduino
- Bluetooth Low Energy
- An MIT App Inventor application
- 3D printed pieces





## User Manual

### **Starting** :

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

### **Setting up** :

The robot must be placed on a table, so that its ToF sensor is activated (it sees that the table is near). The Raspberry must be powered by the 12V battery, and connected to the WiFi (automatically done when started). The pump must be triggered by putting hydroalcoolic liquid in the pipes, otherwise it won't be able to spray some liquid.

Open the MIT app inventor application with an android smartphone. Make sure the bluetooth is enabled on your smartphone. Use the "scan Bluetooth" button to make your smartphone scan available bluetooth devices around. Click "Select and Connect" to connect to the BLE of Bob (named "`BOB_LE_NETTOYEUR`").

### **Play with bob** :

Once connected, you can either choose the control mode or the auto mode. 

- Clicking on "Control mode" will lead you to another screen, where you can click on several buttons to play with Bob's features : make it go forward or backward, left or right, activate the pump to spray some hydroalcoolic liquid or bring down/up the sponge.

- Clicking on "Auto mode" leads you to an almost blank screen. Bob is working on its own, you don't need to do anything! Every 2 second, Bob stops to spray liquid for 2 seconds. Then, it go forward for 2 seconds more. When the void is detected (Bob could fall !), it goes backward and makes a U-turn. Then, it continues to clean the table until detecting void again.

Here is a [link to a youtube video] demonstrating Bob's functionning.

## Have fun with Bob and don't forget to wear a mask !

 [link to a youtube video]: <https://www.youtube.com/watch?v=yAvRzTUq35Y&fbclid=IwAR3IyhD5BtqVDrqxux6i4YPPpNvEkhpMLkPUr5UTj8BFtKCsn-m_kRZWYwU&ab_channel=GuillaumeBernard>






