# balancing_robot_V1
Arduino code for a prototype balancing robot V1 using an ESP32 microcontroller

Folders:
* self_balancing_robot_V1: contains the code for the balance mode of the robot
* **UPDATED**self_balancing_robot_V1_bc: contains the code for the bluetooth control mode of the robot
* calibracionMPU6050:  contains a sketch to calibrate the IMU sensor (MPU6050)
* ... 


The code is divided in three sections (three sketches):
1. Main program: run continuously (self_balancing_robot_V1.ino) 
2. Setup configuration: initialisation/declaration process in form of individual functions (Setup_Configuration.ino)
3. User Functions: modular pieces of code that perform a defined task (User_Functions.ino)

For the balancing robot, I used:
1. ESP32 devkit V1
2. Motor Driver - Dual TB6612FNG (1A)
3. Step-Down Converter MP1584 (3A, 1.5MHz, 28V)
4. Micro gear DC Motor 150:1, Kit (Motor + Wheel + Clamp)
5. 2 potentiometers
6. Lipo batery 3s 500mAh
7. Other components (resistors,transistors,led,diode,jumper dupont)
8. laser cutted and 3d printed parts.

Here you can take a look to the componets

(Fusion360, Solidworks, Inventor) IGES individual files:

https://drive.google.com/drive/folders/1MScjUHHrM-ga2XxOfZKyjb-pYAPPDgPk?usp=sharing

PDF files and other resources:

https://drive.google.com/drive/folders/1MksbtBJ40beVEJUTdcA26Ne1emJZtYYA?usp=sharing
