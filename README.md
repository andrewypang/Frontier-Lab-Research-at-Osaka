# Frontier Lab Research at Osaka, Japan

Upon my arrival at the Hosoda Laboratory in Osaka Univerity, Japan, I was introduced to a seven-degrees-of-freedom robotic arm called the Crane-X7. However, the current system to power and run the robotic arm was physically too big, its software requirements too strict, and requires too many adapters. My Goal at Hosoda Laboratory was to explore new ways to reduce the current system(The NVIDIA Jetson TX2) requirements for something more lightweight and portable, and create new features for future implementations. In the end, I was able to provide an alternative way to power and run the Crane-X7 by swapping the more desktop orientated Jetson TX2 with a more portable, more cost effective, all-in-one microcontroller called the OpenCR.

Technology Used:
* OpenCR
* CRANE-X7
* Arduino IDE
* Dynamixel SDK Library

## The CRANE-X7 by RT CORPORATION.
![Crane-X7](img/AndrewPang-FrontierSummerLab2018-CraneX7.jpeg)

### The Goal:

Replace the NVIDIA Jetson TX2 with OpenCR:
![Jetson TX2](img/AndrewPang-FrontierSummerLab2018-JetsonTX2.jpeg)

![OpenCR](img/AndrewPang-FrontierSummerLab2018-OpenCR.jpeg)

Thus reducing the overall system from this:

![CraneX7 to Jetson](img/AndrewPang-FrontierSummerLab2018-CraneX7&Jetson-SETUP.jpeg)

to this:

![CraneX7 to OpenCR](img/AndrewPang-FrontierSummerLab2018-CraneX7&OpenCR-SETUP.jpeg)


OpenCR also comes with many built-in features like IMU, buttons, and LEDs.
I built a fun program that demonstrates the new features made possible thanks to the OpenCR. The IMU allowed me to control the arm, the buttons allowed me to open/close the gripper, and the on-board LEDs displayed important notifications.

<!-- ![Karaage in Cup](https://youtu.be/N_jZu9cy7EY) -->

[Karaage in Cup](https://youtu.be/N_jZu9cy7EY)

[Pouring a Drink](https://youtu.be/JTcfXIpT0VU)
<!-- ![Pouring a Drink](img/AndrewPang-FrontierSummerLab2018-IMU-Drink-Demo.mov) -->

<!-- ![XYZ Axis Demo](img/AndrewPang-FrontierSummerLab2018-IMU-XYZ-Axis-Demo.mov) -->
[XYZ Axis Demo](https://youtu.be/GwmOkOydiBY)

How XYZ Axis is defined:

![XYZ Axis Def](img/AndrewPang-FrontierSummerLab2018-OpenCR-IMU.jpg)

### Here are some technical stuff about the program:

The Flow Chart:

![Flow Chart](img/IMU-Progran-FlowChart.jpg)

I limited my program to two joints to simplify the gripper kinematics but it severely restricted its movement range. Expanding it beyond two joints can be a future addition.

One joint:

![One Joint Movement](img/OneJointMovement.jpeg)
![One Joint Movement Gif](img/OneJointMovement.gif)

Two joint:

![Two Joint Movement](img/TwoJointMovement.jpeg)
![Two Joint Movement Gif](img/TwoJointMovement.gif)


### Here is the full process from start to end:

![Full Process](img/AndrewPang-FrontierSummerLab2018-Full-Process-480p.mov)

