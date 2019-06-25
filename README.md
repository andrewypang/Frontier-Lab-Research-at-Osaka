# OpenCR-CraneX7

  Upon my arrival at the Hosoda Laboratory in Osaka Univerity, Japan, I was introduced to a seven degrees of freedom robotic arm called the Crane-X7. However, new to the laboratory is a new microcontroller called the OpenCR. My task in the 10 weeks at Osaka University, is to establish a connection between the two. Previous students have developed the arm on a separate system, the NVIDIA Jetson TX2. The Jetson is a supercomputer that runs on Ubuntu 16.04. While it is more than capable to power the robotic arm, it is physically too big, its software requirements are too strict, and requires too many adapters[Fig 1].The physical cord connection is greatly simplified when considering the OpenCR[Fig 2].
  
  I have attempted to make communication between the board and the arm using several methods including ROS, ROSSerial, and robotic simulators like gazebo. All turned out to be overly complicated for a simple task of creating a communication. Eventually, I settled on using the Dynamixel SDK library in the Arduino IDE.

Technology Used:
* OpenCR
* CRANE-X7
* Arduino IDE
* Dynamixel SDK Library


## Videos
