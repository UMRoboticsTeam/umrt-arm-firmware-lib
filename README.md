# UMRT Arm Firmware Library

This library offers functionality common to multiple of the projects supporting the University of Manitoba Robotics Team's robotic arm.

This includes:
* Communication between the arm's high-level and low-level computer
* A small application for testing the communication link
  * [CommunicationTest.h](include/CommunicationTest.h)
* An abstraction for controlling the arm's stepper motors
  * [StepperController.h](include/StepperController.h)