# Ultrasonic Scanner Package

This package has code to use multiple sonar sonar sensors and display them as a laser scan.

**NOTE:** A laser scan is not the correct message for this but it is what we used. Use with caution.

## Nodes

There are two ROS nodes in this package, serial_scanner and od_scanner.
* **serial_scanner:** use this package if data is coming in over a serial line from some microcontroller such as an arduino or trinket
  * Dependencies: [Serial](http://wiki.ros.org/serial) ROS package from wjwwood. Install with `sudo apt-get install ros-kinetic-serial`
* **od_scanner:** use this package if data is coming directly to the odroid's GPIO pins. This package requires the use of an analog multiplexer because of the limited ADCs on the Odroid C2.
  * Dependencies: wiringPi for odroid

## Serial Permissions

You need to add yourself to the `dialout` group in order for you to be able to access the serial ports with this code. This can be done with the following code:

`sudo usermod -a -G dialout magicole`

Assuming `magicole` is your username. You will need to log out and log back in in order for your account to have access to the serial ports.

A temporary work around if you don't want to add yourself to the dialout group is to run `sudo chmod 666 /dev/ttyUSB0` where `/dev/ttyUSB0` is the name of the serial device you are connecting to. This change will be erased whenever you unplug the serial device though.
