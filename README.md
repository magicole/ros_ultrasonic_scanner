# Ultrasonic Scanner Package

This package has code to use multiple sonar sonar sensors and display them as a laser scan.

**NOTE:** A laser scan is not the correct message for this but it is what we used. Use with caution

## Nodes

There are two ROS nodes in this package, serial_scanner and od_scanner.
* **serial_scanner:** use this package if data is coming in over a serial line from some microcontroller such as an arduino or trinket
  * Dependencies: [Serial](http://wiki.ros.org/serial) ROS package from wjwwood. Install with `sudo apt-get install ros-kinetic-serial`
* **od_scanner:** use this package if data is coming directly to the odroid's GPIO pins. This package requires the use of an analog multiplexer because of the limited ADCs on the Odroid C2.
  * Dependencies: wiringPi for odroid 
