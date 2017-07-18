#include <string>
#include <vector>
#include <sstream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "serial/serial.h"

using namespace std;

vector<string> string_split(string str, char delimiter){
  vector<string> tokens;
  stringstream stream(str);
  string token;
  while(getline(stream, token, delimiter)){
    tokens.push_back(token);
  }
  return tokens;
}

sensor_msgs::LaserScan generate_scan(vector<string> data){
  float sensor0_dist, sensor1_dist, sensor2_dist = 0;
  try{
    sensor0_dist = stof(data[0]);
    sensor1_dist = stof(data[1]);
    sensor2_dist = stof(data[2]);
  }catch(const exception& e){
    cout << "Exception Thrown: " << e.what() << endl;
  }
	ros::Time scan_time = ros::Time::now();
	sensor_msgs::LaserScan scan;
	scan.header.stamp = scan_time;
	scan.header.frame_id = "laser";
	scan.angle_min = -1.57;
	scan.angle_max = 1.57;
	scan.angle_increment = 1.57;
	scan.time_increment = 0.1;
	scan.scan_time = 0.3;
	scan.range_min = 0;
	scan.range_max = 7.65;
	scan.ranges.resize(3);
	scan.ranges[0] = sensor0_dist;
	scan.ranges[1] = sensor1_dist;
	scan.ranges[2] = sensor2_dist;
	return scan;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "ultrasonic_scanner_serial");
  ros::NodeHandle nh;
  ros::Publisher scan_publisher = nh.advertise<sensor_msgs::LaserScan>("ultrasonic_scan", 10);
  ROS_INFO("Ultrasonic Node Started");

  // ifstream arduino;
  // char data[20];
  // arduino.open("/dev/ttyUSB0", ios::in);

  string port("/dev/ttyUSB0");
  unsigned long baud = 9600;
  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  if(my_serial.isOpen())
    ROS_INFO("Serial Port Open");
  else{
    ROS_INFO("Serial Port NOT Open");
    return 0;
  }
  ros::Rate loop_rate(15);
  string data;

  while(ros::ok()){
    if(my_serial.available() > 13){
      //cout << "I have stuff: " << endl;
      data = my_serial.readline();
      vector<string> tokens = string_split(data, ',');
      //cout << "I got " << tokens.size() << " items from splitting" << endl;
      if(tokens.size() == 3){
        scan_publisher.publish(generate_scan(tokens));
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
