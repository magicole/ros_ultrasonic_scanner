#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include  <wiringPi.h>

#define PORT_ADC1 0
#define S0 23
#define S1 24
#define trigger 22

int init_scanner(){
	pinMode(S0, OUTPUT);
	pinMode(S1, OUTPUT);
	pinMode(trigger, OUTPUT);
	digitalWrite(S0, LOW);
	digitalWrite(S1, LOW);
	digitalWrite(trigger, LOW);
	return 0;
}

int scan(int sensorNumber){
	switch(sensorNumber){
		case 0: 
			digitalWrite(S0, LOW);
			digitalWrite(S1, LOW);
			break;
		case 1: 
			digitalWrite(S0, HIGH);
			digitalWrite(S1, LOW);
			break;
		case 2:
			digitalWrite(S0, LOW);
			digitalWrite(S1, HIGH);
			break;
		case 3:
			digitalWrite(S0, HIGH);
			digitalWrite(S1, HIGH);
			break;
		default: 
			digitalWrite(S0, LOW);
			digitalWrite(S1, LOW);
			break;
	}
	// ros::Duration(0.001).sleep(); // this may not be necessary.
	return analogRead(PORT_ADC1);
}

float reading_to_m(int analog_value){
	float temp = analog_value * 0.001757; //The number of volts per 0-1024 scale
	temp = temp * 2; //multiply by two to counter the V-Divider to get into the 3.3v range
	temp = temp / 0.0032; //divide by 3.2mV/cm to get answer in cm
	temp = temp / 100;  //divide by 100 go get answer in meters
	return temp;
}

sensor_msgs::LaserScan generate_scan(){
	//bring trigger high
	digitalWrite(trigger, HIGH);
	//wait a bit, we only really need to wait for 20uS but 1ms should be fine
	ros::Duration(0.001).sleep();
	//bring the trigger low again so we can trigger it the next time around
	digitalWrite(trigger, LOW);	
	//delay for 65ish ms to allow for the sonar to make readings and equalize the analog value
	ros::Duration(0.065).sleep();	
	//read the values by scanning through the multiplexer
	int sensor0_raw = scan(0);
	int sensor1_raw = scan(1);
	int sensor2_raw = scan(2);
	float sensor0_dist = reading_to_m(sensor0_raw);
	float sensor1_dist = reading_to_m(sensor1_raw);
	float sensor2_dist = reading_to_m(sensor2_raw);

	
	//generate the ROS laser scan message
	ros::Time scan_time = ros::Time::now();
	sensor_msgs::LaserScan scan;
	scan.header.stamp = scan_time;
	scan.header.frame_id = "laser";
	scan.angle_min = -1.57;
	scan.angle_max = 1.57;
	scan.angle_increment = 1.57;
	scan.time_increment = 0.005;
	scan.scan_time = 0.1;
	scan.range_min = 0;
	scan.range_max = 7.65;
	scan.ranges.resize(3);
	scan.ranges[0] = sensor0_dist;
	scan.ranges[1] = sensor1_dist;
	scan.ranges[2] = sensor2_dist;
	return scan;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ultrasonic_scanner");
	ros::NodeHandle nh;
	ros::Publisher scan_publisher = nh.advertise<sensor_msgs::LaserScan>("ultrasonic_scan", 10);

	wiringPiSetup();
	if( init_scanner() != 0){
		ROS_ERROR("Something failed");
		return 0;
	}

	ROS_INFO("Ultrasonic Node Started");

	ros::Rate loop_rate(10); //cannot go over 10Hz because of the read rates of our sonars.
	while(ros::ok()){
		//doing something
		/*		
		int sensor0_raw = scan(0);
		int sensor1_raw = scan(1);
		int sensor2_raw = scan(2);
		float sensor0_dist = reading_to_m(sensor0_raw);
		float sensor1_dist = reading_to_m(sensor1_raw);
		float sensor2_dist = reading_to_m(sensor2_raw);
		ROS_INFO("Raw Values:\t%i\t\t%i\t\t%i", sensor0_raw, sensor1_raw, sensor2_raw);
		ROS_INFO("Calculated:\t%f\t%f\t%f\n", sensor0_dist, sensor1_dist, sensor2_dist);	
		*/
		scan_publisher.publish(generate_scan());
			
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
