#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include <serial/serial.h>
#include <iostream>
#include "tf/transform_datatypes.h"
/*
 * PD controller tuning constants
 */
double kp1 = 0.8;
double kp2 = 0.8;
double kp3 = 0.8;
double kp_yaw = 0.8;

double kd1 = 0.5;
double kd2 = 0.5;
double kd3 = 0.5;
double kd_yaw = 0.5;

/*
 * variable needed to calculate velocity from slamdunk pose
 */
double r1 = 0;
double r2 = 0;
double r3 = 0;
double yaw = 0;

double r1_prev = 0;
double r2_prev = 0;
double r3_prev = 0;
double yaw_prev = 0;

double v1 = 0;
double v2 = 0;
double v3 = 0;

double v1_prev = 0;
double v2_prev = 0;
double v3_prev = 0;

double a1 = 0;
double a2 = 0;
double a3 = 0;

//3D point target
double rt1 = 0;
double rt2 = 0;
double rt3 = 1;
double yaw_t = 0;

double vt1 = 0;
double vt2 = 0;
double vt3 = 0;


double ep1 = 0;
double ep2 = 0;
double ep3 = 0;
double eyaw = 0;

double ep1_prev = 0;
double ep2_prev = 0;
double ep3_prev = 0;
double eyaw_prev = 0;

double ed1 = 0;
double ed2 = 0;
double ed3 = 0;
double edyaw = 0;

double ades1 = 0;
double ades2 = 0;
double ades3 = 0;
double ades_yaw = 0;

double seconds = 0;
double seconds_prev = 0;
double dt = 0;

double phi = 0;
double theta = 0;
double u1 = 0;

//output to serial for transmission to drone
double controlScale = 500;
int autopitch = 1500;
int autoroll = 1500;
int autoyaw = 1500;

std_msgs::Float64MultiArray rc_out;
std_msgs::String rc_out1;

//serial 
serial::Serial ser;


void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	r1 = msg->pose.pose.position.x;
	r2 = msg->pose.pose.position.y;
	r3 = msg->pose.pose.position.z;
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("yaw: %f",yaw);

	//v1 = msg->twist.twist.linear.x;
	//v2 = msg->twist.twist.linear.y;
	//v3 = msg->twist.twist.linear.z;
	// seconds = msg->header.stamp.sec + ((double)msg->header.stamp.nsec)/1000000000;
    // ROS_INFO("[%f]", seconds);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "outerloop");
	ros::NodeHandle n;

	std::string port;
	n.getParam("/outerloop/port",port);

	n.getParam("/outerloop/target_x",rt1);
	n.getParam("/outerloop/target_y",rt2);
	n.getParam("/outerloop/target_z",rt3);

	n.getParam("/outerloop/kp_p",kp1);
	n.getParam("/outerloop/kd_p",kd1);

	n.getParam("/outerloop/kp_r",kp2);
	n.getParam("/outerloop/kd_r",kd2);

	n.getParam("/outerloop/kp_y",kp3);
	n.getParam("/outerloop/kd_y",kd3);


	std::string quad_name = argv[1];
	std::string quad_topic = "/" + quad_name + "/odom";
	ros::Subscriber sub = n.subscribe(quad_topic, 1000, chatterCallback);
	ros::Publisher read_pub = n.advertise<std_msgs::Float64MultiArray>("write", 1000);

	ROS_INFO("INITIALIZED PARAMETERS: \n [QuadTopic: %s] \n [Port: %s] \n [PD: (Pitch,Roll,Yaw) ==> (%f,%f) (%f,%f) (%f,%f)]",quad_name.c_str(),port.c_str(),kp1,kd1,kp2,kd2,kp3,kd3);
    //initiate serial port
	try
	{
        // ELKA (Vikram lab flight controller) boud rate = 38400
		ser.setPort(port);
		ser.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}
	catch (serial::IOException& e)
	{
		std::cout << e.what() << std::endl;
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}

	if(ser.isOpen()){
		ROS_INFO_STREAM("Serial Port initialized");
	}else{
		return -1;
	}

	ros::Rate loop_rate(30);
	while(ros::ok()){
		ros::spinOnce();
        //ROS_INFO("Position: [%f,%f,%f]", r1, r2, r3);

        //calculate delta time
        seconds = (double)ros::Time::now().toSec();
		dt = seconds - seconds_prev;
		if(dt!=0) {
            //ROS_INFO("[%f]", dt);

            //calculate velocity
            // v1 = (r1 - r1_prev) / dt;
            // v2 = (r2 - r2_prev) / dt;
            // v3 = (r3 - r3_prev) / dt;
            //ROS_INFO("Velocity: [%f,%f,%f]", v1, v2, v3);
            //a1 = (v1 - v1_prev) /dt;
            //a2 = (v1 - v1_prev) /dt;
            //a3 = (v1 - v1_prev) /dt;
            //ROS_INFO("Acceleration: [%f,%f,%f]", a1, a2, a3);

            //calculate errors and desired inputs
			ep1 = rt1 - r1;
			ep2 = rt2 - r2;
			ep3 = rt3 - r3;
			eyaw = yaw_t - yaw; 

			ed1 = (ep1-ep1_prev)/dt;
			ed2 = (ep2-ep2_prev)/dt;
			ed3 = (ep3-ep3_prev)/dt;
			edyaw = (eyaw - eyaw_prev)/dt;



            ades1 = kp1*ep1 + kd1*ed1 + 0;  //trajectory acceleration set to 0
            ades2 = kp2*ep2 + kd2*ed2 + 0;
            ades3 = kp3*ep3 + kd3*ed3 + 0;
            ades_yaw = kp_yaw*eyaw + kd_yaw*edyaw + 0;

            phi = 1/9.8*(-1*ades2);
            theta = 1/9.8*(ades1);
            u1 = ades3;
            ROS_INFO("Phi,Theta,u1: [%f,%f,%f]", phi, theta, u1);

            rc_out.data.clear();
            rc_out.data.push_back(phi);
            rc_out.data.push_back(theta);
            rc_out.data.push_back(u1);
            // read_pub.publish(rc_out);
            autopitch = (int)((double)(theta)*controlScale+1500);
            autoroll  = (int)((double)(phi)*controlScale+1500);
            autoyaw = (int)((double)ades_yaw*controlScale+1500);

            rc_out1.data.clear();
            rc_out1.data.push_back((uint8_t)(autoroll/256));
            rc_out1.data.push_back((uint8_t)(autoroll%256));
            rc_out1.data.push_back((uint8_t)(autopitch/256));
            rc_out1.data.push_back((uint8_t)(autopitch%256));
            rc_out1.data.push_back((uint8_t)(autoyaw%256));
            rc_out1.data.push_back((uint8_t)(autoyaw%256));

            ser.write(rc_out1.data);

            // //update previous records
            // r1_prev = r1;
            // r2_prev = r2;
            // r3_prev = r3;
            // v1_prev = v1;
            // v2_prev = v2;
            // v3_prev = v3;
            ep1_prev = ep1;
            ep2_prev = ep2;
            ep3_prev = ep3;
            eyaw_prev = eyaw;
            
            seconds_prev = seconds;




        }
        loop_rate.sleep();

    }


    return 0;
}

