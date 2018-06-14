/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include "../include/qtCreateTest/qnode.hpp"
#include <turtlesim/Pose.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtCreateTest {

/*****************************************************************************
** Implementation
*****************************************************************************/

void poseMessageRecieved(const turtlesim::Pose& messg) {
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed
    << "position=(" << messg.x << "," << messg.y << ")"
    << " direction=" << messg.theta);

}

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"qtCreateTest");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
	chatter_sub = n.subscribe("turtle1/pose", 1000, &poseMessageRecieved);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"qtCreateTest");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
	chatter_sub = n.subscribe("turtle1/pose", 1000, &poseMessageRecieved);
	start();
	return true;
}
void QNode::setVel(const int setVel, const int value) {
  if (setVel == 1){
    velocity_ = value;
  }
  else {
    velocity_ = 0;
  }
//velocity_ = val;
}





void QNode::run() {
	ros::Rate loop_rate(2);
	int count = 0;
	while ( ros::ok() ) {

		geometry_msgs::Twist msg;
		std::stringstream ss;
		std::stringstream lin_ss;
		std::stringstream ang_ss;
                msg.linear.x = velocity_; //double(rand())/double(RAND_MAX);
                msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;
		chatter_publisher.publish(msg);
		lin_ss << msg.linear.x;
		// chatter_sub.subscribe("turtle1/pose", 1000, &poseMessageRecieved);
		ang_ss << msg.angular.z;
	//	std::string lin_string = to_string(msg.linear.x);
		log(Info,std::string("Sending random velocity command: linear = "
		+ lin_ss.str() + " angular = " +ang_ss.str()));

		// to_string(msg.linear.x)));
			// ROS_INFO_STREAM("Sending random velocity command:"
			// 	<< " linear = " << msg.linear.x
			// 	<< " angular = " << msg.angular.z);

		ros::spinOnce();
		loop_rate.sleep();
		// ++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace qtCreateTest
