// This code is used to prevent the turtle from running into the walls

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

ros::Publisher *pubPtr;

void commandVelocityRecieved(
  const geometry_msgs::Twist& msgIn
) {
  geometry_msgs::Twist msgOut;
  msgOut.linear.x = msgIn.linear.x;
  msgOut.angular.z = msgIn.angular.z;
  pubPtr->publish(msgOut);
}

void currentPose(
  const turtlesim::Pose& poseIn
) {
  float xLoc = poseIn.x;
  float yLoc = poseIn.y;
  ROS_INFO_STREAM("Current Pose Updated to x: " << xLoc << " y: " << yLoc);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "avoid_edges");
  ros::NodeHandle nh;

  pubPtr = new ros::Publisher(
    nh.advertise<geometry_msgs::Twist>(
      "turtle1/cmd_vel_avoid_edges",1000));

  ros::Subscriber sub_vel = nh.subscribe(
    "turtle1/cmd_vel", 1000, &commandVelocityRecieved);

  ros::Subscriber sub_pos = nh.subscribe(
    "turtle1/pose", 1000, &currentPose);


  ros::spin();

  delete pubPtr;
}
