#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>

using namespace std;

double x, y, th;
double x_old, y_old;
double vx, vy, vth;

ros::Time current_time, last_time;
ros::Publisher odom_pub;


void poseCallback(const geometry_msgs::PoseStamped::Ptr& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );

    x = msg->pose.position.x;
    y = msg->pose.position.y;

    tf::Quaternion q;
    q.setX(msg->pose.orientation.x);
    q.setY(msg->pose.orientation.y);
    q.setZ(msg->pose.orientation.z);
    q.setW(msg->pose.orientation.w);
    //q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);
    geometry_msgs::Quaternion q_msg;
    tf::quaternionTFToMsg(q, q_msg);

    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    vx = (x - x_old) / dt;
    vy = (y - y_old) / dt;
    vth = vth;

    x_old = x;
    y_old = y;

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = q_msg;

    //send the transform
    br.sendTransform(odom_trans);
    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = msg->pose.position.x;
    odom.pose.pose.position.y = msg->pose.position.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = q_msg;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_broadcaster");

    ros::NodeHandle n_;
    ros::Subscriber sub = n_.subscribe("/frame_registration_pose/camera1", 1, &poseCallback);
    odom_pub = n_.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Rate loop_rate(50);

    x = 0.0;
    y = 0.0;
    x_old = 0.0;
    y_old = 0.0;
    th = 0.0;

    vx = 0.0;
    vy = 0.0;
    vth = 0.0;

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while (n_.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
};


/*
using namespace std;

void poseCallback(const geometry_msgs::PoseStamped::Ptr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );

  tf::Quaternion q;
  q.setX(msg->pose.orientation.x);
  q.setY(msg->pose.orientation.y);
  q.setZ(msg->pose.orientation.z);
  q.setW(msg->pose.orientation.w);
  //q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/frame_registration_pose/camera1", 1, &poseCallback);

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
};
*/
