#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

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

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base"));

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
