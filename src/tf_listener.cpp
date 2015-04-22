#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_rgbd1 frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped rgbd1_point;
  rgbd1_point.header.frame_id = "base_rgbd1";

  //we'll just use the most recent transform available for our simple example
  rgbd1_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  rgbd1_point.point.x = 1.0;
  rgbd1_point.point.y = 0.2;
  rgbd1_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;

    listener.transformPoint("base_link", rgbd1_point, base_point);

    ROS_INFO("base_rgbd1: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        rgbd1_point.point.x, rgbd1_point.point.y, rgbd1_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_rgbd1\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
