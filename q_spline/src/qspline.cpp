#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ecl/geometry.hpp>

ros::Publisher splinePathPub; // Declaring a publisher for the interpolated spline

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  // Extracting waypoints from the received path message
  std::vector<geometry_msgs::Pose> waypoints;
  for (const auto& poseStamped : msg->poses)
  {
    waypoints.push_back(poseStamped.pose);
  }

  // Separating x and y coordinates from the waypoints
  std::vector<double> x_coords;
  std::vector<double> y_coords;
  for (const auto& pose : waypoints)
  {
    x_coords.push_back(pose.position.x);
    y_coords.push_back(pose.position.y);
  }

  // Interpolating a smooth linear spline using the x and y coordinates
  double max_curvature = 5.0;  //going to be changed according to the result
  ecl::SmoothLinearSpline spline(x_coords, y_coords, max_curvature);

  // Creating a new Path message for the interpolated spline
  nav_msgs::Path splinePath;
  splinePath.header = msg->header; // Copy the header information from the received path

  // Populating the Path message with waypoints along the spline
  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header = msg->header;
    poseStamped.pose.position.x = x_coords[i];
    poseStamped.pose.position.y = y_coords[i];

    splinePath.poses.push_back(poseStamped);
  }

  // Publish the interpolated spline for visualization in RViz
  splinePathPub.publish(splinePath);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_smoother");
  ros::NodeHandle nh;

  // Publisher for the interpolated spline
  splinePathPub = nh.advertise<nav_msgs::Path>("/spline_path", 1); // Advertising the topic for publishing

  ros::Subscriber pathSub = nh.subscribe("/move_base/GlobalPlanner/plan", 1, pathCallback); // Subscribing to the original path

  ros::spin();

  return 0;
}
