#include <ros/ros.h>
#include "pointcloud_roi/detected_roi_filter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_roi");

  ros::NodeHandle nhp("~");

  pointcloud_roi::DetectedRoiFilter filter(nhp);

  ros::spin();
}
