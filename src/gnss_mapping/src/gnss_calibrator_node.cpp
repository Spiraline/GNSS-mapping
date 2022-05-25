#include <ros/ros.h>

#include "gnss_calibrator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gnss_calibrator");
  gnss_calibrator::Nmea2TFPoseNode ntpn;
  ntpn.run();

  return 0;
}
