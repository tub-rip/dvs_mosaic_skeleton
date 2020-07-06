#include <ros/ros.h>
#include <dvs_mosaic/mosaic.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

int main(int argc, char* argv[])
{
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "dvs_mosaic");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  dvs_mosaic::Mosaic mosaic_estimator(nh, nh_private);

  ros::spin();

  return 0;
}
