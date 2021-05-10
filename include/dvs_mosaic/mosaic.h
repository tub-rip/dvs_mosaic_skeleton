#pragma once

#include <ros/ros.h>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <opencv2/core/core.hpp>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <deque>

#include <kindr/minimal/quat-transformation.h>
#include <image_geometry/pinhole_camera_model.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dvs_mosaic/dvs_mosaicConfig.h>


namespace dvs_mosaic
{

using Transformation = kindr::minimal::QuatTransformation;
//using Transformation = kindr::minimal::RotationQuaternion;

class Mosaic {
public:
  Mosaic(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  virtual ~Mosaic();

private:
  ros::NodeHandle nh_;   // Node handle used to subscribe to ROS topics
  ros::NodeHandle pnh_;  // Private node handle for reading parameters

  // Callback functions
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);

  // Subscribers
  ros::Subscriber event_sub_;

  // Publishers
  image_transport::Publisher mosaic_pub_;
  image_transport::Publisher time_map_pub_;
  image_transport::Publisher mosaic_gradx_pub_, mosaic_grady_pub_, mosaic_tracecov_pub_;
  ros::Publisher pose_pub_;
  void publishMap();
  void publishPose();
  ros::Time time_packet_;

  // Dynamic reconfigure
  void reconfigureCallback(dvs_mosaic::dvs_mosaicConfig &config, uint32_t level);
  boost::shared_ptr<dynamic_reconfigure::Server<dvs_mosaic::dvs_mosaicConfig> > server_;
  dynamic_reconfigure::Server<dvs_mosaic::dvs_mosaicConfig>::CallbackType dynamic_reconfigure_callback_;

  // Sliding window of events
  std::deque<dvs_msgs::Event> events_;
  std::vector<dvs_msgs::Event> events_subset_;

  // Camera
  int sensor_width_, sensor_height_;
  image_geometry::PinholeCameraModel dvs_cam_;
  cv::Mat time_map_;

  // Mosaic parameters
  int mosaic_width_, mosaic_height_;
  cv::Size mosaic_size_;
  float fx_, fy_; // speed-up equiareal projection

  // Measurement function
  bool measure_contrast_;
  float var_R_;
  float C_th_;

  // Mapping / mosaicing
  int num_events_map_update_;
  int idx_first_ev_map_;  // index of first event of processing window
  std::vector<cv::Matx33d> map_of_last_rotations_;
  cv::Mat grad_map_, grad_map_covar_, mosaic_img_;
  std::map<ros::Time, Transformation> poses_;
  void loadPoses();
  void processEventForMap(const dvs_msgs::Event& ev, const double t_ev,
    const double t_prev, const cv::Matx33d& Rot, const cv::Matx33d& Rot_prev);
  bool rotationAt(const ros::Time& t_query, cv::Matx33d& Rot_interp);
  void project_EquirectangularProjection(const cv::Point3d& pt_3d, cv::Point2f& pt_on_mosaic);

  // Precomputed bearing vectors for each camera pixel
  std::vector<cv::Point3d> precomputed_bearing_vectors_;
  void precomputeBearingVectors();
};

} // namespace
