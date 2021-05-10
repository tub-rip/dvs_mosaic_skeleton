#include <dvs_mosaic/mosaic.h>
#include <glog/logging.h>


namespace dvs_mosaic
{

/**
* \brief Process each event to refine the mosaic variables (mean and covariance)
*/
void Mosaic::processEventForMap(const dvs_msgs::Event& ev,
  const double t_ev, const double t_prev,
  const cv::Matx33d& Rot, const cv::Matx33d& Rot_prev)
{
  const double dt_ev = t_ev - t_prev;
  CHECK_GT(dt_ev,0) << "Non-positive dt_ev"; // Two events at same pixel with same timestamp

  // FILL in ... lots of gaps. This function is mostly empty

  // Get map point corresponding to current event
  // hint: call project_EquirectangularProjection
  cv::Point3d rotated_bvec;
  cv::Point2f pm;

  // Get map point corresponding to previous event at same pixel
  cv::Point3d rotated_bvec_prev;
  cv::Point2f pm_prev;

  // Get approx optical flow vector (vector v in the paper)
  cv::Point2f flow_vec;

  // Extended Kalman Filter (EKF) for the intensity gradient map.
  // Get gradient and covariance at current map point pm
  cv::Matx21f gm;
  cv::Matx22f Pg;

  // Compute innovation, measurement matrix and Kalman gain
  float nu_innovation;
  cv::Matx21f Kalman_gain;

  // Update gradient (state) and covariance
  gm += Kalman_gain * nu_innovation;
  // Pg -= ...

  // Store updated values on corresponding pixel of grad_map_ and grad_map_covar_

}

}
