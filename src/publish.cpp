#include <dvs_mosaic/mosaic.h>
#include <dvs_mosaic/image_util.h>
#include <geometry_msgs/PoseStamped.h>
#include <dvs_mosaic/reconstruction.h>
#include <glog/logging.h>


namespace dvs_mosaic
{

/**
* \brief Publish several variables related to the mapping (mosaicing) part
*/
void Mosaic::publishMap()
{
  // Publish the current map state
  VLOG(1) << "publishMap()";

  if ( time_map_pub_.getNumSubscribers() > 0 )
  {
    // Time map. Fill content in appropriate range [0,255] and publish
    // Happening at the camera's image plane
    cv_bridge::CvImage cv_image_time;
    cv_image_time.header.stamp = ros::Time::now();
    cv_image_time.encoding = "mono8";
    image_util::normalize(time_map_, cv_image_time.image, 15.);
    time_map_pub_.publish(cv_image_time.toImageMsg());
  }

  // Various mosaic-related topics
  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = ros::Time::now();
  cv_image.encoding = "mono8";
  if ( mosaic_pub_.getNumSubscribers() > 0 )
  {
    // Brightness image. Fill content in appropriate range [0,255] and publish
    // Call Poisson solver and publish on mosaic_pub_
    // FILL IN ...
    // Hints: call image_util::normalize discarding 1% of pixels

  }

  if ( mosaic_gradx_pub_.getNumSubscribers() > 0 ||
       mosaic_grady_pub_.getNumSubscribers() > 0 )
  {
    // Visualize gradient map (x and y components)
    // FILL IN ...
    // Hints: use cv::split to split a multi-channel array into its channels (images)
    //        call image_util::normalize

  }

  if ( mosaic_tracecov_pub_.getNumSubscribers() > 0 )
  {
    // Visualize confidence: trace of the covariance of the gradient map
    // FILL IN ...
    // Hints: use cv::split to split a multi-channel array into its channels (images)
    //        call image_util::normalize
  }
}


/**
* \brief Publish pose once the tracker has estimated it
*/
void Mosaic::publishPose()
{
  if (pose_pub_.getNumSubscribers() <= 0)
    return;

  VLOG(1) << "publishPose()";
  geometry_msgs::PoseStamped pose_msg;
  // FILL IN ... when tracking part is implemented

  pose_pub_.publish(pose_msg);
}

}
