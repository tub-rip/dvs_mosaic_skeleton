#include <dvs_mosaic/image_util.h>

namespace image_util
{

/**
* \brief Compute robust min and max values (statistics) of an image
* \note Requires sorting
*/
void minMaxLocRobust(const cv::Mat& image, float& rmin, float& rmax,
                     const float& percentage_pixels_to_discard)
{
  cv::Mat image_as_row = image.reshape(0,1);
  cv::Mat image_as_row_sorted;
  cv::sort(image_as_row, image_as_row_sorted, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
  image_as_row_sorted.convertTo(image_as_row_sorted, CV_32FC1);
  const int single_row_idx_min = (0.5f*percentage_pixels_to_discard/100.f)*image.total();
  const int single_row_idx_max = (1.f - 0.5f*percentage_pixels_to_discard/100.f)*image.total();
  rmin = image_as_row_sorted.at<float>(single_row_idx_min);
  rmax = image_as_row_sorted.at<float>(single_row_idx_max);
}


/**
* \brief Normalize image to the range [0,255] using robust min and max values
*/
void normalize(const cv::Mat& src, cv::Mat& dst, const float& percentage_pixels_to_discard)
{
  float rmin_val, rmax_val;
  minMaxLocRobust(src, rmin_val, rmax_val, percentage_pixels_to_discard);
  const float scale = ((rmax_val != rmin_val) ? 255.f / (rmax_val - rmin_val) : 1.f);
  cv::Mat state_image_normalized = scale * (src - rmin_val);
  state_image_normalized.convertTo(dst, CV_8UC1);
}

} // namespace
