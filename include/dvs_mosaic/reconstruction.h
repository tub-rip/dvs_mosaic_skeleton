#pragma once

#include <opencv2/core.hpp>


namespace poisson
{
  void reconstructBrightnessFromGradientMap(const cv::Mat& grad_map,
                                                  cv::Mat* map_reconstructed);
}
