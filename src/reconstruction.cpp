#include <dvs_mosaic/poisolve/laplace.h>
#include <dvs_mosaic/reconstruction.h>

#include <boost/multi_array.hpp>
#include <glog/logging.h>


namespace poisson
{

void reconstructBrightnessFromGradientMap(const cv::Mat& grad_map,
                                                cv::Mat* map_reconstructed)
{
  CHECK_EQ(grad_map.type(), CV_32FC2);
  CHECK_GT(grad_map.cols, 0);
  CHECK_GT(grad_map.rows, 0);
  const cv::Size img_size = grad_map.size();

  // Compute the right hand side of Poisson eq.
  // F = dgx/dx + dgy/dy and put it into a boost::multi_array
  const size_t height = img_size.height;
  const size_t width = img_size.width;
  boost::multi_array<double,2> M(boost::extents[height][width]);
  boost::multi_array<double,2> F(boost::extents[height][width]);

  // Compute right hand side (rhs) using one-sided finite differences
  for(size_t i=0; i < height-1; ++i)
  {
    for(size_t j=0; j < width-1; ++j)
    {
      F[i][j] = double( grad_map.at<cv::Vec2f>(i,j+1)[0] - grad_map.at<cv::Vec2f>(i,j)[0]
                      + grad_map.at<cv::Vec2f>(i+1,j)[1] - grad_map.at<cv::Vec2f>(i,j)[1] );
    }
  }
  F[height-1][width-1] = 0.0;

  // Solve for M_xx + M_yy = F using Poisson solver,
  // with constant intensity (zero) boundary conditions
  const double gradient_on_boundary = 0.0;
  pde::poisolve(M, F, 1.0, 1.0, 1.0, 1.0,
                gradient_on_boundary,
                pde::types::boundary::Neumann,false);

  // Fill in output variable
  *map_reconstructed = cv::Mat(img_size, CV_32FC1);
  // FILL IN ...

}

}
