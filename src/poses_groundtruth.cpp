#include <dvs_mosaic/mosaic.h>
#include <fstream>
#include <glog/logging.h>


namespace dvs_mosaic
{

/**
* \brief Load poses from file (ground truth for prototyping the mapping part)
*/
void Mosaic::loadPoses()
{
  std::ifstream input_file;
  // FILL IN ... set the appropriate path to the file
  input_file.open("/home/ggb/ros/catkin_ws_evis/src/dvs_mosaicing/data/synth1/poses.txt");

  // Open file to read data
  if (input_file.is_open())
  {
    VLOG(2) << "Control poses file opened";

    int count = 0;
    std::string line;
    while( getline(input_file, line) )
    {
      std::istringstream stm(line);

      long sec, nsec;
      double x, y, z, qx, qy, qz, qw;
      Transformation T0;
      if (stm >> sec >> nsec >> x >> y >> x >> qx >> qy >> qz >> qw)
      {
        ros::Time(sec,nsec);
        const Eigen::Vector3d position(x,y,z);
        const Eigen::Quaterniond quat(qw,qx,qy,qz);
        Transformation T(position, quat);
        poses_.insert( std::pair<ros::Time, Transformation>(ros::Time(sec,nsec), T) );
      }
      count++;
    }
    VLOG(2) << "count poses = " << count;

    input_file.close();
  }

  // Remove offset: pre-multiply by the inverse of the first pose so that
  // the first rotation becomes the identity (and events project in the middle of the mosaic)

  // FILL IN... get the first control pose
  //Transformation T0 = ... ;
  Transformation T0;

  size_t control_pose_idx = 0u;
  for(auto it : poses_)
  {
    VLOG(3) << "--Control pose #" << control_pose_idx++ << ". time = " << it.first;
    VLOG(3) << "--T = ";
    VLOG(3) << it.second;
    poses_[it.first] = (T0.inverse()) * it.second;
  }

  control_pose_idx = 0u;
  for(auto it : poses_)
  {
    VLOG(3) << "--Control pose #" << control_pose_idx++ << ". time = " << it.first;
    VLOG(3) << "---------------------T normalized = ";
    VLOG(3) << it.second;
  }
}

}
