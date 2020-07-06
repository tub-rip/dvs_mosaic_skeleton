# dvs_mosaic

Partial re-implementation of the paper  [Simultaneous Mosaicing and Tracking with an Event Camera](http://www.bmva.org/bmvc/2014/papers/paper066/), by Kim et al., BMVC 2014.

Le us start with the "mapping" part, that is, creating the brightness mosaic (by estimating the spatial gradients and then integrating them by [Poisson image reconstruction](https://en.wikipedia.org/wiki/Gradient-domain_image_processing)). Locations to be filled with code are marked with `FILL IN`.


## Dependencies

The following catkin package dependencies are specified in the `dependencies.yaml` file. They can be installed with the following commands:

	sudo apt-get install python3-vcstool
	vcs-import < dvs_mosaic/dependencies.yaml

The dependencies are:
- ROS messages for DVS ([rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros))
- [catkin simple](https://github.com/catkin/catkin_simple)
- [Google Logging Library (glog)](https://github.com/catkin/catkin_simple.git)
- [Gflags (formerly Google Commandline Flags)](https://github.com/ethz-asl/gflags_catkin)
- [minkindr](https://github.com/ethz-asl/minkindr) and tf_conversions (for dealing with poses). minkindr depends on [eigen_catkin](https://github.com/ethz-asl/eigen_catkin.git) and [eigen_checks](https://github.com/ethz-asl/eigen_checks.git), as you will notice when you install it.
- [FFTW library](https://launchpad.net/ubuntu/+source/fftw3) (for Poisson integration)


The Poisson solver is from [this page](https://kluge.in-chemnitz.de/opensource/poisson_pde/).

## Compiling

	catkin build dvs_mosaic
	source devel/setup.bash

## Running the code

In a terminal:

	roslaunch dvs_mosaic synth.launch
	
Make sure that the launch file has the correct path to the data (e.g., ROS bag) and that the provided `DVS-synthetic.yaml` file with the camera calibration is in your `~/.ros/camera_info/` folder.

The verbosity level (>=0) for printing and debugging purposes using glog is controlled by means of the parameters passed to the node `args="--v level"` (see the example launch file).

In another terminal, run `rqt_image_view` and listen to the image topics published.

End the program execution with `Ctrl + C` keyboard shortcut.

## References

Based on this [project](https://github.com/uzh-rpg/rpg_image_reconstruction_from_events) and the references therein.
