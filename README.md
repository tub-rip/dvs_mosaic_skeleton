# dvs_mosaic

Partial re-implementation of the paper  [Simultaneous Mosaicing and Tracking with an Event Camera](http://www.bmva.org/bmvc/2014/papers/paper066/), by Kim et al., BMVC 2014.

Le us start with the "mapping" part, that is, creating the brightness mosaic (by estimating the spatial gradients and then integrating them by [Poisson image reconstruction](https://en.wikipedia.org/wiki/Gradient-domain_image_processing)). Locations to be filled with code are marked with `FILL IN`.


## Dependencies

### Create a catkin workspace

Create a catkin workspace (if there is none yet). For example, from your home folder:

	cd
	mkdir -p catkin_ws/src
	cd catkin_ws
	catkin config --init --mkdirs --extend /opt/ros/melodic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
	
Depending on the ROS distribution you installed, you might have to use `kinetic` instead of `melodic` in the previous command.

### Add packages to the catkin workspace

Clone this repository into the `src` folder of your catkin workspace.

The catkin package dependencies are:
- ROS messages for DVS ([rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros))
- [catkin simple](https://github.com/catkin/catkin_simple)
- [Google Logging Library (glog)](https://github.com/catkin/catkin_simple.git)
- [Gflags (formerly Google Commandline Flags)](https://github.com/ethz-asl/gflags_catkin)
- [minkindr](https://github.com/ethz-asl/minkindr) (for dealing with poses). minkindr depends on [eigen_catkin](https://github.com/ethz-asl/eigen_catkin.git) and [eigen_checks](https://github.com/ethz-asl/eigen_checks.git), which are therefore also required.

The above dependencies are specified in the [dependencies.yaml](dependencies.yaml) file. They can be installed with the following commands from the `src` folder of your catkin workspace:

	cd catkin_ws/src
	sudo apt-get install python3-vcstool
	vcs-import < dvs_mosaic/dependencies.yaml

The previous command should clone the the repositories into folders *catkin_simple*, *rpg_dvs_ros*, *glog_catkin*, *gflags_catkin*, *minkindr*, *eigen_checks*, *eigen_catkin* inside the src/ folder of your catking workspace, at the same level as this repository *dvs_mosaic*. They should NOT be inside the *dvs_mosaic* folder.

Additional ROS tools needed (specified in the [package.xml](package.xml) file):

	sudo apt-get install ros-melodic-image-geometry
	sudo apt-get install ros-melodic-tf-conversions

The Poisson solver used is from [this page](https://kluge.in-chemnitz.de/opensource/poisson_pde/). It is already copied in this repository. It requires the [FFTW library](https://launchpad.net/ubuntu/+source/fftw3):

	sudo apt install libfftw3-dev

## Compiling

Assuming your terminal is in the `src` folder of your catkin workspace:

	catkin build dvs_mosaic
	source ../devel/setup.bash

## Running the code

Edits before running the code:
- Move the camera calibration file to your `~/.ros/camera_info/` folder. Create the folder if it does not previously exist.
- In the launch file, edit the path to the data file (events.bag ROS bag) that is provided in the folder called `data`.
- In `poses_groundtruth.cpp`, edit the path to the data file with the ground truth poses.

In a terminal:

	roslaunch dvs_mosaic synth.launch
	
The verbosity level (>=0) for printing and debugging purposes using glog is controlled by means of the parameters passed to the node `args="--v level"` (see the example launch file).

In another terminal, run 

	rqt_image_view

and listen to the image topics published. At the begininng, you might not see anything, since the code to set up the publishers needs to be written. Alternatively, use the provided rqt perspective file in the launch folder. Run the following command in a terminal, from within the folder of this repository:

	rqt --perspective-file launch/mosaic_view.perspective

End the program execution with `Ctrl + C` keyboard shortcut.

## References

Based on this [project](https://github.com/uzh-rpg/rpg_image_reconstruction_from_events) and the references therein.
