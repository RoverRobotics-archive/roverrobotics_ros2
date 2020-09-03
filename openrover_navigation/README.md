# OpenRover Nav2 (Under Development)

## How to get the code

Ideally, all dependencies would be released and available in the [ROS Package Index](https://index.ros.org/packages/), but that's not currently the case. Instead, some packages need to be built from source. The index to the source code is available in the file `openrover.repos` [openrover.repos](openrover.repos).

## How to build the openrover_navigation

You must repeat this on both the robot and the workstation:

| command                                                      | what it does                                                 |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| `source /opt/ros/dashing/setup.sh`                           | Activates the underlay `/opt/ros/dashing` so that your active shell knows to search in for executables, libraries, headers, python packages, and build hooks. |
| `mkdir -p ros2_ws/src`                                       | Ensures the directory `ros2_ws/src` exists and creates it if it doesn't. `ros2_ws` is the root of our workspace and source code will reside in the `src` subfolder. |
| `cd ros2_ws`                                                 | Change directories to the workspace. All `colcon` operations assume that your current directory is the workspace root. |
| `wget https://gist.githubusercontent.com/rotu/0f29b7df4eb6134d4df3f0dce6b38f7e/raw/` | Download the file `openrover.repos`. This contains a list of folders and the source code repositories they should be retrieved from. Ideally, all dependencies would be released and available in binary form in the [ROS Package Index](https://index.ros.org/packages/), but that's not currently the case. |
| `vcs import src < openrover.repos`                      | Import (i.e. clone) all the repos specified in `openrover.repos` into your workspace's source code subfolder. |
| `rosdep install --from-paths src --ignore-src --default-yes` | Search the packages in the source code subfolder for `package.xml` files. It goes through all the package dependencies, ignoring the ones that will be provided by other source code packages there, and queries the ROS Package Index for how best to satisfy the requirement (e.g. `pip install`, `apt install`, etc.). Then it installs those packages, without prompting for each one. |
| `colcon build --merge-install`                               | Build all packages from your workspace into the `build` subfolder and put the resulting built packages into the `install` subfolder. This also creates the workspace script `setup.sh`. The `--merge-install` argument is optional - it makes the install subfolder have a flatter structure and makes the environment variables resulting from the workspace setup script simpler. |

Once you have built the workspace, if you want to use any of its packages from the shell with `ros2`, you must first activate the workspace with `source ros2_ws/install/setup.sh`.

## How to map the space

Except for launching `drive.launch.py`, all below commands are to be run on your workstation. Also don't forget to activate the workspace first!

| command              | what it does |
| -------------------- | ------------ |
| `# on the robot`<br />`ros2 launch openrover_navigation drive.launch.py` | Kicks off ROS2 nodes to interface with the **OpenRover**, the LIDAR, and the IMU. |
| `ros2 launch openrover_navigation rviz.launch.py frame:=odom` | Opens **RViz**, a GUI program for visualizing all kinds of data coming out of ROS. Most useful will be the robot's location, the raw LIDAR data being fed into the SLAM algorithm, and the map as it is generated. At this point, you will not see a map, but you may see red dots, each of which represents a bit of raw LIDAR data. |
| `ros2 launch openrover_navigation slam.launch.py`                  | Kicks off the **Cartographer** ROS SLAM node. This will take in all the LIDAR scan data and stitch it together into a map. It will also publish the relation between that map and the robot's local position. At this point, you should start seeing a fragment of the map in RViz. Note that killing this node will cause you to lose the map, so don't forget to save the map as below! |
| `ros2 launch openrover_navigation teleop.launch.py` | Your robot probably can't see the whole room from where it is, so this launches an interactive process to command the rover with the keyboard as it explores the room.<br />Spacebar stops the robot, up/down arrows change the forward speed, left/right arrows change the angular speed. Go slow and take your time: Turning too fast can cause smearing artifacts on the map. Driving forward too fast can bruise your coworkers' shins and break things.<br />Also be aware that some obstacles may not be visible to the robot - transparent, reflective, or dark objects may not reflect the laser well enough, and anything below the LIDAR's plane of sight will also not show up on the map. |
| `ros2 run nav2_map_server map_saver` | When you have adequately explored the space and your map looks good, this commits the map to disk as a `map.pgm` and `map.yaml` file. |

The generated map serve two purposes: It can be used to determine where the robot is in the mapped space, and can be used as an obstacle map for the robot to plan paths.

## Planning and navigation

All the below instructions expect that you've built openrover_navigation (on both the robot and your workspace) and activated the workspace and its underlays in your terminal with: `source install/setup.sh`

1. On your robot: `ros2 launch openrover_navigation drive.launch.py`
2. On your workstation in one window: `ros2 launch openrover_navigation nav2.launch.py`
3. On your workstation in a second window, open RViz with: `ros2 launch openrover_navigation rviz.launch.py`
4. In RViz, use the **2D Pose Estimate** tool to specify a seed pose for AMCL.
5. In RViz, use the **2D Nav Goal** tool to specify a destination pose for AMCL



