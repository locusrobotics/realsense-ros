^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Allow parametrizing list of resettable errors
  (cherry picked from commit 0f0aafbf1a91430a0a5a82d2f8208e222ea31e6e)
* Reorganize and add missing dependencies
  (cherry picked from commit 1f3bc5e85e14679667d06bdb154de5086c77d192)
* update version to 2.2.7
* Add support for SR305 (`#844 <https://github.com/locusrobotics/realsense-ros/issues/844>`_)
  fix bug in scaling depth.
* move ddynamic_reconfigure settings to before start streaming. (`#843 <https://github.com/locusrobotics/realsense-ros/issues/843>`_)
* publish imu_info (`#809 <https://github.com/locusrobotics/realsense-ros/issues/809>`_)
  * publish imu_info
  * update apt-key for ROS repos.
  * Add option to use external nodelet manager
  * Register dynamic options in parent node handle
  * Fix empty cmake variable and dependency on librealsense (`#747 <https://github.com/locusrobotics/realsense-ros/issues/747>`_)
  * add external_manager option to rs_camera.launch
  * clarify installation process.
  * publish gyro/imu_info and accel/imu_info only once.
  * cleaning
* fixed bug: advertise unsupported streams. (`#811 <https://github.com/locusrobotics/realsense-ros/issues/811>`_)
  * fixed bug: advertise unsupported streams.
  * constraints on accel_up test - loosen up a bit to compensate for different in frames arriving from file.
* cleaning
* Make T265 less noisy in the info log (`#823 <https://github.com/locusrobotics/realsense-ros/issues/823>`_)
* publish gyro/imu_info and accel/imu_info only once.
* add external_manager option to rs_camera.launch
* Fix empty cmake variable and dependency on librealsense (`#747 <https://github.com/locusrobotics/realsense-ros/issues/747>`_)
* Register dynamic options in parent node handle
  If running in an external nodelet manager this makes sure dynamic options are in the right namespace. This does not change existing behaviour.
* Add option to use external nodelet manager
  Right now a nodelet manager is always created when running the realsense
  node. This is not always a wanted behaviour as a nodelet manager might
  already be running somewhere else in the ROS system. This patch adds a
  parameter to disable launching a new nodelet manager. In this case the
  nodelet manager must be provided using the manager arg. The new
  parameter is set to false by default, so existing behaviour is not
  changed.
* add external_manager option to rs_camera.launch
* Fix empty cmake variable and dependency on librealsense (`#747 <https://github.com/locusrobotics/realsense-ros/issues/747>`_)
* publish imu_info
* rename librealsense2.rdmanifest to librealsense2_xenial.rdmanifest
  add file: librealsense2_bionic.rdmanifest
* disable static_tf_1 test - not working on Travis.
  Modified points_cloud_1 test - make more robust
* add librealsense2 dependency
* update version to 2.2.6
* add librealsense2.rdmanifest file
* fix remarks to imu test
* Add accel test
  * add rs_rtabmap.launch
  * Add test for accel in d435i. Needs recorded file: 20190527_D435i.bag
  * add d435i accel unit-test
* update version to 2.2.5
* exit node if failed to initialize.
* Migrate to https://github.com/pal-robotics/ddynamic_reconfigure
* fixed bug: wrong frame_id for imu frames. (`#784 <https://github.com/locusrobotics/realsense-ros/issues/784>`_)
* update version: 2.2.4
* add option: allow_no_texture_points
* add note to rs_rgbd.launch, reminding users to initially install ros package rgbd_launch.
* removed global variable _device, based on @akirayou at https://github.com/IntelRealSense/realsense-ros/issues/774#issuecomment-494236047
* fixed bug: imu and synced imu are now sent in original device coordinates frames - i.e. gyro_optical_frame, accel_optical_frame, imu_optical_frame. Fix issue for both t265 and d435i with different coordinate systems.
  fixed bug: sending united imu without images enabled.
  add imu_optical_frame_id to nodelet.launch.xml.
* camera_link for t265 is POSE instead of GYRO.
  fix is needed due to the availability of t265 extrinsics.
* fix inserted bug reading from file
* removed lock_guard.
  set_devices_changed_callback called AFTER getDevice()
  Keep checking for devices until device is found - for cases where T265 was momentarily taken by another node at the time of query.
  Add a 3rd, optional camera, to rs_multiple_devices.launch file.
* fix bug in pointcloud. Used to send points with Z=0.
  add feature: _allow_no_texture_points - if set to true, will send points with depth, both with and without texture.
* sync get devices
* add decimation filter at the front of the filter list, before the start of disparity filter
* Migrate to https://github.com/pal-robotics/ddynamic_reconfigure
* fix bug scaling depth. (`#717 <https://github.com/locusrobotics/realsense-ros/issues/717>`_)
* change frame_id for imu messages to camera_link's coordinates system, same as imu's sync messages.
* Add hole_filling filter.
  update version to 2.2.3
* update version to 2.2.2
  update README.md link to librealsense v2.19.2
* wheel_odometry (`#691 <https://github.com/locusrobotics/realsense-ros/issues/691>`_)
  * use wheel_Odometry
  Add parameters to launch files:
  * topic_odom_in - The topic on which wheel odometry arrives.
  * calib_odom_file - path to calibration.json file, of the librealsense format. i.e.: https://github.com/IntelRealSense/librealsense/blob/master/unit-tests/resources/calibration_odometry.json
* Register dynamic options in parent node handle
  If running in an external nodelet manager this makes sure dynamic options are in the right namespace. This does not change existing behaviour.
* Add option to use external nodelet manager
  Right now a nodelet manager is always created when running the realsense
  node. This is not always a wanted behaviour as a nodelet manager might
  already be running somewhere else in the ROS system. This patch adds a
  parameter to disable launching a new nodelet manager. In this case the
  nodelet manager must be provided using the manager arg. The new
  parameter is set to false by default, so existing behaviour is not
  changed.
* add flag publish_odom_tf (default to true)
* fix compilation bug with OMP (`#692 <https://github.com/locusrobotics/realsense-ros/issues/692>`_)
* updated references to realsense2_description
* Cast min, step, step to int in get_enum_method
  so we don't loop with float values.
* Fix validation check for enum options
  by finding the ROS (static) param in the enum dictionary, which can take
  values in the min:step:max range (not just 0 <= value < enum_dict.size()).
  Also remove the check when the option is taken from the sensor, which
  should always be correct.
* Transform to lower in create_graph_resource_name
* Use std::replace_if in create_graph_resource_name
  with equivalent to ros::names::isValidCharInName(char c)
* Set config defaults from ROS param server
  fixes `#609 <https://github.com/locusrobotics/realsense-ros/issues/609>`_
  Only if the ROS params are set
* Fixed distortion coefficients bug. (`#662 <https://github.com/locusrobotics/realsense-ros/issues/662>`_)
* Add dependency on nav_msgs (`#674 <https://github.com/locusrobotics/realsense-ros/issues/674>`_)
  thanks.
* fix Twist in odometry problem (`#676 <https://github.com/locusrobotics/realsense-ros/issues/676>`_)
* fix PR`#682 <https://github.com/locusrobotics/realsense-ros/issues/682>`_ (`#683 <https://github.com/locusrobotics/realsense-ros/issues/683>`_)
  * add example for checking the depth at the center of the image.
  * fix bug: did not fix depth scale for single frame.
* fix depth scale (`#682 <https://github.com/locusrobotics/realsense-ros/issues/682>`_)
  * fix depth scale to always follow ROS convention of 1mm
  * incorporates PR`#605 <https://github.com/locusrobotics/realsense-ros/issues/605>`_
* Adjust unit of SR300's depth image into 0.001 meter (same as D435's). This unit adjustment is needed for rgbd_launch package and it's point cloud value.
* update version - 2.2.1
* Add handling t265 coordinate system (`#657 <https://github.com/locusrobotics/realsense-ros/issues/657>`_)
  * fixed launch files (fisheye1,2)
  * renamed spatial_frame to odom_frame
  fixed dependency of librealsense to version 2.19.0
  Add t265_realsense_node.h, t265_realsense_node.cpp to handle the different coordinate system.
  Add demo_t265.launch file and t265.rviz
  send odom_frame tf even without someone registered to odom topic.
* renamed spatial_frame to odom_frame
  fixed dependency of librealsense to version 2.19.0
  Add t265_realsense_node.h, t265_realsense_node.cpp to handle the different coordinate system.
  Add demo_t265.launch file and t265.rviz
  send odom_frame tf even without someone registered to odom topic.
* :
  [Problem]
  [Solution]
  [Test]
  [Links]
  https://issues.labcollab.net/browse/
* fixed launch files (fisheye1,2)
* Fix version in package.xml (`#625 <https://github.com/locusrobotics/realsense-ros/issues/625>`_)
* Modified the CMake file so that URDF and mesh files will be installed (`#615 <https://github.com/locusrobotics/realsense-ros/issues/615>`_)
* Fix `#628 <https://github.com/locusrobotics/realsense-ros/issues/628>`_ - added guards around clang-specific pragmas (`#630 <https://github.com/locusrobotics/realsense-ros/issues/630>`_)
  Also added a guard around an OpenMP pragma
* fix rs_aligned_depth.launch
* increase rs2_test.py robustness for node failing to load.
* fix README.md and launch files.
* auto reset if need to.
* fix README.md and launch files.
* restore initial_reset option.
  Fix bug of locking tracking module (t265) by nodes that don't use it.
* modify behavior: if reconnect if camera disconnected.
  package.xml: upgrade package format
  removed initial_reset option - need to return.
* rename tm2 to t265
* fixed static_tf test in rs2_test and changed the name of vis_avg_1 to non_existent_file to reflect it's true purpose.
* delete topics of aligned depth to index 2 of other sensors. (`#644 <https://github.com/locusrobotics/realsense-ros/issues/644>`_)
  It is not implemented in librealsense and the topics that were published so far do not provide useful information were actually aligned to index 1.
* delete topics of aligned depth to index 2 of other sensors.
  It is not implemented in librealsense and the topics that were published so far do not provide useful information were actually aligned to index 1.
* rs_t265.launch: Add a disclaimer about wheel odometry
* renames and readme (`#629 <https://github.com/locusrobotics/realsense-ros/issues/629>`_)
  * fixed static_tf test in rs2_test and changed the name of vis_avg_1 to non_existent_file to reflect it's true purpose.
  * rename tm2 to t265
  * fix README.md
* check build with librealsense v2.18.1
* update version to 2.2.0
* use tf2 instead of tf for pose static transformation
* Fix pending messages variable name typo (`#608 <https://github.com/locusrobotics/realsense-ros/issues/608>`_)
* Replace spaces and hyphens in parameter names (`#617 <https://github.com/locusrobotics/realsense-ros/issues/617>`_)
* fix dependency between covariance values and confidence value.
  Added to README.md
* fix test. remove some log messages.
* fix frame_id for odom topic.
* TM265 - add odometry topic
  interface change: add parameter: enable_tm2 - cause the wrapper to wait on initialization while tm2 device sets its Unique USB ID
  use enable_gyro and enable_accel instead of enable_imu
  use infra_width, infra_fps instead of infra1_width, infra1_fps and infra2_width, infra2_fps
* add basic support for TM265. Fisheye, Gyro, Accel.
* code reorganization.
  fix bug of reinitializing align operator.
* add support for TM1 fisheye comes in RAW8 and Tm2's in Y8.
  moved enabling HID sensors to enable_devices()
* clean parameters reading.
* set base time on first message (image or imu originated)
  clean code.
* Remove gencfg dependency (`#581 <https://github.com/locusrobotics/realsense-ros/issues/581>`_)
  Now with ddynamic_reconfigure being the backend for dynamic reconfigurability, the ${PROJECT_NAME}_gencfg target doesn't exist anymore and this dependency can be removed.
* fix bug: "No stream match for pointcloud chosen texture" warning was meant to appear when unavailable texture is chosen. As it was, it appears every time a frame was dropped. (`#591 <https://github.com/locusrobotics/realsense-ros/issues/591>`_)
* Remove REQUIRED from find_package to show the correct error message (`#592 <https://github.com/locusrobotics/realsense-ros/issues/592>`_)
* Add filters argument to rs_rgbd.launch (`#593 <https://github.com/locusrobotics/realsense-ros/issues/593>`_)
* No depth required (`#601 <https://github.com/locusrobotics/realsense-ros/issues/601>`_)
  * add benchmark test for static_tf
  * enable running with depth disabled.
  rs2_test.py: Add message to results summery.
* fix bug: no default covariance for separate gyro and accel imu messages. (`#600 <https://github.com/locusrobotics/realsense-ros/issues/600>`_)
* update version to 2.1.4
* fix bug: update camera_info if image size changes. (`#587 <https://github.com/locusrobotics/realsense-ros/issues/587>`_)
* changed the default gyro_fps and accel_fps to match actual values (`#560 <https://github.com/locusrobotics/realsense-ros/issues/560>`_)
* add initial_reset to camera2 in rs_multiple_devices.launch
* fixed urdf.rviz to look nicer.
* fix transform between urdf and driver
* correctred .stl filename
* added realsense D415 urdf
* Fixed d435 collision position
* add bottom_screw joint to _d435.urdf.xacro
* add initial_reset option to rs_multiple_devices.launch
* fix bug in align depth to image. (`#572 <https://github.com/locusrobotics/realsense-ros/issues/572>`_)
  When publishFrame is called from publishAlignedDepthToOthers the format of the images is already set and is different from what is defined in _image_format for that stream type.
* close sensors when Ctrl-C signal is received. (`#571 <https://github.com/locusrobotics/realsense-ros/issues/571>`_)
  add test in makefile for librealsense version
* Fixed different transforms between xacro and driver
* update version number
* add linear interpolation union method for IMU (`#558 <https://github.com/locusrobotics/realsense-ros/issues/558>`_)
  Add linear interpolation method for union of IMU sensors. Thanks to Marius Fehr (@mfehr) for the idea.
  Set the initial behavior to sending IMU sensors separately, since this is the raw data. Enabling union with option unite_imu_method as demonstrated in the file opensource_tracking.launch.
  fix bug if initializing with unavailable imu profile.
* fix to work with librealsense v2.17.0 (`#555 <https://github.com/locusrobotics/realsense-ros/issues/555>`_)
  fixed to work with librealsense v2.17.0
* fix: wrong reference for the gmock dependency (`#546 <https://github.com/locusrobotics/realsense-ros/issues/546>`_)
  fix: typo on ddynamic_reconfigure
* Add notifications for hardware errors.
* add parameter "initial_reset" to reset the device on start up. Default is set to false.
* Fixed: invalid module name format for ROS (`#537 <https://github.com/locusrobotics/realsense-ros/issues/537>`_)
* use ddynamic_reconfigure and support D435i (`#535 <https://github.com/locusrobotics/realsense-ros/issues/535>`_)
  Add dynamic dynamic reconfigure. That means there are no longer differences in the code between D415, D430, SR300.
  Add dynamic options for filters
  Add support for camera D435i.
  Add clipping_disance option. enabled with parameter: clip_distance. units: meters. Default: no clipping.
  Add linear accel covariance - Default: 0.01
  Add option: unite_imu - send linear acceleration and radial velocity in the same Imu message. Default: True
  Add parameter: hold_back_imu_for_frames. If set to true, hold imu messages that arrived while manipulating frames, until frames are actually sent.
  Comply with librealsense v2.17.0
  Add opensource_tracking.launch - demo that runs realsense2_camera, imu_filter_madgwick, rtabmap and robot_localization to demonstrate Slam with realsense D435i
  Set accel_fps to 250 as this is the new maximal rate in librealsense v2.17.0
  * Add NOTICE file, to emphasize the contribution of the ddynamic_reconfigure project.
  Known Issue: Option for toggling sensor on and off while running is missing.
* Update constants.h
  update version to 2.1.2
* Potential Fix for librealsense2 v2.17.0 Compatilbility (`#523 <https://github.com/locusrobotics/realsense-ros/issues/523>`_)
  Fix to comply with librealsense v2.17.0.
  Thanks @m-price-softwearinc
* add log info - when dynamic reconfiguration is done.
* revert PR `#490 <https://github.com/locusrobotics/realsense-ros/issues/490>`_: rgbd_launch file is a running example for using the rgbd module. No need to add elements to installation for all users.
* add disparity processing
  moved colorizer filter to the end of filters pipeline.
* add decimation filter (`#504 <https://github.com/locusrobotics/realsense-ros/issues/504>`_)
  * add decimation filter. enable with filters:=decimation
  * fix tests to check number of holes in depth image.
  add tests to check decimation filter.
* fix tests to check number of holes in depth image.
  add tests to check decimation filter.
* add decimation filter. enable with filters:=decimation
* update version to 2.1.1
* start working on decimation filter
* add filters option to rs_aligned_depth.launch
* fix all sensors.
* fix bug: depth_auto_exposure was override in initialization by depth_exposure.
  fix bug: error in setting a parameter stop setting all other parameters.
* added missing dependencies: rgbd_launch (`#490 <https://github.com/locusrobotics/realsense-ros/issues/490>`_)
* fix bug: Initial dynamic configuration was stopped by starting an already started sensors. While this may not be the best practice, it's not doing any wrong and setting parameters to their default values should continue.
* fix issue: depth is being sent incorrectly if pointcloud is being sent. (`#498 <https://github.com/locusrobotics/realsense-ros/issues/498>`_)
  * add test for depth and aligned_depth_to_infra1.
  * fix bug: _aligned_depth_images initialized incorrectly if width, height not specified in launch parameters.
  * use librealsense2 align filter to align the depth image. Also fix bug that was on the previous projection.
  add test: align_depth_color_1
  * add test depth_w_cloud_1 according to issue `#491 <https://github.com/locusrobotics/realsense-ros/issues/491>`_.
  * fix bug: depth_frame is not sent if pointcloud is on.
* fix bug: depth_frame is not sent if pointcloud is on.
* add test depth_w_cloud_1 according to issue `#491 <https://github.com/locusrobotics/realsense-ros/issues/491>`_. Fails.
* use librealsense2 align filter to align the depth image. Also fix bug that was on the previous projection.
  add test: align_depth_color_1
* fix bug: _aligned_depth_images initialized incorrectly if width, height not specified in launch parameters.
* add test for depth and aligned_depth_to_infra1. The last one is knowingly fails.
* fix bug aligning depth to images
* base_realsense_node.cpp: fix typo.
* set_cams_transforms.py: fix bugs.
* add set_cams_transforms.py to add transformation between cameras.
* Pausing sensors with sens.stop(). Saves about 9% CPU load on useless processing.
* Adding a dynamic_reconfigure option to toggle ROS publication (issue `#477 <https://github.com/locusrobotics/realsense-ros/issues/477>`_).
* Set tf_prefix in demo_pointcloud.launch
* filters applied in given order.
  add spatial and temporal filters.
  pointcloud can be activated as a type of filter (also, still, with flag enable_pointcloud)
* fix build warning.
* modify test for pointcloud because of known bug in setting texture for pointcloud of 1st frame.
  New pointcloud does not put background color so values of test have changed.
* fix image size in pointcloud test.
* Change default names for frames to the same name specified for the camera topics
* new launch parameter for frame distinction in multi camera use
* enable filter colorizer.
  Issue: Can not send both pointcloud and colorized depth image at the same time.
* working pointcloud by filter. need to clean.
* Start adding filters.
  pointcloud is now implemented with filter.
  BUG: Not transmitting texture.
* add test for PointCloud2 in topic /camera/depth/color/points
* Start working on version 2.1.0 - enabling filters.
* Start working on version 2.1.0 - enabling filters.
* removed unnecessary device query (artifact from merge)
* create wrapper class PipelineSyncer to work around librealsense 2.16 feature, removing operator() from class asynchronous_syncer.
* remove librealsense2 from catkin dependencies.
* namespace argument renamed "camera".
* line up <group ns> parameter in all launch files. (`#438 <https://github.com/locusrobotics/realsense-ros/issues/438>`_)
  fixed parameter name for <group ns> to be "namespace", as defined previously in other launch files.
* fixed parameter name for <group ns> to be "namespace", as defined previously in other launch files.
* Travis CI build and test (`#437 <https://github.com/locusrobotics/realsense-ros/issues/437>`_)
  * fix issue `#335 <https://github.com/locusrobotics/realsense-ros/issues/335>`_ according to solution lsolanka as suggested in pull request `#336 <https://github.com/locusrobotics/realsense-ros/issues/336>`_.
  * moving all the properties and material definitions inside the macro as suggested by @felixvd
  * add compilation flag SET_USER_BREAK_AT_STARTUP to create user waiting point for debugging purposes.
  add reading from bagfile option by using <rosbag_filename> parameter in launch file.
  base_realsense_node.cpp: add option - by specifying width, height or fps as 0, pick up on the first sensor profile available.
  scripts/rs2_listener.py, rs2_test.py - initial version for file based, standalone unitest.
  * add .travis.yml file
* remove parse_bag_file.py
* possible fix
* use locations of realsense2
* TravisCI.yml: fix and add data downloading.
  rs2_test.py: fix test to match new bag file: outdoors.bag
* update .travis.yml
  make test expected to fail to display SUCCESS.
* add .travis.yml file
* add compilation flag SET_USER_BREAK_AT_STARTUP to create user waiting point for debugging purposes.
  add reading from bagfile option by using <rosbag_filename> parameter in launch file.
  base_realsense_node.cpp: add option - by specifying width, height or fps as 0, pick up on the first sensor profile available.
  scripts/rs2_listener.py, rs2_test.py - initial version for file based, standalone unitest.
* making camera name configurable, necessity for launching multiple cameras
* Fix the name of the alignment-related parameters when invoking the RealSenseNodeFactory.
* fix issue `#335 <https://github.com/locusrobotics/realsense-ros/issues/335>`_ according to solution lsolanka as suggested in pull request `#336 <https://github.com/locusrobotics/realsense-ros/issues/336>`_.
* moving all the properties and material definitions inside the macro as suggested by @felixvd
* fixed coordinate system for sensors in camera.
  renamed fisheye to color camera
* fix issue `#335 <https://github.com/locusrobotics/realsense-ros/issues/335>`_ according to solution lsolanka as suggested in pull request `#336 <https://github.com/locusrobotics/realsense-ros/issues/336>`_.
* Fixing the length of an array argument in rotationMatrixToQuaternion
* Add mesh and urdf for D435
* Also when align_depth is no, publish proper data on extrinsic topics.
  AFAIK there is no convention of what to publish on extrinsic topics, so you
  may choose to keep it as is, but I would say the current behavior can be
  surprising in a negative way.
* Fix the rotation quaternion in coordinate transforms.
  When going from one optical frame to another, the actual rotation we are
  performing is quaternion_optical.inverse() * Q * quaternion_optical, so we
  need to for the final rotation to be as specific in the extrinsics.
  The pointcloud is now properly aligned.
* Publish coordinate system transforms also when align depth is on.
  That fact that aligned_depth_to\_* is in color coordinates is already
  experessed by these cameras camera_info reporting the color frame. However,
  for the "depth", "infra1" etc. camera to be properly reported and for the
  pointcloud to have a change to align, we need to report the transformations.
* In coordinate system transforms, fix which extrincits we use and use matrix properly.
  Two bugs which cancel out each other for rotation, but not translation:
  - it seems that ROS and Realsense use different conventions of coordinate
  system transformations. In ROS, it is defined as a transformation of child
  fame coordinates to parent frame coordinates (see
  http://wiki.ros.org/tf/Overview/Transformations), while in RealSense
  it seems to be transformation of "from" frame coordinates to "to" frame
  coordinates. Thus, the order needs to be reversed.
  - the matrix in RealSense extrinsics is stored in column-major format, while
  Eigen::Matrix3f expects row-major, causing the matrix to be transposed.
  To see that this is a problem, one can open rviz and add the pointcloud and the
  color/image_raw camera. From the camera viewpoint, the images should align, but
  don't. This patch doesn't yet solve the whole problem, but makes it smaller.
* Fixes librealsense CMake vars.
* fix the aligned depth frame unit conversion issue
* Assign stream cam info instead of depth
* Contributors: Abhijit Majumdar, AlanBB277, Allison Thackston, AndyZe, Anthony Musco, Brian Fulkerson, CameronDevine, David W, Enrique Fernandez, Enrique Fernández Perdomo, Florenz Graf, Guilherme de Campos Affonso, Harsh Pandya, Ian Zhang, Jack Morrison, Jamie Cho, Jarvis Schultz, Mikołaj Zalewski, Miles Price, Nick Giancola, Paul Bovbel, Phillip Schmidt, RhysMcK, Robert Haschke, Ryan Sinnet, Shuntaro Yamazaki, Stephan, Stephan Sundermann, Thiago de Freitas, Unknown, Victor Lopez, akira_you, baumanta, brayan, carlos, doronhi, iliabara, lorenwel, oka, socrob, stevemacenski, vatbrain
