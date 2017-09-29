Change history
==============

101.2.1 (2017-09-29)
--------------------
* try fixing changelog
* update
* point cloud header stamp from gps
* fixed outer hour denotion
* workaround version number
* fixed bug gps time from uint64_t to double
* timestamp support initialized
* velodyne_pointcloud: remove incorrect catkin_package() DEPENDS option (`#93 <https://github.com/prclibo/velodyne/issues/93>`_)
  This eliminates a CMake warning when building on Xenial.
* Added an interface to set up raw data processing from a locally defined calibration file. This method is useful when processing data offline from a bag file, without starting any ros master
* test multiple nodelet manager support (`#108 <https://github.com/prclibo/velodyne/issues/108>`_)
* add launch args to support multiple devices (`#108 <https://github.com/prclibo/velodyne/issues/108>`_)
* Rearranged alphabetically.
* Remove unused constants.
  DISTANCE_MAX and DISTANCE_MAX_UNITS are not used anywhere in the code.
  Furthermore, using them would lead to errors as both VLP-64 manuals state that returns above 120 m should not be used. The VLP-32 manual allows 70 m as the maximum valid sensor range.
* Add more options in launch files.
  - rpm, device_ip, port, read_once, read_fast, repeat_delay
* Fix misleading typecasts.
  intensity and VPoint::intensity are both of type float.
* update change history
* Modified velodyne_point_cloud/src/lib/rawdata.cc to address warning
  that last_azimuth_diff variable may be used uninitialized.  Variable
  is now initialized to 0 at creation.
  velodyne/velodyne_pointcloud/src/lib/rawdata.cc:328:57: error: ‘last_azimuth_diff’ may be used uninitialized in this function [-Werror=maybe-uninitialized]
  azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
* Modified  velodyne_pointcloud/src/conversion/colors.cc to remove
  address build warning for strict-aliasing.
  velodyne/velodyne_pointcloud/src/conversions/colors.cc:84:58:
  error: dereferencing type-punned pointer will break strict-aliasing rules [-Werror=strict-aliasing]
  p.rgb = *reinterpret_cast<float*>(&rainbow[color]);
  ^
* velodyne_pointcloud: Fix compile warning "Wrong initialization order"
* velodyne_pointcloud: add dynamic reconfig update to change log (`#78 <https://github.com/prclibo/velodyne/issues/78>`_)
* velodyne_pointcloud: use recommended add_dependencies() CMake variable `#78 <https://github.com/prclibo/velodyne/issues/78>`_
* velodyne_pointcloud: fix transform unit tests
  Use tf2_ros static_transform_publisher for more consistent timing (`#2 <https://github.com/prclibo/velodyne/issues/2>`_)
* Merge branch 'reconfigure_transform_node' of https://github.com/fudger/velodyne
* prepare change history for coming Indigo release (`#59 <https://github.com/prclibo/velodyne/issues/59>`_)
* calibration: unit test case improvements (`#84 <https://github.com/prclibo/velodyne/issues/84>`_)
* calibration: read all intensities as float, then convert (`#84 <https://github.com/prclibo/velodyne/issues/84>`_)
* calibration: add gtest for `#84 <https://github.com/prclibo/velodyne/issues/84>`_
  This currently fails on 64e_s2.1-sztaki.yaml and on issue_84_float_intensities.yaml.
* calibration: make max_intensity and min_intensity optional (`#84 <https://github.com/prclibo/velodyne/issues/84>`_)
  This fixes a regression in the 32e and VLP-16 calibrations which do not contain
  intensity values. There is still a problem with the 64e_s2.1 calibration.
* Merge pull request `#76 <https://github.com/prclibo/velodyne/issues/76>`_ from pomerlef/master
  Sign inversion in some equations
* fix the yaml-cpp 0.5 code paths
* allow horiz_offset_correction to be optional with 0 as default
* allow floats instead of ints in min/max_intensity
* Resolve frame ID name using tf prefix.
* Improve coding style.
* Set up dynamic reconfiguration for transform_node.
  Previously, transform_node has neither read parameters other than frame_id from the command line nor has it exposed these parameters via dynamic reconfigure. As parameters like max_range and view_width have been initialized to zero, the inconfigurable transform_node has returned an empty point cloud.
  Now, transform_node launches an reconfigure server just as cloud_node. In contrast to cloud_node, transform node exposes another parameter for dynamic reconfiguration: frame_id, i.e. the frame of reference the incoming Velodyne points are transformed to.
* Add a missing space.
* Fix line that always indicates use of model VLP-16.
* Align console output of calibration data.
* Merge branch 'master' of https://github.com/ros-drivers/velodyne
* resolve sign error
* Fix data type error that distorts the point cloud.
* Fix and add a few comments.
* Remove unused variable
  I think that `dsr` was unused. See line 317:
  for (int dsr=0; ...
* VLP-16: skip badly formatted data packets (`#62 <https://github.com/prclibo/velodyne/issues/62>`_, `#63 <https://github.com/prclibo/velodyne/issues/63>`_)
* restore VLP-16 min_range setting to 0.4 (`#60 <https://github.com/prclibo/velodyne/issues/60>`_)
  NOTE: There is still some other problem keeping that from working.
* permit min_range settings below 0.9 meters (`#60 <https://github.com/prclibo/velodyne/issues/60>`_)
  No known models are currently known to return closer measurements.
* Fixed azimuth overflow bug.
  For interpolated azimuth values between 35999.5 and 36000.0 the nested round(fmod())
  yields a value of 36000 which is invalid and overflows the pre-computed sin/cos arrays,
  since they only go form 0..35999
* Added vertical sin angle correction
* Merge pull request `#47 <https://github.com/prclibo/velodyne/issues/47>`_ from prclibo/master
  fixed rounding bug in intensity calculation found by songshiyu
* fixed rounding bug in intensity calculation found by songshiyu
* fix some overly long C++ source lines
* missed the space in the file name which caused the build to fail, removed space before extension
* adding the VLP16 test scripts and updating the CMakeLists to include the test file from http://download.ros.org/data/velodyne/vlp16.pcap
* adding support for the VLP16
* fixed point computation according to the 64e_s2(.1) velodyne manual, with luopei"s help
* fixed gen_calibration min/max intensity type
* fixed a calibration file parsing bug
* Fix broken reference to pull request `#22 <https://github.com/prclibo/velodyne/issues/22>`_
* Contributors: Adam Stambler, Alex Rodrigues, Alexander Schaefer, Andreas Wachaja, Bo Li, Daniel Jartoux, Jack O'Quin, Jose Luis Blanco-Claraco, Kun Li, Todor Stoyanov, William Woodall, libo24, phussey, pomerlef

1.2.0 (2014-08-06)
------------------

* velodyne_pointcloud: remove model-dependent "constants" from
  rawdata.h (`#28
  <https://github.com/ros-drivers/velodyne/issues/28>`_)
* velodyne_pointcloud: change default min_range to 0.9 meters (`#25
  <https://github.com/ros-drivers/velodyne/issues/25>`_)
* Added support for YAML-CPP 0.5+ (`#23
  <https://github.com/ros-drivers/velodyne/pull/23>`_).
* Add dynamic_reconfigure feature.
* Add angular limits to the output point cloud, useful for omitting
  part of it. (`#22 <https://github.com/ros-drivers/velodyne/pull/22>`_).
* Contributors: Jack O'Quin, Scott K Logan, Thomas Solatges

1.1.2 (2013-11-05)
------------------

 * Move unit test data to download.ros.org (`#18`_).
 * Install missing gen_calibration.py script (`#20`_).

1.1.1 (2013-07-30)
------------------

 * Fix lost frame_id transform problem caused by PCL 1.7 fix (`#13`_).
 * Add support for HDL-64E S2 and S2.1 models, which were not working
   before (`#11`_), thanks to Gabor Meszaros (`#12`_).
 * Add additional parameters to launch files (`#14`_).
 * Contributors: Gabor Meszaros, Jack O'Quin

1.1.0 (2013-07-16)
------------------

 * Fix build problems due to PCL 1.7 API incompatibilities (`#8`_),
   thanks to William Woodall.  This version also works with Groovy, as
   long as the correct ``pcl_conversions`` is installed.
 * Fix errors with Mac OSX compiler (`#8`_).
 * Install ``pluginlib`` XML files (`#9`_).
 * Install some launch and parameter files.
 * Enable unit tests when ``CATKIN_ENABLE_TESTING`` is set (`#10`_).

1.0.1 (2013-06-15)
------------------

 * Declare explicit ``pluginlib`` dependency (`#4`_).

1.0.0 (2013-06-14)
------------------

 * Convert to catkin (`#1`_).
 * Release to Hydro.

0.9.2 (2013-07-08)
------------------

 * Fix Groovy build problem (`#7`_).

0.9.1 (2012-06-05)
------------------

 * Only include "enabled" lasers in YAML calibration file.
 * New param subdirectory for parameter files.
 * Add launch file for the HDL-32E.
 * Add rviz_points.vcg file for viewing Velodyne point clouds with rviz.
 * Fix bug when reading configuration with default minIntensity.
 * Add unit tests with 32E data.
 * Released to Electric, Fuerte and Groovy.

0.9.0 (2012-04-03)
------------------

 * Completely revised API, anticipating a 1.0.0 release.
 * HDL-32E device support.
 * New YAML configuration file format.
 * New velodyne_driver and velodyne_pointcloud packages.
 * Old velodyne_common and velodyne_pcl packages no longer included.
 * Released to Electric, Fuerte and Groovy.

0.2.6 (2011-02-23)
------------------

 * Label all timing-dependent tests "realtime" so they do not run by
   default on the build farm machines.

0.2.5 (2010-11-19)
------------------

 * Initial implementation of new 0.3 interfaces.
 * Support for ROS 1.3 `std_msgs::Header` changes.

0.2.0 (2010-08-17)
------------------

 * Initial release to ROS C-turtle.

.. _`#1`: https://github.com/ros-drivers/velodyne/issues/1
.. _`#4`: https://github.com/ros-drivers/velodyne/issues/4
.. _`#7`: https://github.com/ros-drivers/velodyne/issues/7
.. _`#8`: https://github.com/ros-drivers/velodyne/pull/8
.. _`#9`: https://github.com/ros-drivers/velodyne/issues/9
.. _`#10`: https://github.com/ros-drivers/velodyne/issues/10
.. _`#11`: https://github.com/ros-drivers/velodyne/issues/11
.. _`#12`: https://github.com/ros-drivers/velodyne/pull/12
.. _`#13`: https://github.com/ros-drivers/velodyne/issues/13
.. _`#14`: https://github.com/ros-drivers/velodyne/pull/14
.. _`#17`: https://github.com/ros-drivers/velodyne/issues/17
.. _`#18`: https://github.com/ros-drivers/velodyne/issues/18
.. _`#20`: https://github.com/ros-drivers/velodyne/issues/20
.. _`#41`: https://github.com/ros-drivers/velodyne/pull/41
.. _`#42`: https://github.com/ros-drivers/velodyne/pull/42
.. _`#43`: https://github.com/ros-drivers/velodyne/pull/43
.. _`#44`: https://github.com/ros-drivers/velodyne/pull/44
.. _`#47`: https://github.com/ros-drivers/velodyne/pull/47
.. _`#50`: https://github.com/ros-drivers/velodyne/issue/50
.. _`#55`: https://github.com/ros-drivers/velodyne/pull/55
.. _`#76`: https://github.com/ros-drivers/velodyne/pull/76
.. _`#78`: https://github.com/ros-drivers/velodyne/pull/78
