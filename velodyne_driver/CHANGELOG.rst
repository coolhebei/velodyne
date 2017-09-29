Change history
==============

101.2.1 (2017-09-29)
--------------------
* try fixing changelog
* add launch args to support multiple devices (`#108 <https://github.com/prclibo/velodyne/issues/108>`_)
* Rearranged alphabetically.
* Add more options in launch files.
  - rpm, device_ip, port, read_once, read_fast, repeat_delay
* velodyne_driver/src/lib/input.cc : fix for device_ip filter
  Fix for device_ip filter in InputSocket: initialization of devip\_ for correct ip filtering in InputSocket::getPacket.
* velodyne_driver: credit @priyankadey for VLP-16 bug fix (`#96 <https://github.com/prclibo/velodyne/issues/96>`_)
* updated VLP-16 packet rate from user manual.
  Also verified with sensor. It reduced overlap in the pointcloud
* update change history
* fix g++ 5.3.1 compile errors (`#94 <https://github.com/prclibo/velodyne/issues/94>`_)
* merge current master (`#94 <https://github.com/prclibo/velodyne/issues/94>`_)
* update velodyne_driver package description to include all models
* velodyne_driver: Add dynamic_reconfigure and time_offset correction
  The value of time_offset is added to the calculated time stamp in live mode for each packet.
* velodyne_driver: Make input destructors virtual
* prepare change history for coming Indigo release (`#59 <https://github.com/prclibo/velodyne/issues/59>`_)
* velodyne_driver: use port number for PCAP data (`#66 <https://github.com/prclibo/velodyne/issues/66>`_)
* Merge pull request `#39 <https://github.com/prclibo/velodyne/issues/39>`_ from zooxco/multivelodyne
  support for multiple velodynes
* adding the VLP16 test scripts and updating the CMakeLists to include the test file from http://download.ros.org/data/velodyne/vlp16.pcap
* adding support for the VLP16
* parameters to set the udp port
* fixed missing header
* cleanup debug line
* parameter and code added for working with multiple velodynes
* Contributors: Andreas Wachaja, Bo Li, Brice Rebsamen, Daniel Jartoux, Denis Dillenberger, Ilya, Jack O'Quin, Matteo Murtas, Priyanka Dey, junior, phussey

1.2.0 (2014-08-06)
------------------
* Fixed bug in diagnostic rate for driver (`#16
  <https://github.com/ros-drivers/velodyne/issues/16>`_)
* Contributors: Brice Rebsamen, Jack O'Quin

1.1.2 (2013-11-05)
-------------------

 * Move unit test data to download.ros.org (`#18`_).
 * Install missing vdump script (`#17`_).

1.1.1 (2013-07-30)
------------------

 * Add support for HDL-64E S2 and S2.1 models, which were not working
   before (`#11`_), thanks to Gabor Meszaros (`#12`_).
 * Add additional parameters to launch files (`#14`_).

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

 * Driver socket read path improvements.
 * Add unit tests with 32E data.
 * Released to Electric, Fuerte and Groovy.

0.9.0 (2012-04-03)
------------------

 * Completely revised API, anticipating a 1.0.0 release.
 * HDL-32E device support.
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
