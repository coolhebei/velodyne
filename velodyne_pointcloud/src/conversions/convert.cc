/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include "convert.h"

#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh);


    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
      
    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
    prev_min_angle_ = M_PI;
  }
  
  void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      velodyne_rawdata::VPointCloud unpacked;
      data_->unpack(scanMsg->packets[i], unpacked);
      if (unpacked.empty()) continue;

      std::vector<float> angles;
      angles.reserve(unpacked.size());
      for (size_t j = 0; j < unpacked.size(); ++j)
      {
        float angle = std::atan2(unpacked.at(j).y, unpacked.at(j).x);
        angles.push_back(angle);
      }
      std::sort(angles.begin(), angles.end());

      if (angles.front() > 0 && !out_msg_.empty() && prev_min_angle_ <= 0)
      {
        out_msg_.header.stamp =
            static_cast<uint64_t>(out_msg_.front().timestamp * 1e6);
        out_msg_.header.frame_id = scanMsg->header.frame_id;
        out_msg_.height = 1;
        output_.publish(out_msg_);
        out_msg_.clear();
      }
      out_msg_.insert(out_msg_.end(), unpacked.begin(), unpacked.end());
      prev_min_angle_ = angles.front();
    }
  }

} // namespace velodyne_pointcloud
