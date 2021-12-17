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
#include <chrono>
using namespace std::chrono;
using namespace std;

#include "velodyne_pointcloud/convert.h"

#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/Int32MultiArray.h>

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
    
    time_analysis = node.advertise<std_msgs::Int32MultiArray>("time_analysis_pointcloud", 10);
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

    std_msgs::Int32MultiArray array;
    auto start = high_resolution_clock::now();

    // allocate a point cloud with same time and frame ID as raw data
    PointcloudXYZIR outMsg;
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsg.pc->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg.pc->header.frame_id = scanMsg->header.frame_id;
    outMsg.pc->height = 1;

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      data_->unpack(scanMsg->packets[i], outMsg);
    }

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    array.data.push_back(duration.count());
    array.data.push_back(outMsg.pc->width);
    time_analysis.publish(array);

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg.pc->height * outMsg.pc->width
                     << " Velodyne points, time: " << outMsg.pc->header.stamp);
    output_.publish(outMsg.pc);
  }

} // namespace velodyne_pointcloud
