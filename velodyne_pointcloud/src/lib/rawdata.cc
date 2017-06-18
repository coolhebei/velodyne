/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <fstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include <velodyne_pointcloud/rawdata.h>

namespace velodyne_rawdata
{
inline uint64_t gpsTimeFromUnix(double unix_sec)
{
    double leaps = 18; // FIXME
    double gps_time = unix_sec - (315964800 - leaps);
    return gps_time > 0 ? gps_time : 0;
}

  ////////////////////////////////////////////////////////////////////////
  //
  // RawData base class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  RawData::RawData() : has_gps_sync_(false) {}
  
  /** Update parameters: conversions and update */
  void RawData::setParameters(double min_range,
                              double max_range,
                              double view_direction,
                              double view_width)
  {
    config_.min_range = min_range;
    config_.max_range = max_range;

    //converting angle parameters into the velodyne reference (rad)
    config_.tmp_min_angle = view_direction + view_width/2;
    config_.tmp_max_angle = view_direction - view_width/2;
    
    //computing positive modulo to keep theses angles into [0;2*M_PI]
    config_.tmp_min_angle = fmod(fmod(config_.tmp_min_angle,2*M_PI) + 2*M_PI,2*M_PI);
    config_.tmp_max_angle = fmod(fmod(config_.tmp_max_angle,2*M_PI) + 2*M_PI,2*M_PI);
    
    //converting into the hardware velodyne ref (negative yaml and degrees)
    //adding 0.5 perfomrs a centered double to int conversion 
    config_.min_angle = 100 * (2*M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
    config_.max_angle = 100 * (2*M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;
    if (config_.min_angle == config_.max_angle)
    {
      //avoid returning empty cloud if min_angle = max_angle
      config_.min_angle = 0;
      config_.max_angle = 36000;
    }
  }

  /** Set up for on-line operation. */
  int RawData::setup(ros::NodeHandle private_nh)
  {
    // get path to angles.config file for this device
    if (!private_nh.getParam("calibration", config_.calibrationFile))
      {
        ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

        // have to use something: grab unit test version as a default
        std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
        config_.calibrationFile = pkgPath + "/params/64e_utexas.yaml";
      }

    ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized) {
      ROS_ERROR_STREAM("Unable to open calibration file: " << 
          config_.calibrationFile);
      return -1;
    }

    private_nh.getParam("gps_sync", has_gps_sync_);
    ROS_INFO_STREAM("has GPS sync: " << has_gps_sync_);
    
    ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");
    
    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }
   return 0;
  }


  /** Set up for offline operation */
  int RawData::setupOffline(std::string calibration_file, double max_range_, double min_range_, bool gps_sync)
  {
      has_gps_sync_ = gps_sync;

      config_.max_range = max_range_;
      config_.min_range = min_range_;
      ROS_INFO_STREAM("data ranges to publish: ["
	      << config_.min_range << ", "
	      << config_.max_range << "]");

      config_.calibrationFile = calibration_file;

      ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

      calibration_.read(config_.calibrationFile);
      if (!calibration_.initialized) {
	  ROS_ERROR_STREAM("Unable to open calibration file: " <<
		  config_.calibrationFile);
	  return -1;
      }

      // Set up cached values for sin and cos of all the possible headings
      for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
	  float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
	  cos_rot_table_[rot_index] = cosf(rotation);
	  sin_rot_table_[rot_index] = sinf(rotation);
      }
      return 0;
  }


  /** @brief convert raw packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack(const velodyne_msgs::VelodynePacket &pkt,
                       VPointCloud &pc)
  {
    ROS_DEBUG_STREAM("Received packet, time in unpack: " << pkt.stamp);
    
    /** special parsing for the VLP16 **/
    if (calibration_.num_lasers == 16)
    {
      unpack_vlp16(pkt, pc);
      return;
    }
    
    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    for (int i = 0; i < BLOCKS_PER_PACKET; i++) {

      // upper bank lasers are numbered [0..31]
      // NOTE: this is a change from the old velodyne_common implementation
      int bank_origin = 0;
      if (raw->blocks[i].header == LOWER_BANK) {
        // lower bank lasers are [32..63]
        bank_origin = 32;
      }

      for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
        
        float x, y, z;
        float intensity;
        uint8_t laser_number;       ///< hardware laser number

        laser_number = j + bank_origin;
        velodyne_pointcloud::LaserCorrection &corrections = 
          calibration_.laser_corrections[laser_number];

        /** Position Calculation */

        union two_bytes tmp;
        tmp.bytes[0] = raw->blocks[i].data[k];
        tmp.bytes[1] = raw->blocks[i].data[k+1];
        /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
        if ((raw->blocks[i].rotation >= config_.min_angle 
             && raw->blocks[i].rotation <= config_.max_angle 
             && config_.min_angle < config_.max_angle)
             ||(config_.min_angle > config_.max_angle 
             && (raw->blocks[i].rotation <= config_.max_angle 
             || raw->blocks[i].rotation >= config_.min_angle))){
          float distance = tmp.uint * DISTANCE_RESOLUTION;
          distance += corrections.dist_correction;
  
          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          float cos_rot_correction = corrections.cos_rot_correction;
          float sin_rot_correction = corrections.sin_rot_correction;
  
          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle = 
            cos_rot_table_[raw->blocks[i].rotation] * cos_rot_correction + 
            sin_rot_table_[raw->blocks[i].rotation] * sin_rot_correction;
          float sin_rot_angle = 
            sin_rot_table_[raw->blocks[i].rotation] * cos_rot_correction - 
            cos_rot_table_[raw->blocks[i].rotation] * sin_rot_correction;
  
          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;
  
          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
  
          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0) xx=-xx;
          if (yy < 0) yy=-yy;
    
          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available) {
            distance_corr_x = 
              (corrections.dist_correction - corrections.dist_correction_x)
                * (xx - 2.4) / (25.04 - 2.4) 
              + corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y = 
              (corrections.dist_correction - corrections.dist_correction_y)
                * (yy - 1.93) / (25.04 - 1.93)
              + corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }
  
          float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
          ///the expression wiht '-' is proved to be better than the one with '+'
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
  
          float distance_y = distance + distance_corr_y;
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
  
          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;
  
          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;
  
          /** Intensity Calculation */
  
          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;
  
          intensity = raw->blocks[i].data[k+2];
  
          float focal_offset = 256 
                             * (1 - corrections.focal_distance / 13100) 
                             * (1 - corrections.focal_distance / 13100);
          float focal_slope = corrections.focal_slope;
          intensity += focal_slope * (abs(focal_offset - 256 * 
            (1 - static_cast<float>(tmp.uint)/65535)*(1 - static_cast<float>(tmp.uint)/65535)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;
  
          if (pointInRange(distance)) {
  
            // convert polar coordinates to Euclidean XYZ
            VPoint point;
            point.ring = corrections.laser_ring;
            point.x = x_coord;
            point.y = y_coord;
            point.z = z_coord;
            point.intensity = intensity;
  
            // append this point to the cloud
            pc.points.push_back(point);
            ++pc.width;
          }
        }
      }
    }
  }


double timeOffset[2][16][12]={
       {
/*0 */ {0.00, 110.59, 221.18, 331.78, 442.37, 552.96, 663.55, 774.14, 884.74, 995.33, 1105.92, 1216.51},
/*1 */ {2.30, 112.90, 223.49, 334.08, 444.67, 555.26, 665.86, 776.45, 887.04, 997.63, 1108.22, 1218.82},
/*2 */ {4.61, 115.20, 225.79, 336.38, 446.98, 557.57, 668.16, 778.75, 889.34, 999.94, 1110.53, 1221.12 },
/*3 */ {6.91, 117.50, 228.10, 338.69, 449.28, 559.87, 670.46, 781.06, 891.65, 1002.24, 1112.83, 1223.42 },
/*4 */ {9.22, 119.81, 230.40, 340.99, 451.58, 562.18, 672.77, 783.36, 893.95, 1004.54, 1115.14, 1225.73 },
/*5 */ {11.52,122.11, 232.70, 343.30, 453.89, 564.48, 675.07, 785.66, 896.26, 1006.85, 1117.44, 1228.03 },
/*6 */ {13.82,124.42,235.01,345.60,456.19,566.78,677.38,787.97,898.56,1009.15,1119.74,1230.34},
/*7 */ {16.13,126.72,237.31,347.90,458.50,569.09,679.68,790.27,900.86,1011.46,1122.05,1232.64},
/*8 */ {18.43,129.02,239.62,350.21,460.80,571.39,681.98,792.58,903.17,1013.76,1124.35,1234.94},
/*9 */ {20.74,131.33,241.92,352.51,463.10,573.70,684.29,794.88,905.47,1016.06,1126.66,1237.25},
/*10*/ {23.04,133.63,244.22,354.82,465.41,576.00,686.59,797.18,907.78,1018.37,1128.96,1239.55},
/*11*/ {25.34,135.94,246.53,357.12,467.71,578.30,688.90,799.49,910.08,1020.67,1131.26,1241.86},
/*12*/ {27.65,138.24,248.83,359.42,470.02,580.61,691.20,801.79,912.38,1022.98,1133.57,1244.16},
/*13*/ {29.95,140.54,251.14,361.73,472.32,582.91,693.50,804.10,914.69,1025.28,1135.87,1246.46},
/*14*/ {32.26,142.85,253.44,364.03,474.62,585.22,695.81,806.40,916.99,1027.58,1138.18,1248.77},
/*15*/ {34.56,145.15,255.74,366.34,476.93,587.52,698.11,808.70,919.30,1029.89,1140.48,1251.07}},

       {
/*0 */ {55.30,165.89,276.48,387.07,497.66,608.26,718.85,829.44,940.03,1050.62,1161.22,1271.81},
/*1 */ {57.60,168.19,278.78,389.38,499.97,610.56,721.15,831.74,942.34,1052.93,1163.52,1274.11},
/*2 */ {59.90,170.50,281.09,391.68,502.27,612.86,723.46,834.05,944.64,1055.23,1165.82,1276.42},
/*3 */ {62.21,172.80,283.39,393.98,504.58,615.17,725.76,836.35,946.94,1057.54,1168.13,1278.72},
/*4 */ {64.51,175.10,285.70,396.29,506.88,617.47,728.06,838.66,949.25,1059.84,1170.43,1281.02},
/*5 */ {66.82,177.41,288.00,398.59,509.18,619.78,730.37,840.96,951.55,1062.14,1172.74,1283.33},
/*6 */ {69.12,179.71,290.30,400.90,511.49,622.08,732.67,843.26,953.86,1064.45,1175.04,1285.63},
/*7 */ {71.42,182.02,292.61,403.20,513.79,624.38,734.98,845.57,956.16,1066.75,1177.34,1287.94},
/*8 */ {73.73,184.32,294.91,405.50,516.10,626.69,737.28,847.87,958.46,1069.06,1179.65,1290.24},
/*9 */ {76.03,186.62,297.22,407.81,518.40,628.99,739.58,850.18,960.77,1071.36,1181.95,1292.54},
/*10*/ {78.34,188.93,299.52,410.11,520.70,631.30,741.89,852.48,963.07,1073.66,1184.26,1294.85},
/*11*/ {80.64,191.23,301.82,412.42,523.01,633.60,744.19,854.78,965.38,1075.97,1186.56,1297.15},
/*12*/ {82.94,193.54,304.13,414.72,525.31,635.90,746.50,857.09,967.68,1078.27,1188.86,1299.46},
/*13*/ {85.25,195.84,306.43,417.02,527.62,638.21,748.80,859.39,969.98,1080.58,1191.17,1301.76},
/*14*/ {87.55,198.14,308.74,419.33,529.92,640.51,751.10,861.70,972.29,1082.88,1193.47,1304.06},
/*15*/ {89.86,200.45,311.04,421.63,532.22,642.82,753.41,864.00,974.59,1085.18,1195.78,1306.37}} 
};

  
  /** @brief convert raw VLP16 packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack_vlp16(const velodyne_msgs::VelodynePacket &pkt,
                             VPointCloud &pc)
  {
    //msg stamp
    ROS_INFO("Received packet, time in unpack_vlp16:%f ", pkt.stamp.toSec());//double 
    //uint64_t timeNSec;
    //timeNSec = pkt.stamp.toNSec();
    //ROS_INFO("timeNSec:%ld", timeNSec);

    float azimuth;
    float azimuth_diff;
    float last_azimuth_diff=0;
    float azimuth_corrected_f;
    int azimuth_corrected;
    float x, y, z;
    float intensity;

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    double gps_packet_time = 0;

    if (has_gps_sync_) {
        double outer_hour = std::floor(pkt.stamp.toSec() / 3600);
        double ros_inner_hour = std::fmod(pkt.stamp.toSec(), 3600);
        double pkt_inner_hour;
        uint32_t buffer;
        std::memcpy(&buffer, pkt.data.data() + 1200, sizeof(uint32_t));
        pkt_inner_hour = static_cast<double>(buffer) / 1e6;
        if (ros_inner_hour > pkt_inner_hour + 3600 - 5)
            outer_hour += 3600;
        else if (pkt_inner_hour > ros_inner_hour + 3600 - 5)
            outer_hour -= 3600;
        else if (std::abs(ros_inner_hour - pkt_inner_hour) > 1) {
            ROS_FATAL("VelodynePacket stamp inconsistent with receive time."
                    " (%f, %f).\n", ros_inner_hour, pkt_inner_hour);
            ros::shutdown();
        }
        gps_packet_time = gpsTimeFromUnix(
                static_cast<double>(outer_hour + pkt_inner_hour) / 1e6);
    }

                                //blocks_per_packet = 12
    for (int block = 0; block < BLOCKS_PER_PACKET; block++) {

      // ignore packets with mangled or otherwise different contents
      if (UPPER_BANK != raw->blocks[block].header) {
        // Do not flood the log with messages, only issue at most one
        // of these warnings per minute.
        ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
                                 << block << " header value is "
                                 << raw->blocks[block].header);
        return;                         // bad packet: skip the rest
      }

      // Calculate difference between current and next block's azimuth angle.
      azimuth = (float)(raw->blocks[block].rotation);
      if (block < (BLOCKS_PER_PACKET-1)){
        azimuth_diff = (float)((36000 + raw->blocks[block+1].rotation - raw->blocks[block].rotation)%36000);
        last_azimuth_diff = azimuth_diff;
      }else{
        azimuth_diff = last_azimuth_diff;
      }
                                      // 2
      for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++){
                                // 16
         for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE){
          velodyne_pointcloud::LaserCorrection &corrections = 
            calibration_.laser_corrections[dsr];

          /** Position Calculation */
          union two_bytes tmp;
          tmp.bytes[0] = raw->blocks[block].data[k];
          tmp.bytes[1] = raw->blocks[block].data[k+1];
          
          /** correct for the laser rotation as a function of timing during the firings **/
          azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
          azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;
          
          /*condition added to avoid calculating points which are not
            in the interesting defined area (min_angle < area < max_angle)*/
          if ((azimuth_corrected >= config_.min_angle 
               && azimuth_corrected <= config_.max_angle 
               && config_.min_angle < config_.max_angle)
               ||(config_.min_angle > config_.max_angle 
               && (azimuth_corrected <= config_.max_angle 
               || azimuth_corrected >= config_.min_angle))){

            // convert polar coordinates to Euclidean XYZ
            float distance = tmp.uint * DISTANCE_RESOLUTION;
            distance += corrections.dist_correction;
            
            float cos_vert_angle = corrections.cos_vert_correction;
            float sin_vert_angle = corrections.sin_vert_correction;
            float cos_rot_correction = corrections.cos_rot_correction;
            float sin_rot_correction = corrections.sin_rot_correction;
    
            // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
            // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
            float cos_rot_angle = 
              cos_rot_table_[azimuth_corrected] * cos_rot_correction + 
              sin_rot_table_[azimuth_corrected] * sin_rot_correction;
            float sin_rot_angle = 
              sin_rot_table_[azimuth_corrected] * cos_rot_correction - 
              cos_rot_table_[azimuth_corrected] * sin_rot_correction;
    
            float horiz_offset = corrections.horiz_offset_correction;
            float vert_offset = corrections.vert_offset_correction;
    
            // Compute the distance in the xy plane (w/o accounting for rotation)
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
    
            // Calculate temporal X, use absolute value.
            float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
            // Calculate temporal Y, use absolute value
            float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
            if (xx < 0) xx=-xx;
            if (yy < 0) yy=-yy;
      
            // Get 2points calibration values,Linear interpolation to get distance
            // correction for X and Y, that means distance correction use
            // different value at different distance
            float distance_corr_x = 0;
            float distance_corr_y = 0;
            if (corrections.two_pt_correction_available) {
              distance_corr_x = 
                (corrections.dist_correction - corrections.dist_correction_x)
                  * (xx - 2.4) / (25.04 - 2.4) 
                + corrections.dist_correction_x;
              distance_corr_x -= corrections.dist_correction;
              distance_corr_y = 
                (corrections.dist_correction - corrections.dist_correction_y)
                  * (yy - 1.93) / (25.04 - 1.93)
                + corrections.dist_correction_y;
              distance_corr_y -= corrections.dist_correction;
            }
    
            float distance_x = distance + distance_corr_x;
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
            x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
    
            float distance_y = distance + distance_corr_y;
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
            y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
    
            // Using distance_y is not symmetric, but the velodyne manual
            // does this.
            /**the new term of 'vert_offset * cos_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;
  
    
            /** Use standard ROS coordinate system (right-hand rule) */
            float x_coord = y;
            float y_coord = -x;
            float z_coord = z;
    
            /** Intensity Calculation */
            float min_intensity = corrections.min_intensity;
            float max_intensity = corrections.max_intensity;
    
            intensity = raw->blocks[block].data[k+2];
    
            float focal_offset = 256 
                               * (1 - corrections.focal_distance / 13100) 
                               * (1 - corrections.focal_distance / 13100);
            float focal_slope = corrections.focal_slope;
            intensity += focal_slope * (abs(focal_offset - 256 * 
              (1 - tmp.uint/65535)*(1 - tmp.uint/65535)));
            intensity = (intensity < min_intensity) ? min_intensity : intensity;
            intensity = (intensity > max_intensity) ? max_intensity : intensity;

            /*
               Sequence Index = 23
               Data Point Index = 15
               Time Stamp = 45,231,878us
               ExactPointTime = Timestamp + TimeOffset
               ExactPointTime = 45,231,878 + (55.296us * 23) + (2.304us * 15)
               ExactPointTime = 45,231,878 + 1,306.368us
               ExactPointTime = 45,233,184.368us
            */
            // TimeOffset[firing][dsr] = 55.296 * firing + 2.304 *dsr;

    
            if (pointInRange(distance)) {
    
              // append this point to the cloud
              VPoint point;
              point.ring = corrections.laser_ring;
              point.x = x_coord;
              point.y = y_coord;
              point.z = z_coord;
              point.intensity = intensity;
              
              //out of hour + inner hour + timeoffset;
              point.timestamp = gps_packet_time + timeOffset[firing][dsr][block]/1000000.0; // 
              // ROS_INFO("point.timestamp[%d][%d][%d]:%f", firing,dsr,block,point.timestamp);
              
              pc.points.push_back(point);
              ++pc.width;
            }
          }
        }
      }
    }
  }  

} // namespace velodyne_rawdata
