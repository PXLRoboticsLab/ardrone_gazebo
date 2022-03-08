#ifndef PLUGIN_ROS_IMU_NATIVE_H
#define PLUGIN_ROS_IMU_NATIVE_H

#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"
#include "sensor_msgs/Imu.h"
#include <ros/ros.h>

namespace gazebo
{
   class RosImuPlugin: public SensorPlugin
   {
   public:
      RosImuPlugin(){} // No hard coded topics in my code! {topicName = "/ardrone/imu";}
      virtual ~RosImuPlugin(){}
    
      virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
      virtual void onUpdated();
    
   protected:
      sensors::ImuSensorPtr imu_;
      event::ConnectionPtr updated_conn_;
      
      sensor_msgs::Imu imu_msg_;
      ros::NodeHandle* node_handle_;
      ros::Publisher pub_;
      // std::string topicName;
      std::string imu_topic_;

      // Added namespace capability
      std::string robot_namespace_;
   };
}

#endif // PLUGIN_ROS_IMU_NATIVE_H
