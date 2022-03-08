#ifndef PLUGIN_ROS_SONAR_H
#define PLUGIN_ROS_SONAR_H

#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Range.h"
#include <ros/ros.h>

namespace gazebo
{
   class RosSonarPlugin: public SensorPlugin
   {
   public:
      RosSonarPlugin(){} // No hard coded topics in my code!{topicName = "/ardrone/sonar";}
      virtual ~RosSonarPlugin(){}

      virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
      virtual void onUpdated();

   protected:
      //sensors::ImuSensorPtr imu_;
      sensors::SonarSensorPtr sonar_;
      
      event::ConnectionPtr updated_conn_;
      
      //sensor_msgs::Imu imu_msg_;
      sensor_msgs::Range sonar_msg_;
      
      ros::NodeHandle* node_handle_;
      ros::Publisher pub_;
      // std::string topicName;
      std::string sonar_topic_;

      // Added namespace capability
      std::string robot_namespace_;

   };
}

#endif // PLUGIN_ROS_SONAR_H
