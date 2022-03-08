#ifndef ARDRONE_GAZEBO_PLUGIN_H
#define ARDRONE_GAZEBO_PLUGIN_H

#include <thread>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/Imu.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

#include "ardrone_gazebo/pid_controller.h"

#include "ardrone_autonomy/Navdata.h"

#define Emergency       0
#define Inited          1
#define LANDED_MODEL    2
#define FLYING_MODEL    3
#define HOOVERING_MODEL 4
#define TEST_MODEL      5
#define TAKINGOFF_MODEL 6
#define GOTOHOVER_MODEL 7
#define LANDING_MODEL   8
#define LOOPING_MODEL   9

#define EPS 1E-6

namespace gazebo
{
   class ARDronePlugin : public ModelPlugin
   {
   private:
      // Important variables to link the code to the model.
      std::string ardrone_namespace;
      physics::ModelPtr ardrone_model;
      physics::WorldPtr ardrone_world;
      std::string ardrone_link_name;
      physics::LinkPtr ardrone_link;
      std::unique_ptr<ros::NodeHandle> ardrone_ros_node;
      event::ConnectionPtr ardrone_update_connection;

      // Publishers
      ros::Publisher ardrone_navdata_publisher;

      ros::Publisher ardrone_gt_pose_publisher; //for publishing ground truth pose
      ros::Publisher ardrone_gt_vel_publisher; //ground truth velocity in the body frame
      ros::Publisher ardrone_gt_acc_publisher; //ground truth acceleration in the body frame

      // Subscribers
      ros::Subscriber ardrone_takeoff_subscriber;
      ros::Subscriber ardrone_land_subscriber;
      ros::Subscriber ardrone_cmd_subscriber;
      
      ros::Subscriber ardrone_posctrl_subscriber;
      ros::Subscriber ardrone_imu_subscriber;
      ros::Subscriber ardrone_reset_subscriber;
      ros::Subscriber ardrone_switch_mode_subscriber;

      // Other variables
      double ardrone_time_after_cmd;
      bool ardrone_pos_ctrl;
      bool ardrone_vel_mode;
      unsigned int ardrone_state;

      bool ardrone_drain_battery;
      int  ardrone_battery_percentage;
      long double ardrone_max_flight_time;
      long double ardrone_current_flight_time;

      ros::Time state_stamp;
      
      // TODO: Change to pointers and init in constructor
      ignition::math::v6::Pose3<double> pose;
      ignition::math::v6::Vector3<double> euler;
      ignition::math::v6::Vector3<double> velocity;      
      ignition::math::v6::Vector3<double> acceleration;
      ignition::math::v6::Vector3<double> angular_velocity;
      ignition::math::v6::Vector3<double> position;

      ignition::math::v6::Vector3<double> inertia;

      common::Time last_time;      
      
      double mass;
      double max_force_;
      double motion_small_noise_;
      double motion_drift_noise_;
      double motion_drift_noise_time_;
      
      struct Controllers {
	 PIDController roll;
	 PIDController pitch;
	 PIDController yaw;
	 PIDController velocity_x;
	 PIDController velocity_y;
	 PIDController velocity_z;
	 PIDController pos_x;
	 PIDController pos_y;
	 PIDController pos_z;
      } ardrone_controllers;
      


      ros::CallbackQueue callback_queue_;
      
      geometry_msgs::Twist cmd_val;

      
   public:
      ARDronePlugin()
      {
	 ardrone_state     = LANDED_MODEL;
	 ardrone_pos_ctrl  = false;
	 ardrone_vel_mode  = false;

	 ardrone_drain_battery       = true;
	 ardrone_battery_percentage  = 100;
	 ardrone_max_flight_time     = 20 * 60.0; // 20 minutes in seconds. :-)
      }
      
      virtual ~ARDronePlugin()
      { 
	 ardrone_update_connection.reset();
	 ardrone_ros_node->shutdown();
      }
      

   protected:
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
	 // Topics:
	 std::string ardrone_cmd_topic;
	 std::string ardrone_takeoff_topic;
	 std::string ardrone_land_topic;
	 
	 std::string ardrone_navdata_topic;
	 std::string ardrone_imu_topic;
	 std::string ardrone_reset_topic;
	 std::string ardrone_switch_mode_topic;
	 
	 std::string ardrone_posctrl_topic;
	 
	 std::string ardrone_gt_topic; //ground truth
	 
	 // Those the real AR.Drone have these topics? TODO: Find out!
	 std::string ardrone_gt_acc_topic;
	 std::string ardrone_gt_pose_topic;
	 std::string ardrone_gt_vel_topic;
	 
	 if (!ros::isInitialized()) {
	    int argc = 0;
	    char **argv = NULL;
	    ros::init(argc, argv, "ardrone_plugin", ros::init_options::NoSigintHandler);
	 }
	 ROS_INFO("The ardrone_gazebo plugin is loading...");
	 
	 ardrone_model = _model;	 
	 ardrone_world = ardrone_model->GetWorld();
	 	 
	 if (!_sdf->HasElement("imuTopic")) {
	    ardrone_imu_topic.clear(); // TODO: Test when this will be executed. (Will this cause an error?)
	 } else {
	    ardrone_imu_topic = _sdf->GetElement("imuTopic")->Get<std::string>();
	 }

	 if (!_sdf->HasElement("bodyName")) {
	    ardrone_link = ardrone_model->GetLink();
	    ardrone_link_name = ardrone_link->GetName();
	    
	 } else {
	    ardrone_link_name = _sdf->GetElement("bodyName")->Get<std::string>();
	    ardrone_link = boost::dynamic_pointer_cast<physics::Link>(ardrone_model->GetLink(ardrone_link_name));
	 }

	 if (!ardrone_link) {
	    ROS_FATAL("ardrone_gazebo plugin error: Link to model does not exist!");
	    return;
	 }

	 if (!_sdf->HasElement("maxForce")) {
	    max_force_ = -1;
	 } else {
	    max_force_ = _sdf->GetElement("maxForce")->Get<double>();
	 }
	 
	 if (!_sdf->HasElement("motionSmallNoise")) {
	    motion_small_noise_ = 0;
	 } else {
	    motion_small_noise_ = _sdf->GetElement("motionSmallNoise")->Get<double>();
	 }
	 
	 if (!_sdf->HasElement("motionDriftNoise")) {
	    motion_drift_noise_ = 0;
	 } else {
	    motion_drift_noise_ = _sdf->GetElement("motionDriftNoise")->Get<double>();
	 }
	 
	 if (!_sdf->HasElement("motionDriftNoiseTime")) {
	    motion_drift_noise_time_ = 1.0;
	 } else {
	    motion_drift_noise_time_ = _sdf->GetElement("motionDriftNoiseTime")->Get<double>();
	 }

	 // Added namespace capability
	 if (!_sdf->HasElement("robotNamespace")) {
	    // If there isn't a namespace provided, default to the original ardrone topics.
	    ardrone_namespace = "";
	    
	    ardrone_cmd_topic         = "/cmd_vel";
	    ardrone_takeoff_topic     = "/ardrone/takeoff";
	    ardrone_land_topic        = "/ardrone/land";
	    ardrone_reset_topic       = "/ardrone/reset";
	    ardrone_posctrl_topic     = "/ardrone/posctrl";
	    ardrone_gt_topic          = "/ardrone/gt_pose";
	    ardrone_switch_mode_topic = "/ardrone/vel_mode";
	    ardrone_navdata_topic     = "/ardrone/navdata";
	    ardrone_imu_topic         = "/ardrone/imu";
	    
	    // TODO: Does the real drone have these topics?
	    ardrone_gt_pose_topic = "/ardrone/gt_pose";
	    ardrone_gt_vel_topic  = "/ardrone/gt_vel";
	    ardrone_gt_acc_topic  = "/ardrone/gt_acc";

	 } else {
	    ardrone_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
	    
	    if (ardrone_namespace.compare("/ardrone/") == 0) {
	       // Let's remove the namepace here to fake the original ardrone topics.
	       // TODO: This feels hacky, find a better solution?!?
	       ardrone_namespace = "";
	       
	       ardrone_cmd_topic         = "/cmd_vel";
	       ardrone_takeoff_topic     = "/ardrone/takeoff";
	       ardrone_land_topic        = "/ardrone/land";
	       ardrone_reset_topic       = "/ardrone/reset";
	       ardrone_posctrl_topic     = "/ardrone/posctrl";
	       ardrone_gt_topic          = "/ardrone/gt_pose";
	       ardrone_switch_mode_topic = "/ardrone/vel_mode";
	       ardrone_navdata_topic     = "/ardrone/navdata";
	       ardrone_imu_topic         = "/ardrone/imu";
	       
	       // TODO: Does the real drone have these topics?
	       ardrone_gt_pose_topic = "/ardrone/gt_pose";
	       ardrone_gt_vel_topic  = "/ardrone/gt_vel";
	       ardrone_gt_acc_topic  = "/ardrone/gt_acc";
	       
	    } else {
	       ardrone_cmd_topic         = "cmd_vel";
	       ardrone_takeoff_topic     = "takeoff";
	       ardrone_land_topic        = "land";
	       ardrone_reset_topic       = "reset";
	       ardrone_posctrl_topic     = "posctrl";
	       ardrone_gt_topic          = "gt_pose";
	       ardrone_switch_mode_topic = "vel_mode";
	       ardrone_navdata_topic     = "navdata";
	       ardrone_imu_topic         = "imu";
	       
	       // TODO: Does the real drone have these topics?
	       ardrone_gt_pose_topic =  "gt_pose";
	       ardrone_gt_vel_topic  =  "gt_vel";
	       ardrone_gt_acc_topic  =  "gt_acc";
	    }
	 }   
	 
	 // Get inertia and mass of the ardrone
	 inertia = ardrone_link->GetInertial()->PrincipalMoments();
	 mass = ardrone_link->GetInertial()->Mass();

	 // Create the node handler in the correct namespace.
	 ardrone_ros_node.reset(new ros::NodeHandle(ardrone_namespace));

	 // Subscribe command: control command
	 if (!ardrone_cmd_topic.empty()) {
	    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
	       ardrone_cmd_topic, 1,
	       boost::bind(&ARDronePlugin::CmdCallback, this, _1),
	       ros::VoidPtr(), &callback_queue_);
	    ardrone_cmd_subscriber = ardrone_ros_node->subscribe(ops);
    
	    if (ardrone_cmd_subscriber.getTopic() != "") {
	       ROS_INFO_NAMED("ardrone_gazebo", "Using ardrone_cmd_topic: %s.", ardrone_cmd_topic.c_str());	       
	    } else {
	       ROS_INFO("Cannot find the command topic!");
	    }
	 }
  
	 if (!ardrone_posctrl_topic.empty()) {
	    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Bool>(
	       ardrone_posctrl_topic, 1,
	       boost::bind(&ARDronePlugin::PosCtrlCallback, this, _1),
	       ros::VoidPtr(), &callback_queue_);
	    ardrone_posctrl_subscriber = ardrone_ros_node->subscribe(ops);
	    
	    if (ardrone_posctrl_subscriber.getTopic() != "") {
	       ROS_INFO("Found the position control topic.");
	    } else {
	       ROS_INFO("Cannot find the position control topic!");
	    }
	 }
  
	 // Subscribe imu
	 if (!ardrone_imu_topic.empty()) {
	    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
	       ardrone_imu_topic, 1,
	       boost::bind(&ARDronePlugin::ImuCallback, this, _1),
	       ros::VoidPtr(), &callback_queue_);
	    ardrone_imu_subscriber = ardrone_ros_node->subscribe(ops);
	    
	    if (ardrone_imu_subscriber.getTopic() != "") {
	       ROS_INFO_NAMED("ardrone_gazebo", "Using imu information from topic %s as source of orientation and angular velocity.", ardrone_imu_topic.c_str());
	    } else {
	       ROS_INFO("cannot find the IMU topic!");
	    }
	 }
	 
	 // Subscribe command: take off command
	 if (!ardrone_takeoff_topic.empty()) {
	    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
	       ardrone_takeoff_topic, 1,
	       boost::bind(&ARDronePlugin::TakeoffCallback, this, _1),
	       ros::VoidPtr(), &callback_queue_);

	    ardrone_takeoff_subscriber = ardrone_ros_node->subscribe(ops);
	    
	    if (ardrone_takeoff_subscriber.getTopic() != "") {
	       ROS_INFO("Found the takeoff topic.");
	    } else {
	       ROS_INFO("Cannot find the takeoff topic!");
	    }
	 }

	 // Subscribe command: land command
	 if (!ardrone_land_topic.empty()) {
	    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
	       ardrone_land_topic, 1,
	       boost::bind(&ARDronePlugin::LandCallback, this, _1),
	       ros::VoidPtr(), &callback_queue_);
	    ardrone_land_subscriber = ardrone_ros_node->subscribe(ops);
	 }

	 // Subscribe command: reset command
	 if (!ardrone_reset_topic.empty()) {
	    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
	       ardrone_reset_topic, 1,
	       boost::bind(&ARDronePlugin::ResetCallback, this, _1),
	       ros::VoidPtr(), &callback_queue_);
	    ardrone_reset_subscriber = ardrone_ros_node->subscribe(ops);
	 }
      
	 if (!ardrone_gt_topic.empty()) {
	    ardrone_gt_pose_publisher = ardrone_ros_node->advertise<geometry_msgs::Pose>(ardrone_gt_topic, 1024);
	 }
	 
	 ardrone_gt_vel_publisher = ardrone_ros_node->advertise<geometry_msgs::Twist>(ardrone_gt_vel_topic, 1024);
	 ardrone_gt_acc_publisher = ardrone_ros_node->advertise<geometry_msgs::Twist>(ardrone_gt_acc_topic, 1024);
	 
	 ardrone_navdata_publisher = ardrone_ros_node->advertise<ardrone_autonomy::Navdata>(ardrone_navdata_topic, 1024);
  
	 if (!ardrone_switch_mode_topic.empty()) {
	    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Bool>(
	       ardrone_switch_mode_topic, 1,
	       boost::bind(&ARDronePlugin::SwitchModeCallback, this, _1),
	       ros::VoidPtr(), &callback_queue_);
	    ardrone_switch_mode_subscriber = ardrone_ros_node->subscribe(ops);
	 }
      
	 LoadControllerSettings(_model, _sdf);
	 
	 Reset();
      
	 // Listen to the update event. This event is broadcast every simulation iteration.
	 ardrone_update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ARDronePlugin::Update, this));
      }
      
      virtual void LoadControllerSettings(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
	 ardrone_controllers.roll.Load(_sdf, "rollpitch");
	 ardrone_controllers.pitch.Load(_sdf, "rollpitch");
	 ardrone_controllers.yaw.Load(_sdf, "yaw");
	 
	 ardrone_controllers.velocity_x.Load(_sdf, "velocityXY");
	 ardrone_controllers.velocity_y.Load(_sdf, "velocityXY");
	 ardrone_controllers.velocity_z.Load(_sdf, "velocityZ");
	 
	 ardrone_controllers.pos_x.Load(_sdf, "positionXY");
	 ardrone_controllers.pos_y.Load(_sdf, "positionXY");
	 ardrone_controllers.pos_z.Load(_sdf, "positionZ");
      }
      
      virtual void Update()
      {
	 // Get new commands/state
	 callback_queue_.callAvailable();

	 // Get simulator time
	 common::Time sim_time = ardrone_world->SimTime();
	 double dt = (sim_time - last_time).Double();

	 if (dt == 0.0) {
	    return;
	 }
	 
	 UpdateState(dt);
	 UpdateDynamics(dt);
    
	 // Save last time stamp
	 last_time = sim_time;   
      }
      
      void UpdateDynamics(double dt)
      {
	 ignition::math::v6::Vector3<double> force, torque;
      
	 pose = ardrone_link->WorldPose();
	 angular_velocity = ardrone_link->WorldAngularVel();
	 euler = pose.Rot().Euler();
	 
	 acceleration = (ardrone_link->WorldLinearVel() - velocity) / dt;
	 velocity = ardrone_link->WorldLinearVel();
	 
	 // Publish the ground truth pose of the drone to the ROS topic
	 geometry_msgs::Pose gt_pose;
	 gt_pose.position.x = pose.Pos().X();
	 gt_pose.position.y = pose.Pos().Y();
	 gt_pose.position.z = pose.Pos().Z();
	 
	 gt_pose.orientation.w = pose.Rot().W();
	 gt_pose.orientation.x = pose.Rot().X();
	 gt_pose.orientation.y = pose.Rot().Y();
	 gt_pose.orientation.z = pose.Rot().Z();
	 ardrone_gt_pose_publisher.publish(gt_pose);
	 
	 // Convert the acceleration and velocity into the body frame
	 ignition::math::v6::Vector3 body_vel = pose.Rot().RotateVector(velocity);
	 ignition::math::v6::Vector3 body_acc = pose.Rot().RotateVector(acceleration);
	 
	 // Publish the velocity
	 geometry_msgs::Twist tw;
	 tw.linear.x = body_vel.X();
	 tw.linear.y = body_vel.Y();
	 tw.linear.z = body_vel.Z();
	 ardrone_gt_vel_publisher.publish(tw);
	 
	 // Publish the acceleration
	 tw.linear.x = body_acc.X();
	 tw.linear.y = body_acc.Y();
	 tw.linear.z = body_acc.Z();
	 ardrone_gt_acc_publisher.publish(tw);
	 
	 ignition::math::v6::Vector3 poschange = pose.Pos() - position;
	 position = pose.Pos();
	 
	 // Get gravity
	 ignition::math::v6::Vector3 gravity_body = pose.Rot().RotateVector(ardrone_world->Gravity());
	 double gravity = gravity_body.Length();
	 double load_factor = gravity * gravity / ardrone_world->Gravity().Dot(gravity_body);  // Get gravity
	 
	 // Rotate vectors to coordinate frames relevant for control
	 ignition::math::v6::Quaternion heading_quaternion(cos(euler[2]/2), 0.0, 0.0, sin(euler[2]/2));
	 ignition::math::v6::Vector3 velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
	 ignition::math::v6::Vector3 acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
	 ignition::math::v6::Vector3 angular_velocity_body = pose.Rot().RotateVectorReverse(angular_velocity);
	 
	 // Update controllers
	 force.Set(0.0, 0.0, 0.0);
	 torque.Set(0.0, 0.0, 0.0);
	 
	 if (ardrone_pos_ctrl) {
	    // Position control
	    if (ardrone_state == FLYING_MODEL) {
	       double vx = ardrone_controllers.pos_x.update(cmd_val.linear.x, position[0], poschange[0], dt);
	       double vy = ardrone_controllers.pos_y.update(cmd_val.linear.y, position[1], poschange[1], dt);
	       double vz = ardrone_controllers.pos_z.update(cmd_val.linear.z, position[2], poschange[2], dt);
	       
	       ignition::math::v6::Vector3 vb = heading_quaternion.RotateVectorReverse(ignition::math::v6::Vector3(vx,vy,vz));
	       
	       double pitch_command =  ardrone_controllers.velocity_x.update(vb[0], velocity_xy[0], acceleration_xy[0], dt) / gravity;
	       double roll_command  = -ardrone_controllers.velocity_y.update(vb[1], velocity_xy[1], acceleration_xy[1], dt) / gravity;
	       
	       torque[0] = inertia[0] *  ardrone_controllers.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
	       torque[1] = inertia[1] *  ardrone_controllers.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);            
	       force[2]  = mass       * (ardrone_controllers.velocity_z.update(vz,  velocity[2], acceleration[2], dt) + load_factor * gravity);
	    }
	    
	 } else {
	    // Normal control
	    if (ardrone_state == FLYING_MODEL) { //&& cmd_val.linear.x >= 0 && cmd_val.linear.y >= 0)
	       // Hovering
	       double pitch_command =  ardrone_controllers.velocity_x.update(cmd_val.linear.x, velocity_xy[0], acceleration_xy[0], dt) / gravity;
	       double roll_command  = -ardrone_controllers.velocity_y.update(cmd_val.linear.y, velocity_xy[1], acceleration_xy[1], dt) / gravity;
	       
	       torque[0] = inertia[0] *  ardrone_controllers.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
	       torque[1] = inertia[1] *  ardrone_controllers.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);
	    } else {
	       // Control by velocity
	       if (ardrone_vel_mode) {
		  double pitch_command =  ardrone_controllers.velocity_x.update(cmd_val.angular.x, velocity_xy[0], acceleration_xy[0], dt) / gravity;
		  double roll_command  = -ardrone_controllers.velocity_y.update(cmd_val.angular.y, velocity_xy[1], acceleration_xy[1], dt) / gravity;
		  
		  torque[0] = inertia[0] *  ardrone_controllers.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
		  torque[1] = inertia[1] *  ardrone_controllers.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);              
	       } else {
		  // Control by tilting
		  torque[0] = inertia[0] * ardrone_controllers.roll.update(cmd_val.angular.x, euler[0], angular_velocity_body[0], dt);
		  torque[1] = inertia[1] * ardrone_controllers.pitch.update(cmd_val.angular.y, euler[1], angular_velocity_body[1], dt);
	       }
	    }
	    
	    torque[2] = inertia[2] *  ardrone_controllers.yaw.update(cmd_val.angular.z, angular_velocity[2], 0, dt);
	    force[2]  = mass       * (ardrone_controllers.velocity_z.update(cmd_val.linear.z,  velocity[2], acceleration[2], dt) + load_factor * gravity);
	 }
	 
	 if (max_force_ > 0.0 && force[2] > max_force_) {
	    force[2] = max_force_;
	 }
	 
	 if (force[2] < 0.0) {
	    force[2] = 0.0;
	 }
	 
	 /* Process robot state information */
	 if(ardrone_state == LANDED_MODEL) {
	    // Nothing to do... Some quality R&R!
	    
	 } else if(ardrone_state == FLYING_MODEL) {
	    ardrone_link->AddRelativeForce(force);
	    ardrone_link->AddRelativeTorque(torque);
	    
	 } else if(ardrone_state == TAKINGOFF_MODEL) {
	    ardrone_link->AddRelativeForce(force*1.5);
	    ardrone_link->AddRelativeTorque(torque*1.5);
	    
	 } else if(ardrone_state == LANDING_MODEL) {
	    ardrone_link->AddRelativeForce(force*0.8);
	    ardrone_link->AddRelativeTorque(torque*0.8);  
	 }

	 if ((ardrone_state != LANDED_MODEL) /* && ardrone_drain_battery */ ) {
	    ardrone_current_flight_time += dt;
	    ardrone_battery_percentage = (int) 100 - ((100 / ardrone_max_flight_time) * ardrone_current_flight_time);
	 } else {
	    ardrone_current_flight_time = 0.0;
	 }
	 
	 ardrone_autonomy::Navdata navdata;
	 navdata.batteryPercent = ardrone_battery_percentage;
	 navdata.rotX = 0.; // TODO: pose.rot.GetRoll() / M_PI * 180.;
	 navdata.rotY = 0.; // TODO: pose.rot.GetPitch() / M_PI * 180.;
	 navdata.rotZ = 0.; // TODO: pose.rot.GetYaw() / M_PI * 180.;
	 navdata.altd = 0;
	 navdata.vx = 0; // TODO: 1000*velocity_xy.x;
	 navdata.vy = 0; // TODO: 1000*velocity_xy.y;
	 navdata.vz = 0; // TODO: 1000*velocity_xy.z;
	 navdata.ax = 0; // TODO: acceleration_xy.x/10;
	 navdata.ay = 0; // TODO: acceleration_xy.y/10;
	 navdata.az = 0; // TODO: acceleration_xy.z/10 + 1;
	 navdata.tm = ros::Time::now().toSec()*1000000; // FIXME what is the real drone sending here?
	 navdata.header.stamp = ros::Time::now();
	 navdata.header.frame_id = ardrone_link_name;
	 navdata.state = ardrone_state;
	 navdata.magX = 0;
	 navdata.magY = 0;
	 navdata.magZ = 0;
	 navdata.pressure = 0;
	 navdata.temp = 0;
	 navdata.wind_speed = 0.0;
	 navdata.wind_angle = 0.0;
	 navdata.wind_comp_angle = 0.0;
	 navdata.tags_count = 0;
	 
	 ardrone_navdata_publisher.publish(navdata);
      }
      
      void UpdateState(double dt)
      {
	 if (ardrone_state == TAKINGOFF_MODEL) {
	    ardrone_time_after_cmd += dt;
	    if (ardrone_time_after_cmd > 0.5) {
	       ardrone_state = FLYING_MODEL;
	       std::cout << "Entering flying model!" << std::endl;
	    }
	 } else if (ardrone_state == LANDING_MODEL) {
	    ardrone_time_after_cmd += dt;
	    if (ardrone_time_after_cmd > 1.0) {
	       ardrone_state = LANDED_MODEL;
	       std::cout << "Landed!" <<std::endl;
	    }
	 } else {
	    ardrone_time_after_cmd = 0;
	 }      
      }
      
      virtual void Reset()
      {
	 ardrone_controllers.roll.reset();
	 ardrone_controllers.pitch.reset();
	 ardrone_controllers.yaw.reset();
	 ardrone_controllers.velocity_x.reset();
	 ardrone_controllers.velocity_y.reset();
	 ardrone_controllers.velocity_z.reset();
	 
	 ardrone_link->SetForce(ignition::math::Vector3(0.0, 0.0, 0.0));
	 ardrone_link->SetTorque(ignition::math::v6::Vector3(0.0, 0.0, 0.0));
	 
	 // reset state
	 pose.Reset();
	 velocity.Set();
	 angular_velocity.Set();
	 acceleration.Set();
	 euler.Set();
	 state_stamp = ros::Time();
      }
      
      
   private:
      
      // callback functions for subscribers
      void CmdCallback(const geometry_msgs::TwistConstPtr& cmd)
      {
	 cmd_val = *cmd;
	 
	 static common::Time last_sim_time = ardrone_world->SimTime();
	 static double time_counter_for_drift_noise = 0;
	 static double drift_noise[4] = {0.0, 0.0, 0.0, 0.0};
	 
	 // Get simulator time
	 common::Time cur_sim_time = ardrone_world->SimTime();
	 double dt = (cur_sim_time - last_sim_time).Double();
      
	 // save last time stamp
	 last_sim_time = cur_sim_time;

	 // generate noise
	 if(time_counter_for_drift_noise > motion_drift_noise_time_) {
	    drift_noise[0] = 2*motion_drift_noise_*(drand48()-0.5);
	    drift_noise[1] = 2*motion_drift_noise_*(drand48()-0.5);
	    drift_noise[2] = 2*motion_drift_noise_*(drand48()-0.5);
	    drift_noise[3] = 2*motion_drift_noise_*(drand48()-0.5);
	    time_counter_for_drift_noise = 0.0;
	 }
	 time_counter_for_drift_noise += dt;

	 cmd_val.angular.x += drift_noise[0] + 2*motion_small_noise_ * (drand48() - 0.5);
	 cmd_val.angular.y += drift_noise[1] + 2*motion_small_noise_ * (drand48() - 0.5);
	 cmd_val.angular.z += drift_noise[3] + 2*motion_small_noise_ * (drand48() - 0.5);
	 cmd_val.linear.z  += drift_noise[2] + 2*motion_small_noise_ * (drand48() - 0.5);	  
      }
   
      void PosCtrlCallback(const std_msgs::BoolConstPtr& cmd)
      {
	 ardrone_pos_ctrl = cmd->data;
      }
      
      void ImuCallback(const sensor_msgs::ImuConstPtr& imu)
      {
	 // Directly read the quternion from the IMU data.
	 pose.Rot().Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
	 euler = pose.Rot().Euler();
	 angular_velocity = pose.Rot().RotateVector(ignition::math::v6::Vector3(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
      }
      
      void TakeoffCallback(const std_msgs::EmptyConstPtr& msg)
      {
	 if(ardrone_state == LANDED_MODEL) {
	    ardrone_state = TAKINGOFF_MODEL;
	    ardrone_time_after_cmd = 0;
	    ROS_INFO("The drone will now take off.");
	 }
      }
      
      void LandCallback(const std_msgs::EmptyConstPtr& msg)
      {
	 if(ardrone_state == FLYING_MODEL || ardrone_state == TAKINGOFF_MODEL) {
	    ardrone_state = LANDING_MODEL;
	    ardrone_time_after_cmd = 0;
	    ROS_INFO("The drone will now land.");
	 }
      }
      
      void ResetCallback(const std_msgs::EmptyConstPtr& msg)
      {
	 ROS_INFO("The drone doesn't have a reset feature... It's a TODO. :-)");
      }
      
      void SwitchModeCallback(const std_msgs::BoolConstPtr& msg)
      {
	 ardrone_vel_mode = msg->data;
      }
      
   };

   GZ_REGISTER_MODEL_PLUGIN(ARDronePlugin)
}

#endif // ARDRONE_GAZEBO_PLUGIN_H
