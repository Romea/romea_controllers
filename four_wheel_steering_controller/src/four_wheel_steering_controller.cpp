#include <cmath>

#include <tf/transform_datatypes.h>

#include <boost/assign.hpp>

#include <four_wheel_steering_controller/four_wheel_steering_controller.h>
#include <urdf_vehicle_kinematic/urdf_vehicle_kinematic.h>

namespace four_wheel_steering_controller{

  FourWheelSteeringController::FourWheelSteeringController()
    : open_loop_(false)
    , command_struct_()
    , command_struct_four_wheel_steering_()
    , track_(0.0)
    , wheel_radius_(0.0)
    , wheel_base_(0.0)
    , cmd_vel_timeout_(0.5)
    , base_frame_id_("base_link")
    , enable_odom_tf_(true)
    , enable_twist_cmd_(false)
  {
  }

  bool FourWheelSteeringController::initRequest(hardware_interface::RobotHW *const robot_hw,
                         ros::NodeHandle& root_nh,
                         ros::NodeHandle& ctrlr_nh,
                         std::set<std::string> &claimed_resources)
  {
    if (state_ != CONSTRUCTED)
    {
      ROS_ERROR("The four_wheel_steering controller could not be created.");
      return false;
    }

    hardware_interface::PositionJointInterface *const pos_joint_hw = robot_hw->get<hardware_interface::PositionJointInterface>();
    hardware_interface::VelocityJointInterface *const vel_joint_hw = robot_hw->get<hardware_interface::VelocityJointInterface>();

    if (pos_joint_hw == NULL)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the hardware_interface::RobotHW class.",
                hardware_interface::internal::demangledTypeName<hardware_interface::PositionJointInterface>().c_str());
      return false;
    }
    else if (vel_joint_hw == NULL)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the hardware_interface::RobotHW class.",
                hardware_interface::internal::demangledTypeName<hardware_interface::PositionJointInterface>().c_str());
      return false;
    }

    pos_joint_hw->clearClaims();
    vel_joint_hw->clearClaims();
    if(init(pos_joint_hw, vel_joint_hw, root_nh, ctrlr_nh) == false)
    {
      ROS_ERROR("Failed to initialize the controller");
      return false;
    }

    claimed_resources.clear();
    const std::set<std::string> claims_pos = pos_joint_hw->getClaims();
    claimed_resources.insert(claims_pos.begin(), claims_pos.end());
    pos_joint_hw->clearClaims();

    const std::set<std::string> claims_vel = vel_joint_hw->getClaims();
    claimed_resources.insert(claims_vel.begin(), claims_vel.end());
    vel_joint_hw->clearClaims();

    state_ = INITIALIZED;
    return true;
  }

  bool FourWheelSteeringController::init(hardware_interface::PositionJointInterface* hw_pos,
                                 hardware_interface::VelocityJointInterface* hw_vel,
                                 ros::NodeHandle& root_nh,
                                 ros::NodeHandle &controller_nh)
  {
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    // Get joint names from the parameter server
    std::vector<std::string> front_wheel_names, rear_wheel_names;
    if (!getWheelNames(controller_nh, "front_wheel", front_wheel_names) ||
        !getWheelNames(controller_nh, "rear_wheel", rear_wheel_names))
    {
      return false;
    }

    if (front_wheel_names.size() != rear_wheel_names.size())
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#front wheels (" << front_wheel_names.size() << ") != " <<
          "#rear wheels (" << rear_wheel_names.size() << ").");
      return false;
    }
    else if (front_wheel_names.size() != 2)
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#two wheels by axle (left and right) is needed; now : "<<front_wheel_names.size()<<" .");
      return false;
    }
    else
    {
      front_wheel_joints_.resize(front_wheel_names.size());
      rear_wheel_joints_.resize(front_wheel_names.size());
    }

    // Get steering joint names from the parameter server
    std::vector<std::string> front_steering_names, rear_steering_names;
    if (!getWheelNames(controller_nh, "front_steering", front_steering_names) ||
        !getWheelNames(controller_nh, "rear_steering", rear_steering_names))
    {
      return false;
    }

    if (front_steering_names.size() != rear_steering_names.size())
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#left steerings (" << front_steering_names.size() << ") != " <<
          "#right steerings (" << rear_steering_names.size() << ").");
      return false;
    }
    else if (front_steering_names.size() != 2)
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#two steering by axle (left and right) is needed; now : "<<front_steering_names.size()<<" .");
      return false;
    }
    else
    {
      front_steering_joints_.resize(front_steering_names.size());
      rear_steering_joints_.resize(front_steering_names.size());
    }

    // Odometry related:
    double publish_rate;
    controller_nh.param("publish_rate", publish_rate, 50.0);
    ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                          << publish_rate << "Hz.");
    publish_period_ = ros::Duration(1.0 / publish_rate);

    controller_nh.param("open_loop", open_loop_, open_loop_);

    int velocity_rolling_window_size = 10;
    controller_nh.param("velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
    ROS_INFO_STREAM_NAMED(name_, "Velocity rolling window size of "
                          << velocity_rolling_window_size << ".");

    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    // Twist command related:
    controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
    ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                          << cmd_vel_timeout_ << "s.");

    controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

    controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
    ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

    controller_nh.param("enable_twist_cmd", enable_twist_cmd_, enable_twist_cmd_);
    ROS_INFO_STREAM_NAMED(name_, "Twist cmd is " << (enable_twist_cmd_?"enabled":"disabled")<<" (default is four_wheel_steering)");

    // Velocity and acceleration limits:
    controller_nh.param("linear/x/has_velocity_limits"    , limiter_lin_.has_velocity_limits    , limiter_lin_.has_velocity_limits    );
    controller_nh.param("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits, limiter_lin_.has_acceleration_limits);
    controller_nh.param("linear/x/max_velocity"           , limiter_lin_.max_velocity           ,  limiter_lin_.max_velocity          );
    controller_nh.param("linear/x/min_velocity"           , limiter_lin_.min_velocity           , -limiter_lin_.max_velocity          );
    controller_nh.param("linear/x/max_acceleration"       , limiter_lin_.max_acceleration       ,  limiter_lin_.max_acceleration      );
    controller_nh.param("linear/x/min_acceleration"       , limiter_lin_.min_acceleration       , -limiter_lin_.max_acceleration      );

    controller_nh.param("angular/z/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
    controller_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
    controller_nh.param("angular/z/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
    controller_nh.param("angular/z/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );

    // If either parameter is not available, we need to look up the value in the URDF
    bool lookup_track = !controller_nh.getParam("track", track_);
    bool lookup_wheel_radius = !controller_nh.getParam("wheel_radius", wheel_radius_);
    bool lookup_wheel_base = !controller_nh.getParam("wheel_base", wheel_base_);

    urdf_vehicle_kinematic::UrdfVehicleKinematic uvk(root_nh, base_frame_id_);
    if(lookup_track)
      if(!uvk.getDistanceBetweenJoints(front_wheel_names[0], front_wheel_names[1], track_))
        return false;
      else
        controller_nh.setParam("track",track_);
    if(lookup_wheel_radius)
      if(!uvk.getJointRadius(front_wheel_names[0], wheel_radius_))
        return false;
      else
        controller_nh.setParam("wheel_radius",wheel_radius_);
    if(lookup_wheel_base)
      if(!uvk.getDistanceBetweenJoints(front_wheel_names[0], rear_wheel_names[0], wheel_base_))
        return false;
      else
        controller_nh.setParam("wheel_base",wheel_base_);

    // Regardless of how we got the separation and radius, use them
    // to set the odometry parameters
    const double ws = track_;
    const double wr = wheel_radius_;
    const double wb = wheel_base_;
    odometry_.setWheelParams(ws, wr, wb);
    ROS_INFO_STREAM_NAMED(name_,
                          "Odometry params : wheel separation " << ws
                          << ", wheel radius " << wr
                          << ", wheel base " << wb);

    setOdomPubFields(root_nh, controller_nh);

    // Get the joint object to use in the realtime loop
    for (int i = 0; i < front_wheel_joints_.size(); ++i)
    {
      ROS_INFO_STREAM_NAMED(name_,
                            "Adding left wheel with joint name: " << front_wheel_names[i]
                            << " and right wheel with joint name: " << rear_wheel_names[i]);
      front_wheel_joints_[i] = hw_vel->getHandle(front_wheel_names[i]);  // throws on failure
      rear_wheel_joints_[i] = hw_vel->getHandle(rear_wheel_names[i]);  // throws on failure
    }

    // Get the steering joint object to use in the realtime loop
    for (int i = 0; i < front_steering_joints_.size(); ++i)
    {
      ROS_INFO_STREAM_NAMED(name_,
                            "Adding left steering with joint name: " << front_steering_names[i]
                            << " and right steering with joint name: " << rear_steering_names[i]);
      front_steering_joints_[i] = hw_pos->getHandle(front_steering_names[i]);  // throws on failure
      rear_steering_joints_[i] = hw_pos->getHandle(rear_steering_names[i]);  // throws on failure
    }

    if(enable_twist_cmd_ == true)
      sub_command_ = controller_nh.subscribe("cmd_vel", 1, &FourWheelSteeringController::cmdVelCallback, this);
    else
      sub_command_four_wheel_steering_ = controller_nh.subscribe("cmd_four_wheel_steering", 1, &FourWheelSteeringController::cmdFourWheelSteeringCallback, this);

    return true;
  }

  void FourWheelSteeringController::update(const ros::Time& time, const ros::Duration& period)
  {
    // COMPUTE AND PUBLISH ODOMETRY
    if (open_loop_)
    {
      odometry_.updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, time);
    }
    else
    {
      const double fl_speed = front_wheel_joints_[0].getVelocity();
      const double fr_speed = front_wheel_joints_[1].getVelocity();
      const double rl_speed = rear_wheel_joints_[0].getVelocity();
      const double rr_speed = rear_wheel_joints_[1].getVelocity();
      if (std::isnan(fl_speed) || std::isnan(fr_speed)
          || std::isnan(rl_speed) || std::isnan(rr_speed))
        return;

      const double fl_steering = front_steering_joints_[0].getPosition();
      const double fr_steering = front_steering_joints_[1].getPosition();
      const double rl_steering = rear_steering_joints_[0].getPosition();
      const double rr_steering = rear_steering_joints_[1].getPosition();
      if (std::isnan(fl_steering) || std::isnan(fr_steering)
          || std::isnan(rl_steering) || std::isnan(rr_steering))
        return;
      double front_steering_pos = atan2(2, 1/tan(fl_steering)
                                          + 1/tan(fr_steering));
      double rear_steering_pos = atan2(2, 1/tan(rl_steering)
                                          + 1/tan(rr_steering));

      ROS_DEBUG_STREAM("rl_speed "<<rl_speed<<" front_steering_pos "<<front_steering_pos<<" rear_steering_pos "<<rear_steering_pos);
      // Estimate linear and angular velocity using joint information
      odometry_.update(fl_speed, fr_speed, rl_speed, rr_speed,
                       front_steering_pos, rear_steering_pos, time);
    }

    // Publish odometry message
    if (last_state_publish_time_ + publish_period_ < time)
    {
      last_state_publish_time_ += publish_period_;
      // Compute and store orientation info
      const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

      // Populate odom message and publish
      if (odom_pub_->trylock())
      {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
        odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
        odom_pub_->msg_.pose.pose.orientation = orientation;
        odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinearX();
        odom_pub_->msg_.twist.twist.linear.y  = odometry_.getLinearY();
        odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
        odom_pub_->unlockAndPublish();
      }

      // Publish tf /odom frame
      if (enable_odom_tf_ && tf_odom_pub_->trylock())
      {
        geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = odometry_.getX();
        odom_frame.transform.translation.y = odometry_.getY();
        odom_frame.transform.rotation = orientation;
        tf_odom_pub_->unlockAndPublish();
      }
    }

    // MOVE ROBOT
    // Retreive current velocity command and time step:
    Commands curr_cmd;
    if(enable_twist_cmd_ == false)
      curr_cmd = *(command_four_wheel_steering_.readFromRT());
    else
      curr_cmd = *(command_.readFromRT());

    const double dt = (time - curr_cmd.stamp).toSec();

    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_)
    {
      curr_cmd.lin = 0.0;
      curr_cmd.ang = 0.0;
      curr_cmd.front_steering = 0.0;
      curr_cmd.rear_steering = 0.0;
    }

    // Limit velocities and accelerations:
    const double cmd_dt(period.toSec());

    limiter_lin_.limit(curr_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmd_dt);
    limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

    last1_cmd_ = last0_cmd_;
    last0_cmd_ = curr_cmd;


    const double angular_speed = odometry_.getAngular();
    const double ws = track_;
    const double wr = wheel_radius_;

    ROS_DEBUG_STREAM("angular_speed "<<angular_speed<<" curr_cmd.lin "<<curr_cmd.lin<< " wr "<<wr);
    // Compute wheels velocities:
    const double vel_left_front  = copysign(1.0, curr_cmd.lin) * sqrt((pow(curr_cmd.lin - angular_speed*ws/2,2)
                                                                       +pow(wheel_base_*angular_speed/2.0,2)))/wr;
    const double vel_right_front = copysign(1.0, curr_cmd.lin) * sqrt((pow(curr_cmd.lin + angular_speed*ws/2,2)
                                                                       +pow(wheel_base_*angular_speed/2.0,2)))/wr;
    const double vel_left_rear = copysign(1.0, curr_cmd.lin) * sqrt((pow(curr_cmd.lin - angular_speed*ws/2,2)
                                                                     +pow(wheel_base_*angular_speed/2.0,2)))/wr;
    const double vel_right_rear = copysign(1.0, curr_cmd.lin) * sqrt((pow(curr_cmd.lin + angular_speed*ws/2,2)
                                                                      +pow(wheel_base_*angular_speed/2.0,2)))/wr;
    // Set wheels velocities:
    if(front_wheel_joints_.size() == 2 && rear_wheel_joints_.size() == 2)
    {
      front_wheel_joints_[0].setCommand(vel_left_front);
      rear_wheel_joints_[0].setCommand(vel_right_front);
      front_wheel_joints_[1].setCommand(vel_left_rear);
      rear_wheel_joints_[1].setCommand(vel_right_rear);
    }

    double front_left_steering = 0, front_right_steering = 0;
    double rear_left_steering = 0, rear_right_steering = 0;
    if(enable_twist_cmd_ == true)
    {
      if(fabs(2.0*odometry_.getLinear()) > fabs(curr_cmd.ang*track_))
      {
        front_left_steering = atan(curr_cmd.ang*wheel_base_ /
                                    (2.0*odometry_.getLinear() - curr_cmd.ang*track_));
        front_right_steering = atan(curr_cmd.ang*wheel_base_ /
                                     (2.0*odometry_.getLinear() + curr_cmd.ang*track_));
        rear_left_steering = -atan(curr_cmd.ang*wheel_base_ /
                                    (2.0*odometry_.getLinear() - curr_cmd.ang*track_));
        rear_right_steering = -atan(curr_cmd.ang*wheel_base_ /
                                     (2.0*odometry_.getLinear() + curr_cmd.ang*track_));
      }
      else
      {
        front_left_steering = copysign(M_PI_2, curr_cmd.ang);
        front_right_steering = copysign(M_PI_2, curr_cmd.ang);
        rear_left_steering = copysign(M_PI_2, -curr_cmd.ang);
        rear_right_steering = copysign(M_PI_2, -curr_cmd.ang);
      }

    }
    else
    {
      front_left_steering = curr_cmd.front_steering;
      front_right_steering = curr_cmd.front_steering;
      rear_left_steering = curr_cmd.rear_steering;
      rear_right_steering = curr_cmd.rear_steering;
    }

    /// TODO check limits to not apply the same steering on right and left when saturated !

    if(front_steering_joints_.size() == 2 && rear_steering_joints_.size() == 2)
    {
      ROS_DEBUG_STREAM("front_left_steering "<<front_left_steering<<" rear_right_steering "<<rear_right_steering);
      front_steering_joints_[0].setCommand(front_left_steering);
      front_steering_joints_[1].setCommand(front_right_steering);
      rear_steering_joints_[0].setCommand(rear_left_steering);
      rear_steering_joints_[1].setCommand(rear_right_steering);
    }
  }

  void FourWheelSteeringController::starting(const ros::Time& time)
  {
    brake();

    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;

    odometry_.init(time);
  }

  void FourWheelSteeringController::stopping(const ros::Time& /*time*/)
  {
    brake();
  }

  void FourWheelSteeringController::brake()
  {
    const double vel = 0.0;
    for (size_t i = 0; i < front_wheel_joints_.size(); ++i)
    {
      front_wheel_joints_[i].setCommand(vel);
      rear_wheel_joints_[i].setCommand(vel);
    }

    const double pos = 0.0;
    for (size_t i = 0; i < front_steering_joints_.size(); ++i)
    {
      front_steering_joints_[i].setCommand(pos);
      rear_steering_joints_[i].setCommand(pos);
    }
  }

  void FourWheelSteeringController::cmdVelCallback(const geometry_msgs::Twist& command)
  {
    if (isRunning())
    {
      command_struct_.ang   = command.angular.z;
      command_struct_.lin   = command.linear.x;
      command_struct_.stamp = ros::Time::now();
      command_.writeFromNonRT (command_struct_);
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Added values to command. "
                             << "Ang: "   << command_struct_.ang << ", "
                             << "Lin: "   << command_struct_.lin << ", "
                             << "Stamp: " << command_struct_.stamp);
    }
    else
    {
      ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
  }

  void FourWheelSteeringController::cmdFourWheelSteeringCallback(const four_wheel_steering_msgs::FourWheelSteeringDrive& command)
  {
    if (isRunning())
    {
      command_struct_four_wheel_steering_.front_steering   = command.front_steering_angle;
      command_struct_four_wheel_steering_.rear_steering   = command.rear_steering_angle;
      command_struct_four_wheel_steering_.lin   = command.speed;
      command_struct_four_wheel_steering_.stamp = ros::Time::now();
      command_four_wheel_steering_.writeFromNonRT (command_struct_four_wheel_steering_);
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Added values to command. "
                             << "Steering front : "   << command_struct_four_wheel_steering_.front_steering << ", "
                             << "Steering rear : "   << command_struct_four_wheel_steering_.rear_steering << ", "
                             << "Lin: "   << command_struct_four_wheel_steering_.lin << ", "
                             << "Stamp: " << command_struct_four_wheel_steering_.stamp);
    }
    else
    {
      ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
  }

  bool FourWheelSteeringController::getWheelNames(ros::NodeHandle& controller_nh,
                              const std::string& wheel_param,
                              std::vector<std::string>& wheel_names)
  {
      XmlRpc::XmlRpcValue wheel_list;
      if (!controller_nh.getParam(wheel_param, wheel_list))
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Couldn't retrieve wheel param '" << wheel_param << "'.");
        return false;
      }

      if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        if (wheel_list.size() == 0)
        {
          ROS_ERROR_STREAM_NAMED(name_,
              "Wheel param '" << wheel_param << "' is an empty list");
          return false;
        }

        for (int i = 0; i < wheel_list.size(); ++i)
        {
          if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
          {
            ROS_ERROR_STREAM_NAMED(name_,
                "Wheel param '" << wheel_param << "' #" << i <<
                " isn't a string.");
            return false;
          }
        }

        wheel_names.resize(wheel_list.size());
        for (int i = 0; i < wheel_list.size(); ++i)
        {
          wheel_names[i] = static_cast<std::string>(wheel_list[i]);
          //ROS_INFO_STREAM("wheel name "<<i<<" " << wheel_names[i]);
        }
      }
      else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        wheel_names.push_back(wheel_list);
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Wheel param '" << wheel_param <<
            "' is neither a list of strings nor a string.");
        return false;
      }
      return true;
  }

  void FourWheelSteeringController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    // Get and check params for covariances
    XmlRpc::XmlRpcValue pose_cov_list;
    controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
      ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
      ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = "odom";
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = boost::assign::list_of
        (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
    odom_pub_->msg_.twist.twist.linear.y  = 0;
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = boost::assign::list_of
        (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
  }

} // namespace four_wheel_steering_controller
