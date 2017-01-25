#include <cmath>

#include <tf/transform_datatypes.h>

#include <urdf_parser/urdf_parser.h>

#include <boost/assign.hpp>

#include <urdf_vehicle_kinematic/urdf_vehicle_kinematic.h>

#include <ackermann_controller/ackermann_controller.h>

namespace ackermann_controller{

  AckermannController::AckermannController()
    : open_loop_(false)
    , command_struct_()
    , command_struct_ackermann_()
    , track_(0.0)
    , front_wheel_radius_(0.0)
    , rear_wheel_radius_(0.0)
    , steering_limit_(0.0)
    , wheel_base_(0.0)
    , cmd_vel_timeout_(0.5)
    , base_frame_id_("base_link")
    , enable_odom_tf_(true)
    , enable_twist_cmd_(false)
  {
  }

  bool AckermannController::initRequest(hardware_interface::RobotHW *const robot_hw,
                         ros::NodeHandle& root_nh,
                         ros::NodeHandle& ctrlr_nh,
                         std::set<std::string> &claimed_resources)
  {
    if (state_ != CONSTRUCTED)
    {
      ROS_ERROR("The ackermann controller could not be created.");
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

  bool AckermannController::init(hardware_interface::PositionJointInterface* hw_pos,
                                 hardware_interface::VelocityJointInterface* hw_vel,
                                 ros::NodeHandle& root_nh,
                                 ros::NodeHandle &controller_nh)
  {
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    // Get joint names from the parameter server
    std::vector<std::string> front_wheel_names, rear_wheel_names;
    if (!getWheelNames(controller_nh, "front_wheel", front_wheel_names) or
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
    std::vector<std::string> front_steering_names;
    if (!getWheelNames(controller_nh, "front_steering", front_steering_names))
    {
      return false;
    }

    if (front_steering_names.size() != 2)
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#two steering by axle (left and right) is needed; now : "<<front_steering_names.size()<<" .");
      return false;
    }
    else
    {
      front_steering_joints_.resize(front_steering_names.size());
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
    ROS_INFO_STREAM_NAMED(name_, "Twist cmd is " << (enable_twist_cmd_?"enabled":"disabled")<<" (default is ackermann)");

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
    bool lookup_front_wheel_radius = !controller_nh.getParam("front_wheel_radius", front_wheel_radius_);
    bool lookup_rear_wheel_radius = !controller_nh.getParam("rear_wheel_radius", rear_wheel_radius_);
    bool lookup_wheel_base = !controller_nh.getParam("wheel_base", wheel_base_);

    urdf_vehicle_kinematic::UrdfVehicleKinematic uvk(root_nh, base_frame_id_);
    if(lookup_track)
    {
      if(!uvk.getDistanceBetweenJoints(front_wheel_names[0], front_wheel_names[1], track_))
        return false;
      else
        controller_nh.setParam("track",track_);
    }
    if(lookup_front_wheel_radius)
    {
      if(!uvk.getJointRadius(front_wheel_names[0], front_wheel_radius_))
        return false;
      else
        controller_nh.setParam("front_wheel_radius",front_wheel_radius_);
    }
    if(lookup_rear_wheel_radius)
    {
      if(!uvk.getJointRadius(rear_wheel_names[0], rear_wheel_radius_))
        return false;
      else
        controller_nh.setParam("rear_wheel_radius",rear_wheel_radius_);
    }
    if(lookup_wheel_base)
    {
      if(!uvk.getDistanceBetweenJoints(front_wheel_names[0], rear_wheel_names[0], wheel_base_))
        return false;
      else
        controller_nh.setParam("wheel_base",wheel_base_);
    }

    if(!uvk.getJointSteeringLimits(front_steering_names[0], steering_limit_))
      return false;
    else
    {
      controller_nh.setParam("steering_limit",steering_limit_);
    }

    // Regardless of how we got the separation and radius, use them
    // to set the odometry parameters
    const double ws = track_;
    const double wb = wheel_base_;
    odometry_.setWheelParams(ws, front_wheel_radius_, rear_wheel_radius_, wb);
    ROS_INFO_STREAM_NAMED(name_,
                          "Odometry params : wheel separation " << ws
                          << ", front wheel radius " << front_wheel_radius_
                          << ", rear wheel radius " << rear_wheel_radius_
                          << ", wheel base " << wb);

    setOdomPubFields(root_nh, controller_nh);

    // Get the joint object to use in the realtime loop
    for (int i = 0; i < front_wheel_joints_.size(); ++i)
    {
      ROS_INFO_STREAM_NAMED(name_,
                            "Adding front wheel with joint name: " << front_wheel_names[i]
                            << " and rear wheel with joint name: " << rear_wheel_names[i]);
      front_wheel_joints_[i] = hw_vel->getHandle(front_wheel_names[i]);  // throws on failure
      rear_wheel_joints_[i] = hw_vel->getHandle(rear_wheel_names[i]);  // throws on failure
    }

    // Get the steering joint object to use in the realtime loop
    for (int i = 0; i < front_steering_joints_.size(); ++i)
    {
      ROS_INFO_STREAM_NAMED(name_,
                            "Adding front steering with joint name: " << front_steering_names[i]);
      front_steering_joints_[i] = hw_pos->getHandle(front_steering_names[i]);  // throws on failure
    }

    if(enable_twist_cmd_ == true)
      sub_command_ = controller_nh.subscribe("cmd_vel", 1, &AckermannController::cmdVelCallback, this);
    else
      sub_command_ackermann_ = controller_nh.subscribe("cmd_ackermann", 1, &AckermannController::cmdAckermannCallback, this);

    return true;
  }

  void AckermannController::update(const ros::Time& time, const ros::Duration& period)
  {
    // COMPUTE AND PUBLISH ODOMETRY
    if (open_loop_)
    {
      odometry_.updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, time);
    }
    else
    {
      double front_pos  = 0.0;
      double rear_pos = 0.0;
      double front_vel  = 0.0;
      double rear_vel = 0.0;
      for (size_t i = 0; i < front_wheel_joints_.size(); ++i)
      {
        const double fp = front_wheel_joints_[i].getPosition();
        const double rp = rear_wheel_joints_[i].getPosition();
        if (std::isnan(fp) || std::isnan(rp))
          return;
        front_pos  += fp;
        rear_pos += rp;

        const double ls = front_wheel_joints_[i].getVelocity();
        const double rs = rear_wheel_joints_[i].getVelocity();
        if (std::isnan(ls) || std::isnan(rs))
          return;
        front_vel  += ls;
        rear_vel += rs;
      }
      front_pos  /= front_wheel_joints_.size();
      rear_pos /= front_wheel_joints_.size();
      front_vel  /= front_wheel_joints_.size();
      rear_vel /= front_wheel_joints_.size();

      double front_left_steering_pos = 0.0, front_right_steering_pos = 0.0;
      if (front_steering_joints_.size() == 2)
      {
        front_left_steering_pos = front_steering_joints_[0].getPosition();
        front_right_steering_pos = front_steering_joints_[1].getPosition();
      }
      double front_steering_pos = 0.0;
      if(fabs(front_left_steering_pos) > 0.001 || fabs(front_right_steering_pos) > 0.001)
      {
        front_steering_pos = atan(2*tan(front_left_steering_pos)*tan(front_right_steering_pos)/
                                        (tan(front_left_steering_pos) + tan(front_right_steering_pos)));
      }
      ROS_DEBUG_STREAM_THROTTLE(1, "front_left_steering_pos "<<front_left_steering_pos<<" front_right_steering_pos "<<front_right_steering_pos<<" front_steering_pos "<<front_steering_pos);
      // Estimate linear and angular velocity using joint information
      odometry_.update(front_pos, front_vel, rear_pos, rear_vel, front_steering_pos, time);
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
        odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinear();
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
      curr_cmd = *(command_ackermann_.readFromRT());
    else
      curr_cmd = *(command_.readFromRT());

    const double dt = (time - curr_cmd.stamp).toSec();

    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_)
    {
      curr_cmd.lin = 0.0;
      curr_cmd.ang = 0.0;
      curr_cmd.steering = 0.0;
    }

    // Limit velocities and accelerations:
    const double cmd_dt(period.toSec());

    limiter_lin_.limit(curr_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmd_dt);
    limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

    last1_cmd_ = last0_cmd_;
    last0_cmd_ = curr_cmd;


    const double angular_speed = odometry_.getAngular();

    ROS_DEBUG_STREAM("angular_speed "<<angular_speed<<" curr_cmd.lin "<<curr_cmd.lin);
    // Compute wheels velocities:
    // TODO should use angular cmd instead of angular odom and differenciate twist and ackermann cmd
    const double vel_left_front  = copysign(1.0, curr_cmd.lin) *
                                   sqrt((pow((curr_cmd.lin - angular_speed*track_/2),2)
                                         +pow(wheel_base_*angular_speed,2)))/front_wheel_radius_;
    const double vel_right_front = copysign(1.0, curr_cmd.lin) *
                                   sqrt((pow((curr_cmd.lin + angular_speed*track_/2),2)+
                                         pow(wheel_base_*angular_speed,2)))/front_wheel_radius_;
    const double vel_left_rear = (curr_cmd.lin - angular_speed*track_/2)/rear_wheel_radius_;
    const double vel_right_rear = (curr_cmd.lin + angular_speed*track_/2)/rear_wheel_radius_;
    // Set wheels velocities:
    if(front_wheel_joints_.size() == 2 && rear_wheel_joints_.size() == 2)
    {
      front_wheel_joints_[0].setCommand(vel_left_front);
      rear_wheel_joints_[0].setCommand(vel_left_rear);
      front_wheel_joints_[1].setCommand(vel_right_front);
      rear_wheel_joints_[1].setCommand(vel_right_rear);
    }

    double front_left_steering = 0, front_right_steering = 0;
    if(enable_twist_cmd_ == true)
    {
      if(fabs(odometry_.getLinear()) > fabs(curr_cmd.ang*track_/2.0))
      {
        front_left_steering = atan(curr_cmd.ang*wheel_base_ /
                                    (odometry_.getLinear() - curr_cmd.ang*track_/2.0));
        front_right_steering = atan(curr_cmd.ang*wheel_base_ /
                                     (odometry_.getLinear() + curr_cmd.ang*track_/2.0));
      }
      else
      {
        if(fabs(curr_cmd.ang) > std::numeric_limits<double>::epsilon())
        {
          // TODO CIR is not converging, do not put left and right to the same limit
          front_left_steering = copysign(steering_limit_, curr_cmd.ang*odometry_.getLinear());
          front_right_steering = copysign(steering_limit_, curr_cmd.ang*odometry_.getLinear());
        }
        else
        {
          front_left_steering = 0.0;
          front_right_steering = 0.0;
        }
      }
    }
    else
    {
      front_left_steering = atan2(tan(curr_cmd.steering),
                                  1 - tan(curr_cmd.steering)*track_/(2*wheel_base_));
      front_right_steering = atan2(tan(curr_cmd.steering),
                                   1 + tan(curr_cmd.steering)*track_/(2*wheel_base_));
    }

    /// TODO check limits to not apply the same steering on right and left when saturated !

    if(front_steering_joints_.size() == 2)
    {
      ROS_DEBUG_STREAM("front_left_steering "<<front_left_steering<<"front_right_steering "<<front_right_steering);
      front_steering_joints_[0].setCommand(front_left_steering);
      front_steering_joints_[1].setCommand(front_right_steering);
    }
  }

  void AckermannController::starting(const ros::Time& time)
  {
    brake();

    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;

    odometry_.init(time);
  }

  void AckermannController::stopping(const ros::Time& /*time*/)
  {
    brake();
  }

  void AckermannController::brake()
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
    }
  }

  void AckermannController::cmdVelCallback(const geometry_msgs::Twist& command)
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

  void AckermannController::cmdAckermannCallback(const ackermann_msgs::AckermannDrive& command)
  {
    if (isRunning())
    {
      command_struct_ackermann_.steering   = command.steering_angle;
      command_struct_ackermann_.lin   = command.speed;
      command_struct_ackermann_.stamp = ros::Time::now();
      command_ackermann_.writeFromNonRT (command_struct_ackermann_);
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Added values to command. "
                             << "Steering: "   << command_struct_ackermann_.steering << ", "
                             << "Lin: "   << command_struct_ackermann_.lin << ", "
                             << "Stamp: " << command_struct_ackermann_.stamp);
    }
    else
    {
      ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
  }

  bool AckermannController::getWheelNames(ros::NodeHandle& controller_nh,
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

  void AckermannController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
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

} // namespace ackermann_controller
