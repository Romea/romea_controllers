
#include <controller_interface/controller_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include <four_wheel_steering_msgs/FourWheelSteeringDrive.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <four_wheel_steering_controller/odometry.h>
#include <four_wheel_steering_controller/speed_limiter.h>

namespace four_wheel_steering_controller{

  /**
   * This class makes some assumptions on the model of the robot:
   *  - the rotation axes of wheels are collinear
   *  - the wheels are identical in radius
   * Additional assumptions to not duplicate information readily available in the URDF:
   *  - the wheels have the same parent frame
   *  - a wheel collision geometry is a cylinder in the urdf
   *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
   */
  class FourWheelSteeringController
      : public controller_interface::ControllerBase
  {
  public:
    FourWheelSteeringController();

    /**
     * \brief Initialize controller
     * \param hw            Velocity joint interface for the wheels
     * \param root_nh       Node handle at root namespace
     * \param controller_nh Node handle inside the controller namespace
     */
    virtual bool initRequest(hardware_interface::RobotHW *const robot_hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh,
              std::set<std::string>& claimed_resources);
    /// Get the name of this controller's hardware interface type
    std::string getHardwareInterfaceType() const
    {
      return "";
    }

    bool init(hardware_interface::PositionJointInterface* hw_pos,
                                     hardware_interface::VelocityJointInterface* hw_vel,
                                     ros::NodeHandle& root_nh,
                                     ros::NodeHandle &controller_nh);

    /**
     * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
     * \param time   Current time
     * \param period Time since the last called to update
     */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
     * \brief Starts controller
     * \param time Current time
     */
    void starting(const ros::Time& time);

    /**
     * \brief Stops controller
     * \param time Current time
     */
    void stopping(const ros::Time& /*time*/);

  private:
    std::string name_;

    /// Odometry related:
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;
    bool open_loop_;

    /// Hardware handles:
    std::vector<hardware_interface::JointHandle> front_wheel_joints_;
    std::vector<hardware_interface::JointHandle> rear_wheel_joints_;
    std::vector<hardware_interface::JointHandle> front_steering_joints_;
    std::vector<hardware_interface::JointHandle> rear_steering_joints_;

    /// Velocity command related:
    struct Commands
    {
      double lin;
      double ang;
      double front_steering;
      double rear_steering;
      ros::Time stamp;

      Commands() : lin(0.0), ang(0.0), front_steering(0.0), rear_steering(0.0), stamp(0.0) {}
    };
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
    ros::Subscriber sub_command_;

    /// FourWheelSteering command related:
    realtime_tools::RealtimeBuffer<Commands> command_four_wheel_steering_;
    Commands command_struct_four_wheel_steering_;
    ros::Subscriber sub_command_four_wheel_steering_;

    /// Odometry related:
    boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
    Odometry odometry_;

    /// Wheel separation (or track), distance between left and right wheels (from the midpoint of the wheel width):
    double track_;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheel_radius_;

    /// Wheel base (distance between front and rear wheel):
    double wheel_base_;

    /// Timeout to consider cmd_vel commands old:
    double cmd_vel_timeout_;

    /// Frame to use for the robot base:
    std::string base_frame_id_;

    /// Whether to publish odometry to tf or not:
    bool enable_odom_tf_;

    /// Whether the control is make with four_wheel_steering msg or twist msg:
    bool enable_twist_cmd_;

    /// Speed limiters:
    Commands last1_cmd_;
    Commands last0_cmd_;
    SpeedLimiter limiter_lin_;
    SpeedLimiter limiter_ang_;

  private:
    /**
     * \brief Brakes the wheels, i.e. sets the velocity to 0
     */
    void brake();

    /**
     * \brief Velocity command callback
     * \param command Velocity command message (twist)
     */
    void cmdVelCallback(const geometry_msgs::Twist& command);

    /**
     * \brief Velocity command callback
     * \param command Velocity command message (twist)
     */
    void cmdFourWheelSteeringCallback(const four_wheel_steering_msgs::FourWheelSteeringDrive& command);

    /**
     * \brief Get the wheel names from a wheel param
     * \param [in]  controller_nh Controller node handler
     * \param [in]  wheel_param   Param name
     * \param [out] wheel_names   Vector with the whel names
     * \return true if the wheel_param is available and the wheel_names are
     *        retrieved successfully from the param server; false otherwise
     */
    bool getWheelNames(ros::NodeHandle& controller_nh,
                       const std::string& wheel_param,
                       std::vector<std::string>& wheel_names);

    /**
     * \brief Sets the odometry publishing fields
     * \param root_nh Root node handle
     * \param controller_nh Node handle inside the controller namespace
     */
    void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  };

  PLUGINLIB_EXPORT_CLASS(four_wheel_steering_controller::FourWheelSteeringController, controller_interface::ControllerBase);
} // namespace four_wheel_steering_controller
