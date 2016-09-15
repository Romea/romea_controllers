#ifndef URDF_VEHICLE_KINEMATIC_H
#define URDF_VEHICLE_KINEMATIC_H

#include <ros/ros.h>

#include <urdf_parser/urdf_parser.h>

namespace four_wheel_steering_controller {

class UrdfVehicleKinematic {

public:
  UrdfVehicleKinematic(ros::NodeHandle& root_nh);

  /**
   * \brief Get track from the URDF
   * \param left_wheel_name Name of the left wheel joint
   * \param right_wheel_name Name of the right wheel joint
   * \param track Distance in meter between left and right
   */
  bool getTrack(const std::string& left_wheel_name,
                const std::string& right_wheel_name,
                double& track);

  /**
   * \brief Sets odometry parameters from the URDF, i.e. the wheel radius and separation
   * \param front_wheel_name Name of the left wheel joint
   * \param rear_wheel_name Name of the right wheel joint
   */
  bool setOdomParamsFromUrdf(const std::string& front_wheel_name,
                             const std::string& rear_wheel_name,
                             bool lookup_track,
                             bool lookup_wheel_radius);
private:
  double track_;
  double wheel_radius_;
  double wheel_base_;

  boost::shared_ptr<urdf::ModelInterface> model_;
};

}

#endif
