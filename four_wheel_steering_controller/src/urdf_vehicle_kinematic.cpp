
#include <four_wheel_steering_controller/urdf_vehicle_kinematic.h>

static double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x-vec2.x,2) +
                   std::pow(vec1.y-vec2.y,2) +
                   std::pow(vec1.z-vec2.z,2));
}

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const boost::shared_ptr<const urdf::Link>& link)
{
  if (!link)
  {
    ROS_ERROR("Link == NULL.");
    return false;
  }

  if (!link->collision)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
    return false;
  }

  if (!link->collision->geometry)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have cylinder geometry");
    return false;
  }

  return true;
}

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
static bool getWheelRadius(const boost::shared_ptr<const urdf::Link>& wheel_link, double& wheel_radius)
{
  if (!isCylinder(wheel_link))
  {
    ROS_ERROR_STREAM("Wheel link " << wheel_link->name << " is NOT modeled as a cylinder!");
    return false;
  }

  wheel_radius = (static_cast<urdf::Cylinder*>(wheel_link->collision->geometry.get()))->radius;
  return true;
}

namespace four_wheel_steering_controller{
  UrdfVehicleKinematic::UrdfVehicleKinematic(ros::NodeHandle& root_nh)
  {
    // Parse robot description
    const std::string model_param_name = "robot_description";
    bool res = root_nh.hasParam(model_param_name);
    std::string robot_model_str="";
    if (!res || !root_nh.getParam(model_param_name,robot_model_str))
    {
      ROS_ERROR("Robot descripion couldn't be retrieved from param server.");
    }
    else
    {
      model_ = urdf::parseURDF(robot_model_str);
      if(!model_)
        ROS_ERROR_STREAM("Could not parse the urdf robot model "<<model_param_name);
    }
  }

  bool UrdfVehicleKinematic::getTransformVector(const std::string& joint_name, const std::string& parent_link_name
                                                , urdf::Vector3 &transform_vector)
  {
    if(model_)
    {
      boost::shared_ptr<const urdf::Joint> joint(model_->getJoint(joint_name));
      if (!joint)
      {
        ROS_ERROR_STREAM(joint_name
                               << " couldn't be retrieved from model description");
        return false;
      }

      transform_vector = joint->parent_to_joint_origin_transform.position;
      while(joint->parent_link_name != parent_link_name)
      {
        boost::shared_ptr<const urdf::Link> link_parent(model_->getLink(joint->parent_link_name));
        if (!link_parent || !link_parent->parent_joint)
        {
          ROS_ERROR_STREAM(joint->parent_link_name
                                 << " couldn't be retrieved from model description or his parent joint");
          return false;
        }
        joint = link_parent->parent_joint;
        transform_vector = transform_vector + joint->parent_to_joint_origin_transform.position;
      }
      return true;
    }
    else
      return false;
  }

  bool UrdfVehicleKinematic::getDistanceBetweenJoints(const std::string& first_joint_name,
                                                      const std::string& second_joint_name,
                                                      double& distance)
  {
    urdf::Vector3 first_transform;
    if(!getTransformVector(first_joint_name, "base_link", first_transform))
      return false;

    urdf::Vector3 second_transform;
    if(!getTransformVector(second_joint_name, "base_link", second_transform))
      return false;

    distance = euclideanOfVectors(first_transform,
                               second_transform);
    ROS_INFO_STREAM("first_transform : "<<first_transform.x<<","<<first_transform.y);
    ROS_INFO_STREAM("distance "<<distance);
    return true;
  }

  bool UrdfVehicleKinematic::getJointRadius(const std::string& joint_name,
                                            double& radius)
  {
    if(model_)
    {
      boost::shared_ptr<const urdf::Joint> joint(model_->getJoint(joint_name));
      // Get wheel radius
      if (!getWheelRadius(model_->getLink(joint->child_link_name), radius))
      {
        ROS_ERROR_STREAM("Couldn't retrieve " << joint_name << " wheel radius");
        return false;
      }
      return true;
    }
    else
      return false;
  }

  bool UrdfVehicleKinematic::setOdomParamsFromUrdf(
                             const std::string& front_wheel_name,
                             const std::string& rear_wheel_name,
                             bool lookup_track,
                             bool lookup_wheel_radius)
  {
    if (!(lookup_track || lookup_wheel_radius))
    {
      // Short-circuit in case we don't need to look up anything, so we don't have to parse the URDF
      return true;
    }

    boost::shared_ptr<const urdf::Joint> front_wheel_joint(model_->getJoint(front_wheel_name));
    boost::shared_ptr<const urdf::Joint> rear_wheel_joint(model_->getJoint(rear_wheel_name));

    if (lookup_track)
    {
      // Get wheel separation
      if (!front_wheel_joint)
      {
        ROS_ERROR_STREAM(front_wheel_name
                               << " couldn't be retrieved from model description");
        return false;
      }

      if (!rear_wheel_joint)
      {
        ROS_ERROR_STREAM(rear_wheel_name
                               << " couldn't be retrieved from model description");
        return false;
      }

      ROS_INFO_STREAM("left wheel to origin: " << front_wheel_joint->parent_to_joint_origin_transform.position.x << ","
                      << front_wheel_joint->parent_to_joint_origin_transform.position.y << ", "
                      << front_wheel_joint->parent_to_joint_origin_transform.position.z);
      ROS_INFO_STREAM("right wheel to origin: " << rear_wheel_joint->parent_to_joint_origin_transform.position.x << ","
                      << rear_wheel_joint->parent_to_joint_origin_transform.position.y << ", "
                      << rear_wheel_joint->parent_to_joint_origin_transform.position.z);

      track_ = euclideanOfVectors(front_wheel_joint->parent_to_joint_origin_transform.position,
                                             rear_wheel_joint->parent_to_joint_origin_transform.position);

    }

    if (lookup_wheel_radius)
    {
      // Get wheel radius
      if (!getWheelRadius(model_->getLink(front_wheel_joint->child_link_name), wheel_radius_))
      {
        ROS_ERROR_STREAM("Couldn't retrieve " << front_wheel_name << " wheel radius");
        return false;
      }
    }

    return true;
  }
}
