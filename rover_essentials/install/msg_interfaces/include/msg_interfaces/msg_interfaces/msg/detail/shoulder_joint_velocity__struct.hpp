// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_interfaces:msg/ShoulderJointVelocity.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__SHOULDER_JOINT_VELOCITY__STRUCT_HPP_
#define MSG_INTERFACES__MSG__DETAIL__SHOULDER_JOINT_VELOCITY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msg_interfaces__msg__ShoulderJointVelocity __attribute__((deprecated))
#else
# define DEPRECATED__msg_interfaces__msg__ShoulderJointVelocity __declspec(deprecated)
#endif

namespace msg_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ShoulderJointVelocity_
{
  using Type = ShoulderJointVelocity_<ContainerAllocator>;

  explicit ShoulderJointVelocity_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angular_speed = 0.0;
    }
  }

  explicit ShoulderJointVelocity_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angular_speed = 0.0;
    }
  }

  // field types and members
  using _angular_speed_type =
    double;
  _angular_speed_type angular_speed;

  // setters for named parameter idiom
  Type & set__angular_speed(
    const double & _arg)
  {
    this->angular_speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_interfaces::msg::ShoulderJointVelocity_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_interfaces::msg::ShoulderJointVelocity_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_interfaces::msg::ShoulderJointVelocity_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_interfaces::msg::ShoulderJointVelocity_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::ShoulderJointVelocity_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::ShoulderJointVelocity_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::ShoulderJointVelocity_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::ShoulderJointVelocity_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_interfaces::msg::ShoulderJointVelocity_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_interfaces::msg::ShoulderJointVelocity_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_interfaces__msg__ShoulderJointVelocity
    std::shared_ptr<msg_interfaces::msg::ShoulderJointVelocity_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_interfaces__msg__ShoulderJointVelocity
    std::shared_ptr<msg_interfaces::msg::ShoulderJointVelocity_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ShoulderJointVelocity_ & other) const
  {
    if (this->angular_speed != other.angular_speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const ShoulderJointVelocity_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ShoulderJointVelocity_

// alias to use template instance with default allocator
using ShoulderJointVelocity =
  msg_interfaces::msg::ShoulderJointVelocity_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__SHOULDER_JOINT_VELOCITY__STRUCT_HPP_
