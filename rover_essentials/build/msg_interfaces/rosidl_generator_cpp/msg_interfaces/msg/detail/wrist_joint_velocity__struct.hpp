// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_interfaces:msg/WristJointVelocity.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__WRIST_JOINT_VELOCITY__STRUCT_HPP_
#define MSG_INTERFACES__MSG__DETAIL__WRIST_JOINT_VELOCITY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msg_interfaces__msg__WristJointVelocity __attribute__((deprecated))
#else
# define DEPRECATED__msg_interfaces__msg__WristJointVelocity __declspec(deprecated)
#endif

namespace msg_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WristJointVelocity_
{
  using Type = WristJointVelocity_<ContainerAllocator>;

  explicit WristJointVelocity_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit WristJointVelocity_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _angular_speed_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _angular_speed_type angular_speed;

  // setters for named parameter idiom
  Type & set__angular_speed(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->angular_speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_interfaces::msg::WristJointVelocity_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_interfaces::msg::WristJointVelocity_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_interfaces::msg::WristJointVelocity_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_interfaces::msg::WristJointVelocity_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::WristJointVelocity_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::WristJointVelocity_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::WristJointVelocity_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::WristJointVelocity_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_interfaces::msg::WristJointVelocity_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_interfaces::msg::WristJointVelocity_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_interfaces__msg__WristJointVelocity
    std::shared_ptr<msg_interfaces::msg::WristJointVelocity_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_interfaces__msg__WristJointVelocity
    std::shared_ptr<msg_interfaces::msg::WristJointVelocity_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WristJointVelocity_ & other) const
  {
    if (this->angular_speed != other.angular_speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const WristJointVelocity_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WristJointVelocity_

// alias to use template instance with default allocator
using WristJointVelocity =
  msg_interfaces::msg::WristJointVelocity_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__WRIST_JOINT_VELOCITY__STRUCT_HPP_
