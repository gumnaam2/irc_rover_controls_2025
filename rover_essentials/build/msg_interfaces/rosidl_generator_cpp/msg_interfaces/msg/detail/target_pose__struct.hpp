// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_interfaces:msg/TargetPose.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__TARGET_POSE__STRUCT_HPP_
#define MSG_INTERFACES__MSG__DETAIL__TARGET_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msg_interfaces__msg__TargetPose __attribute__((deprecated))
#else
# define DEPRECATED__msg_interfaces__msg__TargetPose __declspec(deprecated)
#endif

namespace msg_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TargetPose_
{
  using Type = TargetPose_<ContainerAllocator>;

  explicit TargetPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->base = 0.0;
      this->shoulder = 0.0;
      this->elbow = 0.0;
      this->wrist_up_down = 0.0;
      this->wrist_rot = 0.0;
    }
  }

  explicit TargetPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->base = 0.0;
      this->shoulder = 0.0;
      this->elbow = 0.0;
      this->wrist_up_down = 0.0;
      this->wrist_rot = 0.0;
    }
  }

  // field types and members
  using _base_type =
    double;
  _base_type base;
  using _shoulder_type =
    double;
  _shoulder_type shoulder;
  using _elbow_type =
    double;
  _elbow_type elbow;
  using _wrist_up_down_type =
    double;
  _wrist_up_down_type wrist_up_down;
  using _wrist_rot_type =
    double;
  _wrist_rot_type wrist_rot;

  // setters for named parameter idiom
  Type & set__base(
    const double & _arg)
  {
    this->base = _arg;
    return *this;
  }
  Type & set__shoulder(
    const double & _arg)
  {
    this->shoulder = _arg;
    return *this;
  }
  Type & set__elbow(
    const double & _arg)
  {
    this->elbow = _arg;
    return *this;
  }
  Type & set__wrist_up_down(
    const double & _arg)
  {
    this->wrist_up_down = _arg;
    return *this;
  }
  Type & set__wrist_rot(
    const double & _arg)
  {
    this->wrist_rot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_interfaces::msg::TargetPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_interfaces::msg::TargetPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_interfaces::msg::TargetPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_interfaces::msg::TargetPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::TargetPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::TargetPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::TargetPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::TargetPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_interfaces::msg::TargetPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_interfaces::msg::TargetPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_interfaces__msg__TargetPose
    std::shared_ptr<msg_interfaces::msg::TargetPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_interfaces__msg__TargetPose
    std::shared_ptr<msg_interfaces::msg::TargetPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TargetPose_ & other) const
  {
    if (this->base != other.base) {
      return false;
    }
    if (this->shoulder != other.shoulder) {
      return false;
    }
    if (this->elbow != other.elbow) {
      return false;
    }
    if (this->wrist_up_down != other.wrist_up_down) {
      return false;
    }
    if (this->wrist_rot != other.wrist_rot) {
      return false;
    }
    return true;
  }
  bool operator!=(const TargetPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TargetPose_

// alias to use template instance with default allocator
using TargetPose =
  msg_interfaces::msg::TargetPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__TARGET_POSE__STRUCT_HPP_
