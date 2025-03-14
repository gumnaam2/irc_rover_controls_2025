// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_interfaces:msg/ArmAngle.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ARM_ANGLE__STRUCT_HPP_
#define MSG_INTERFACES__MSG__DETAIL__ARM_ANGLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msg_interfaces__msg__ArmAngle __attribute__((deprecated))
#else
# define DEPRECATED__msg_interfaces__msg__ArmAngle __declspec(deprecated)
#endif

namespace msg_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ArmAngle_
{
  using Type = ArmAngle_<ContainerAllocator>;

  explicit ArmAngle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->alpha = 0.0;
      this->beta = 0.0;
    }
  }

  explicit ArmAngle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->alpha = 0.0;
      this->beta = 0.0;
    }
  }

  // field types and members
  using _alpha_type =
    double;
  _alpha_type alpha;
  using _beta_type =
    double;
  _beta_type beta;

  // setters for named parameter idiom
  Type & set__alpha(
    const double & _arg)
  {
    this->alpha = _arg;
    return *this;
  }
  Type & set__beta(
    const double & _arg)
  {
    this->beta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_interfaces::msg::ArmAngle_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_interfaces::msg::ArmAngle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_interfaces::msg::ArmAngle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_interfaces::msg::ArmAngle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::ArmAngle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::ArmAngle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::ArmAngle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::ArmAngle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_interfaces::msg::ArmAngle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_interfaces::msg::ArmAngle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_interfaces__msg__ArmAngle
    std::shared_ptr<msg_interfaces::msg::ArmAngle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_interfaces__msg__ArmAngle
    std::shared_ptr<msg_interfaces::msg::ArmAngle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArmAngle_ & other) const
  {
    if (this->alpha != other.alpha) {
      return false;
    }
    if (this->beta != other.beta) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArmAngle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArmAngle_

// alias to use template instance with default allocator
using ArmAngle =
  msg_interfaces::msg::ArmAngle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__ARM_ANGLE__STRUCT_HPP_
