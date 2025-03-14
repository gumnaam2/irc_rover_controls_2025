// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_interfaces:msg/BaseMotion.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__BASE_MOTION__STRUCT_HPP_
#define MSG_INTERFACES__MSG__DETAIL__BASE_MOTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msg_interfaces__msg__BaseMotion __attribute__((deprecated))
#else
# define DEPRECATED__msg_interfaces__msg__BaseMotion __declspec(deprecated)
#endif

namespace msg_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BaseMotion_
{
  using Type = BaseMotion_<ContainerAllocator>;

  explicit BaseMotion_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->direction = 0ll;
      this->speed = 0ll;
    }
  }

  explicit BaseMotion_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->direction = 0ll;
      this->speed = 0ll;
    }
  }

  // field types and members
  using _direction_type =
    int64_t;
  _direction_type direction;
  using _speed_type =
    int64_t;
  _speed_type speed;

  // setters for named parameter idiom
  Type & set__direction(
    const int64_t & _arg)
  {
    this->direction = _arg;
    return *this;
  }
  Type & set__speed(
    const int64_t & _arg)
  {
    this->speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_interfaces::msg::BaseMotion_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_interfaces::msg::BaseMotion_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_interfaces::msg::BaseMotion_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_interfaces::msg::BaseMotion_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::BaseMotion_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::BaseMotion_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::BaseMotion_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::BaseMotion_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_interfaces::msg::BaseMotion_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_interfaces::msg::BaseMotion_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_interfaces__msg__BaseMotion
    std::shared_ptr<msg_interfaces::msg::BaseMotion_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_interfaces__msg__BaseMotion
    std::shared_ptr<msg_interfaces::msg::BaseMotion_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BaseMotion_ & other) const
  {
    if (this->direction != other.direction) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const BaseMotion_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BaseMotion_

// alias to use template instance with default allocator
using BaseMotion =
  msg_interfaces::msg::BaseMotion_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__BASE_MOTION__STRUCT_HPP_
