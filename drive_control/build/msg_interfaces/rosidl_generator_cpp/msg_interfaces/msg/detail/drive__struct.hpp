// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_interfaces:msg/Drive.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__DRIVE__STRUCT_HPP_
#define MSG_INTERFACES__MSG__DETAIL__DRIVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msg_interfaces__msg__Drive __attribute__((deprecated))
#else
# define DEPRECATED__msg_interfaces__msg__Drive __declspec(deprecated)
#endif

namespace msg_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Drive_
{
  using Type = Drive_<ContainerAllocator>;

  explicit Drive_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sys_check = false;
    }
  }

  explicit Drive_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sys_check = false;
    }
  }

  // field types and members
  using _direction_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _direction_type direction;
  using _speed_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _speed_type speed;
  using _sys_check_type =
    bool;
  _sys_check_type sys_check;

  // setters for named parameter idiom
  Type & set__direction(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->direction = _arg;
    return *this;
  }
  Type & set__speed(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__sys_check(
    const bool & _arg)
  {
    this->sys_check = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_interfaces::msg::Drive_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_interfaces::msg::Drive_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_interfaces::msg::Drive_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_interfaces::msg::Drive_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::Drive_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::Drive_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::Drive_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::Drive_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_interfaces::msg::Drive_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_interfaces::msg::Drive_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_interfaces__msg__Drive
    std::shared_ptr<msg_interfaces::msg::Drive_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_interfaces__msg__Drive
    std::shared_ptr<msg_interfaces::msg::Drive_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Drive_ & other) const
  {
    if (this->direction != other.direction) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->sys_check != other.sys_check) {
      return false;
    }
    return true;
  }
  bool operator!=(const Drive_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Drive_

// alias to use template instance with default allocator
using Drive =
  msg_interfaces::msg::Drive_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__DRIVE__STRUCT_HPP_
