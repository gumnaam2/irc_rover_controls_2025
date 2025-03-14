// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_interfaces:msg/CamDrive.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__CAM_DRIVE__STRUCT_HPP_
#define MSG_INTERFACES__MSG__DETAIL__CAM_DRIVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msg_interfaces__msg__CamDrive __attribute__((deprecated))
#else
# define DEPRECATED__msg_interfaces__msg__CamDrive __declspec(deprecated)
#endif

namespace msg_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CamDrive_
{
  using Type = CamDrive_<ContainerAllocator>;

  explicit CamDrive_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0;
      this->direction = "";
    }
  }

  explicit CamDrive_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : direction(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0;
      this->direction = "";
    }
  }

  // field types and members
  using _speed_type =
    double;
  _speed_type speed;
  using _direction_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _direction_type direction;

  // setters for named parameter idiom
  Type & set__speed(
    const double & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__direction(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->direction = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_interfaces::msg::CamDrive_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_interfaces::msg::CamDrive_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_interfaces::msg::CamDrive_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_interfaces::msg::CamDrive_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::CamDrive_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::CamDrive_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::CamDrive_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::CamDrive_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_interfaces::msg::CamDrive_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_interfaces::msg::CamDrive_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_interfaces__msg__CamDrive
    std::shared_ptr<msg_interfaces::msg::CamDrive_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_interfaces__msg__CamDrive
    std::shared_ptr<msg_interfaces::msg::CamDrive_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CamDrive_ & other) const
  {
    if (this->speed != other.speed) {
      return false;
    }
    if (this->direction != other.direction) {
      return false;
    }
    return true;
  }
  bool operator!=(const CamDrive_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CamDrive_

// alias to use template instance with default allocator
using CamDrive =
  msg_interfaces::msg::CamDrive_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__CAM_DRIVE__STRUCT_HPP_
