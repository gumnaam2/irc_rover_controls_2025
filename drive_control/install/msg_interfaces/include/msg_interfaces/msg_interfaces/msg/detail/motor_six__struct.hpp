// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_interfaces:msg/MotorSix.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__MOTOR_SIX__STRUCT_HPP_
#define MSG_INTERFACES__MSG__DETAIL__MOTOR_SIX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msg_interfaces__msg__MotorSix __attribute__((deprecated))
#else
# define DEPRECATED__msg_interfaces__msg__MotorSix __declspec(deprecated)
#endif

namespace msg_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorSix_
{
  using Type = MotorSix_<ContainerAllocator>;

  explicit MotorSix_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor1_speed = 0.0;
      this->motor1_direction = "";
      this->motor2_speed = 0.0;
      this->motor2_direction = "";
      this->motor3_speed = 0.0;
      this->motor3_direction = "";
      this->motor4_speed = 0.0;
      this->motor4_direction = "";
      this->motor5_speed = 0.0;
      this->motor5_direction = "";
      this->motor6_speed = 0.0;
      this->motor6_direction = "";
    }
  }

  explicit MotorSix_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : motor1_direction(_alloc),
    motor2_direction(_alloc),
    motor3_direction(_alloc),
    motor4_direction(_alloc),
    motor5_direction(_alloc),
    motor6_direction(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor1_speed = 0.0;
      this->motor1_direction = "";
      this->motor2_speed = 0.0;
      this->motor2_direction = "";
      this->motor3_speed = 0.0;
      this->motor3_direction = "";
      this->motor4_speed = 0.0;
      this->motor4_direction = "";
      this->motor5_speed = 0.0;
      this->motor5_direction = "";
      this->motor6_speed = 0.0;
      this->motor6_direction = "";
    }
  }

  // field types and members
  using _motor1_speed_type =
    double;
  _motor1_speed_type motor1_speed;
  using _motor1_direction_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _motor1_direction_type motor1_direction;
  using _motor2_speed_type =
    double;
  _motor2_speed_type motor2_speed;
  using _motor2_direction_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _motor2_direction_type motor2_direction;
  using _motor3_speed_type =
    double;
  _motor3_speed_type motor3_speed;
  using _motor3_direction_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _motor3_direction_type motor3_direction;
  using _motor4_speed_type =
    double;
  _motor4_speed_type motor4_speed;
  using _motor4_direction_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _motor4_direction_type motor4_direction;
  using _motor5_speed_type =
    double;
  _motor5_speed_type motor5_speed;
  using _motor5_direction_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _motor5_direction_type motor5_direction;
  using _motor6_speed_type =
    double;
  _motor6_speed_type motor6_speed;
  using _motor6_direction_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _motor6_direction_type motor6_direction;

  // setters for named parameter idiom
  Type & set__motor1_speed(
    const double & _arg)
  {
    this->motor1_speed = _arg;
    return *this;
  }
  Type & set__motor1_direction(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->motor1_direction = _arg;
    return *this;
  }
  Type & set__motor2_speed(
    const double & _arg)
  {
    this->motor2_speed = _arg;
    return *this;
  }
  Type & set__motor2_direction(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->motor2_direction = _arg;
    return *this;
  }
  Type & set__motor3_speed(
    const double & _arg)
  {
    this->motor3_speed = _arg;
    return *this;
  }
  Type & set__motor3_direction(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->motor3_direction = _arg;
    return *this;
  }
  Type & set__motor4_speed(
    const double & _arg)
  {
    this->motor4_speed = _arg;
    return *this;
  }
  Type & set__motor4_direction(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->motor4_direction = _arg;
    return *this;
  }
  Type & set__motor5_speed(
    const double & _arg)
  {
    this->motor5_speed = _arg;
    return *this;
  }
  Type & set__motor5_direction(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->motor5_direction = _arg;
    return *this;
  }
  Type & set__motor6_speed(
    const double & _arg)
  {
    this->motor6_speed = _arg;
    return *this;
  }
  Type & set__motor6_direction(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->motor6_direction = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_interfaces::msg::MotorSix_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_interfaces::msg::MotorSix_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_interfaces::msg::MotorSix_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_interfaces::msg::MotorSix_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::MotorSix_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::MotorSix_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::MotorSix_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::MotorSix_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_interfaces::msg::MotorSix_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_interfaces::msg::MotorSix_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_interfaces__msg__MotorSix
    std::shared_ptr<msg_interfaces::msg::MotorSix_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_interfaces__msg__MotorSix
    std::shared_ptr<msg_interfaces::msg::MotorSix_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorSix_ & other) const
  {
    if (this->motor1_speed != other.motor1_speed) {
      return false;
    }
    if (this->motor1_direction != other.motor1_direction) {
      return false;
    }
    if (this->motor2_speed != other.motor2_speed) {
      return false;
    }
    if (this->motor2_direction != other.motor2_direction) {
      return false;
    }
    if (this->motor3_speed != other.motor3_speed) {
      return false;
    }
    if (this->motor3_direction != other.motor3_direction) {
      return false;
    }
    if (this->motor4_speed != other.motor4_speed) {
      return false;
    }
    if (this->motor4_direction != other.motor4_direction) {
      return false;
    }
    if (this->motor5_speed != other.motor5_speed) {
      return false;
    }
    if (this->motor5_direction != other.motor5_direction) {
      return false;
    }
    if (this->motor6_speed != other.motor6_speed) {
      return false;
    }
    if (this->motor6_direction != other.motor6_direction) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorSix_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorSix_

// alias to use template instance with default allocator
using MotorSix =
  msg_interfaces::msg::MotorSix_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__MOTOR_SIX__STRUCT_HPP_
