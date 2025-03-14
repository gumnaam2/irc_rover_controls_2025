// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_interfaces:msg/MotorSix.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__MOTOR_SIX__BUILDER_HPP_
#define MSG_INTERFACES__MSG__DETAIL__MOTOR_SIX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_interfaces/msg/detail/motor_six__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotorSix_motor6_direction
{
public:
  explicit Init_MotorSix_motor6_direction(::msg_interfaces::msg::MotorSix & msg)
  : msg_(msg)
  {}
  ::msg_interfaces::msg::MotorSix motor6_direction(::msg_interfaces::msg::MotorSix::_motor6_direction_type arg)
  {
    msg_.motor6_direction = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_interfaces::msg::MotorSix msg_;
};

class Init_MotorSix_motor6_speed
{
public:
  explicit Init_MotorSix_motor6_speed(::msg_interfaces::msg::MotorSix & msg)
  : msg_(msg)
  {}
  Init_MotorSix_motor6_direction motor6_speed(::msg_interfaces::msg::MotorSix::_motor6_speed_type arg)
  {
    msg_.motor6_speed = std::move(arg);
    return Init_MotorSix_motor6_direction(msg_);
  }

private:
  ::msg_interfaces::msg::MotorSix msg_;
};

class Init_MotorSix_motor5_direction
{
public:
  explicit Init_MotorSix_motor5_direction(::msg_interfaces::msg::MotorSix & msg)
  : msg_(msg)
  {}
  Init_MotorSix_motor6_speed motor5_direction(::msg_interfaces::msg::MotorSix::_motor5_direction_type arg)
  {
    msg_.motor5_direction = std::move(arg);
    return Init_MotorSix_motor6_speed(msg_);
  }

private:
  ::msg_interfaces::msg::MotorSix msg_;
};

class Init_MotorSix_motor5_speed
{
public:
  explicit Init_MotorSix_motor5_speed(::msg_interfaces::msg::MotorSix & msg)
  : msg_(msg)
  {}
  Init_MotorSix_motor5_direction motor5_speed(::msg_interfaces::msg::MotorSix::_motor5_speed_type arg)
  {
    msg_.motor5_speed = std::move(arg);
    return Init_MotorSix_motor5_direction(msg_);
  }

private:
  ::msg_interfaces::msg::MotorSix msg_;
};

class Init_MotorSix_motor4_direction
{
public:
  explicit Init_MotorSix_motor4_direction(::msg_interfaces::msg::MotorSix & msg)
  : msg_(msg)
  {}
  Init_MotorSix_motor5_speed motor4_direction(::msg_interfaces::msg::MotorSix::_motor4_direction_type arg)
  {
    msg_.motor4_direction = std::move(arg);
    return Init_MotorSix_motor5_speed(msg_);
  }

private:
  ::msg_interfaces::msg::MotorSix msg_;
};

class Init_MotorSix_motor4_speed
{
public:
  explicit Init_MotorSix_motor4_speed(::msg_interfaces::msg::MotorSix & msg)
  : msg_(msg)
  {}
  Init_MotorSix_motor4_direction motor4_speed(::msg_interfaces::msg::MotorSix::_motor4_speed_type arg)
  {
    msg_.motor4_speed = std::move(arg);
    return Init_MotorSix_motor4_direction(msg_);
  }

private:
  ::msg_interfaces::msg::MotorSix msg_;
};

class Init_MotorSix_motor3_direction
{
public:
  explicit Init_MotorSix_motor3_direction(::msg_interfaces::msg::MotorSix & msg)
  : msg_(msg)
  {}
  Init_MotorSix_motor4_speed motor3_direction(::msg_interfaces::msg::MotorSix::_motor3_direction_type arg)
  {
    msg_.motor3_direction = std::move(arg);
    return Init_MotorSix_motor4_speed(msg_);
  }

private:
  ::msg_interfaces::msg::MotorSix msg_;
};

class Init_MotorSix_motor3_speed
{
public:
  explicit Init_MotorSix_motor3_speed(::msg_interfaces::msg::MotorSix & msg)
  : msg_(msg)
  {}
  Init_MotorSix_motor3_direction motor3_speed(::msg_interfaces::msg::MotorSix::_motor3_speed_type arg)
  {
    msg_.motor3_speed = std::move(arg);
    return Init_MotorSix_motor3_direction(msg_);
  }

private:
  ::msg_interfaces::msg::MotorSix msg_;
};

class Init_MotorSix_motor2_direction
{
public:
  explicit Init_MotorSix_motor2_direction(::msg_interfaces::msg::MotorSix & msg)
  : msg_(msg)
  {}
  Init_MotorSix_motor3_speed motor2_direction(::msg_interfaces::msg::MotorSix::_motor2_direction_type arg)
  {
    msg_.motor2_direction = std::move(arg);
    return Init_MotorSix_motor3_speed(msg_);
  }

private:
  ::msg_interfaces::msg::MotorSix msg_;
};

class Init_MotorSix_motor2_speed
{
public:
  explicit Init_MotorSix_motor2_speed(::msg_interfaces::msg::MotorSix & msg)
  : msg_(msg)
  {}
  Init_MotorSix_motor2_direction motor2_speed(::msg_interfaces::msg::MotorSix::_motor2_speed_type arg)
  {
    msg_.motor2_speed = std::move(arg);
    return Init_MotorSix_motor2_direction(msg_);
  }

private:
  ::msg_interfaces::msg::MotorSix msg_;
};

class Init_MotorSix_motor1_direction
{
public:
  explicit Init_MotorSix_motor1_direction(::msg_interfaces::msg::MotorSix & msg)
  : msg_(msg)
  {}
  Init_MotorSix_motor2_speed motor1_direction(::msg_interfaces::msg::MotorSix::_motor1_direction_type arg)
  {
    msg_.motor1_direction = std::move(arg);
    return Init_MotorSix_motor2_speed(msg_);
  }

private:
  ::msg_interfaces::msg::MotorSix msg_;
};

class Init_MotorSix_motor1_speed
{
public:
  Init_MotorSix_motor1_speed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorSix_motor1_direction motor1_speed(::msg_interfaces::msg::MotorSix::_motor1_speed_type arg)
  {
    msg_.motor1_speed = std::move(arg);
    return Init_MotorSix_motor1_direction(msg_);
  }

private:
  ::msg_interfaces::msg::MotorSix msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_interfaces::msg::MotorSix>()
{
  return msg_interfaces::msg::builder::Init_MotorSix_motor1_speed();
}

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__MOTOR_SIX__BUILDER_HPP_
