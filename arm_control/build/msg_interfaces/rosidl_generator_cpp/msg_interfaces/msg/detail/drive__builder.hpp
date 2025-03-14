// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_interfaces:msg/Drive.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__DRIVE__BUILDER_HPP_
#define MSG_INTERFACES__MSG__DETAIL__DRIVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_interfaces/msg/detail/drive__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_interfaces
{

namespace msg
{

namespace builder
{

class Init_Drive_reset
{
public:
  explicit Init_Drive_reset(::msg_interfaces::msg::Drive & msg)
  : msg_(msg)
  {}
  ::msg_interfaces::msg::Drive reset(::msg_interfaces::msg::Drive::_reset_type arg)
  {
    msg_.reset = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_interfaces::msg::Drive msg_;
};

class Init_Drive_sys_check
{
public:
  explicit Init_Drive_sys_check(::msg_interfaces::msg::Drive & msg)
  : msg_(msg)
  {}
  Init_Drive_reset sys_check(::msg_interfaces::msg::Drive::_sys_check_type arg)
  {
    msg_.sys_check = std::move(arg);
    return Init_Drive_reset(msg_);
  }

private:
  ::msg_interfaces::msg::Drive msg_;
};

class Init_Drive_speed
{
public:
  explicit Init_Drive_speed(::msg_interfaces::msg::Drive & msg)
  : msg_(msg)
  {}
  Init_Drive_sys_check speed(::msg_interfaces::msg::Drive::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_Drive_sys_check(msg_);
  }

private:
  ::msg_interfaces::msg::Drive msg_;
};

class Init_Drive_direction
{
public:
  Init_Drive_direction()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Drive_speed direction(::msg_interfaces::msg::Drive::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return Init_Drive_speed(msg_);
  }

private:
  ::msg_interfaces::msg::Drive msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_interfaces::msg::Drive>()
{
  return msg_interfaces::msg::builder::Init_Drive_direction();
}

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__DRIVE__BUILDER_HPP_
