// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_interfaces:msg/FinalVel.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__FINAL_VEL__BUILDER_HPP_
#define MSG_INTERFACES__MSG__DETAIL__FINAL_VEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_interfaces/msg/detail/final_vel__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_interfaces
{

namespace msg
{

namespace builder
{

class Init_FinalVel_direction
{
public:
  explicit Init_FinalVel_direction(::msg_interfaces::msg::FinalVel & msg)
  : msg_(msg)
  {}
  ::msg_interfaces::msg::FinalVel direction(::msg_interfaces::msg::FinalVel::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_interfaces::msg::FinalVel msg_;
};

class Init_FinalVel_speed
{
public:
  Init_FinalVel_speed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FinalVel_direction speed(::msg_interfaces::msg::FinalVel::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_FinalVel_direction(msg_);
  }

private:
  ::msg_interfaces::msg::FinalVel msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_interfaces::msg::FinalVel>()
{
  return msg_interfaces::msg::builder::Init_FinalVel_speed();
}

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__FINAL_VEL__BUILDER_HPP_
