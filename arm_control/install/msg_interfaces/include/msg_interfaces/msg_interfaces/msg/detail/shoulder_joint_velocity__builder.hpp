// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_interfaces:msg/ShoulderJointVelocity.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__SHOULDER_JOINT_VELOCITY__BUILDER_HPP_
#define MSG_INTERFACES__MSG__DETAIL__SHOULDER_JOINT_VELOCITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_interfaces/msg/detail/shoulder_joint_velocity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_interfaces
{

namespace msg
{

namespace builder
{

class Init_ShoulderJointVelocity_angular_speed
{
public:
  Init_ShoulderJointVelocity_angular_speed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::msg_interfaces::msg::ShoulderJointVelocity angular_speed(::msg_interfaces::msg::ShoulderJointVelocity::_angular_speed_type arg)
  {
    msg_.angular_speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_interfaces::msg::ShoulderJointVelocity msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_interfaces::msg::ShoulderJointVelocity>()
{
  return msg_interfaces::msg::builder::Init_ShoulderJointVelocity_angular_speed();
}

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__SHOULDER_JOINT_VELOCITY__BUILDER_HPP_
