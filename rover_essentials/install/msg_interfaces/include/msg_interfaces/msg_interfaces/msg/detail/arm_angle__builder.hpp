// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_interfaces:msg/ArmAngle.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ARM_ANGLE__BUILDER_HPP_
#define MSG_INTERFACES__MSG__DETAIL__ARM_ANGLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_interfaces/msg/detail/arm_angle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_interfaces
{

namespace msg
{

namespace builder
{

class Init_ArmAngle_beta
{
public:
  explicit Init_ArmAngle_beta(::msg_interfaces::msg::ArmAngle & msg)
  : msg_(msg)
  {}
  ::msg_interfaces::msg::ArmAngle beta(::msg_interfaces::msg::ArmAngle::_beta_type arg)
  {
    msg_.beta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_interfaces::msg::ArmAngle msg_;
};

class Init_ArmAngle_alpha
{
public:
  Init_ArmAngle_alpha()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArmAngle_beta alpha(::msg_interfaces::msg::ArmAngle::_alpha_type arg)
  {
    msg_.alpha = std::move(arg);
    return Init_ArmAngle_beta(msg_);
  }

private:
  ::msg_interfaces::msg::ArmAngle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_interfaces::msg::ArmAngle>()
{
  return msg_interfaces::msg::builder::Init_ArmAngle_alpha();
}

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__ARM_ANGLE__BUILDER_HPP_
