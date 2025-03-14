// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_interfaces:msg/BaseMotion.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__BASE_MOTION__BUILDER_HPP_
#define MSG_INTERFACES__MSG__DETAIL__BASE_MOTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_interfaces/msg/detail/base_motion__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_interfaces
{

namespace msg
{

namespace builder
{

class Init_BaseMotion_speed
{
public:
  explicit Init_BaseMotion_speed(::msg_interfaces::msg::BaseMotion & msg)
  : msg_(msg)
  {}
  ::msg_interfaces::msg::BaseMotion speed(::msg_interfaces::msg::BaseMotion::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_interfaces::msg::BaseMotion msg_;
};

class Init_BaseMotion_direction
{
public:
  Init_BaseMotion_direction()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BaseMotion_speed direction(::msg_interfaces::msg::BaseMotion::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return Init_BaseMotion_speed(msg_);
  }

private:
  ::msg_interfaces::msg::BaseMotion msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_interfaces::msg::BaseMotion>()
{
  return msg_interfaces::msg::builder::Init_BaseMotion_direction();
}

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__BASE_MOTION__BUILDER_HPP_
