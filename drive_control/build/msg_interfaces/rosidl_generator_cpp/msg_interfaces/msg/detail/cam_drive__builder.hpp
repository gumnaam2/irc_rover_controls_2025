// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_interfaces:msg/CamDrive.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__CAM_DRIVE__BUILDER_HPP_
#define MSG_INTERFACES__MSG__DETAIL__CAM_DRIVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_interfaces/msg/detail/cam_drive__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_interfaces
{

namespace msg
{

namespace builder
{

class Init_CamDrive_direction
{
public:
  explicit Init_CamDrive_direction(::msg_interfaces::msg::CamDrive & msg)
  : msg_(msg)
  {}
  ::msg_interfaces::msg::CamDrive direction(::msg_interfaces::msg::CamDrive::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_interfaces::msg::CamDrive msg_;
};

class Init_CamDrive_speed
{
public:
  Init_CamDrive_speed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CamDrive_direction speed(::msg_interfaces::msg::CamDrive::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_CamDrive_direction(msg_);
  }

private:
  ::msg_interfaces::msg::CamDrive msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_interfaces::msg::CamDrive>()
{
  return msg_interfaces::msg::builder::Init_CamDrive_speed();
}

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__CAM_DRIVE__BUILDER_HPP_
