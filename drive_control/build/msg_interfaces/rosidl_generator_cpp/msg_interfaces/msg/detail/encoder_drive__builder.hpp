// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_interfaces:msg/EncoderDrive.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ENCODER_DRIVE__BUILDER_HPP_
#define MSG_INTERFACES__MSG__DETAIL__ENCODER_DRIVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_interfaces/msg/detail/encoder_drive__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_interfaces
{

namespace msg
{

namespace builder
{

class Init_EncoderDrive_drive_node5
{
public:
  explicit Init_EncoderDrive_drive_node5(::msg_interfaces::msg::EncoderDrive & msg)
  : msg_(msg)
  {}
  ::msg_interfaces::msg::EncoderDrive drive_node5(::msg_interfaces::msg::EncoderDrive::_drive_node5_type arg)
  {
    msg_.drive_node5 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_interfaces::msg::EncoderDrive msg_;
};

class Init_EncoderDrive_drive_node4
{
public:
  explicit Init_EncoderDrive_drive_node4(::msg_interfaces::msg::EncoderDrive & msg)
  : msg_(msg)
  {}
  Init_EncoderDrive_drive_node5 drive_node4(::msg_interfaces::msg::EncoderDrive::_drive_node4_type arg)
  {
    msg_.drive_node4 = std::move(arg);
    return Init_EncoderDrive_drive_node5(msg_);
  }

private:
  ::msg_interfaces::msg::EncoderDrive msg_;
};

class Init_EncoderDrive_drive_node3
{
public:
  explicit Init_EncoderDrive_drive_node3(::msg_interfaces::msg::EncoderDrive & msg)
  : msg_(msg)
  {}
  Init_EncoderDrive_drive_node4 drive_node3(::msg_interfaces::msg::EncoderDrive::_drive_node3_type arg)
  {
    msg_.drive_node3 = std::move(arg);
    return Init_EncoderDrive_drive_node4(msg_);
  }

private:
  ::msg_interfaces::msg::EncoderDrive msg_;
};

class Init_EncoderDrive_drive_node2
{
public:
  explicit Init_EncoderDrive_drive_node2(::msg_interfaces::msg::EncoderDrive & msg)
  : msg_(msg)
  {}
  Init_EncoderDrive_drive_node3 drive_node2(::msg_interfaces::msg::EncoderDrive::_drive_node2_type arg)
  {
    msg_.drive_node2 = std::move(arg);
    return Init_EncoderDrive_drive_node3(msg_);
  }

private:
  ::msg_interfaces::msg::EncoderDrive msg_;
};

class Init_EncoderDrive_drive_node1
{
public:
  explicit Init_EncoderDrive_drive_node1(::msg_interfaces::msg::EncoderDrive & msg)
  : msg_(msg)
  {}
  Init_EncoderDrive_drive_node2 drive_node1(::msg_interfaces::msg::EncoderDrive::_drive_node1_type arg)
  {
    msg_.drive_node1 = std::move(arg);
    return Init_EncoderDrive_drive_node2(msg_);
  }

private:
  ::msg_interfaces::msg::EncoderDrive msg_;
};

class Init_EncoderDrive_drive_node0
{
public:
  Init_EncoderDrive_drive_node0()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EncoderDrive_drive_node1 drive_node0(::msg_interfaces::msg::EncoderDrive::_drive_node0_type arg)
  {
    msg_.drive_node0 = std::move(arg);
    return Init_EncoderDrive_drive_node1(msg_);
  }

private:
  ::msg_interfaces::msg::EncoderDrive msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_interfaces::msg::EncoderDrive>()
{
  return msg_interfaces::msg::builder::Init_EncoderDrive_drive_node0();
}

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__ENCODER_DRIVE__BUILDER_HPP_
