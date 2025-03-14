// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_interfaces:msg/EncoderArm.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ENCODER_ARM__BUILDER_HPP_
#define MSG_INTERFACES__MSG__DETAIL__ENCODER_ARM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_interfaces/msg/detail/encoder_arm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_interfaces
{

namespace msg
{

namespace builder
{

class Init_EncoderArm_arm_node5
{
public:
  explicit Init_EncoderArm_arm_node5(::msg_interfaces::msg::EncoderArm & msg)
  : msg_(msg)
  {}
  ::msg_interfaces::msg::EncoderArm arm_node5(::msg_interfaces::msg::EncoderArm::_arm_node5_type arg)
  {
    msg_.arm_node5 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_interfaces::msg::EncoderArm msg_;
};

class Init_EncoderArm_arm_node4
{
public:
  explicit Init_EncoderArm_arm_node4(::msg_interfaces::msg::EncoderArm & msg)
  : msg_(msg)
  {}
  Init_EncoderArm_arm_node5 arm_node4(::msg_interfaces::msg::EncoderArm::_arm_node4_type arg)
  {
    msg_.arm_node4 = std::move(arg);
    return Init_EncoderArm_arm_node5(msg_);
  }

private:
  ::msg_interfaces::msg::EncoderArm msg_;
};

class Init_EncoderArm_arm_node3
{
public:
  explicit Init_EncoderArm_arm_node3(::msg_interfaces::msg::EncoderArm & msg)
  : msg_(msg)
  {}
  Init_EncoderArm_arm_node4 arm_node3(::msg_interfaces::msg::EncoderArm::_arm_node3_type arg)
  {
    msg_.arm_node3 = std::move(arg);
    return Init_EncoderArm_arm_node4(msg_);
  }

private:
  ::msg_interfaces::msg::EncoderArm msg_;
};

class Init_EncoderArm_arm_node2
{
public:
  explicit Init_EncoderArm_arm_node2(::msg_interfaces::msg::EncoderArm & msg)
  : msg_(msg)
  {}
  Init_EncoderArm_arm_node3 arm_node2(::msg_interfaces::msg::EncoderArm::_arm_node2_type arg)
  {
    msg_.arm_node2 = std::move(arg);
    return Init_EncoderArm_arm_node3(msg_);
  }

private:
  ::msg_interfaces::msg::EncoderArm msg_;
};

class Init_EncoderArm_arm_node1
{
public:
  explicit Init_EncoderArm_arm_node1(::msg_interfaces::msg::EncoderArm & msg)
  : msg_(msg)
  {}
  Init_EncoderArm_arm_node2 arm_node1(::msg_interfaces::msg::EncoderArm::_arm_node1_type arg)
  {
    msg_.arm_node1 = std::move(arg);
    return Init_EncoderArm_arm_node2(msg_);
  }

private:
  ::msg_interfaces::msg::EncoderArm msg_;
};

class Init_EncoderArm_arm_node0
{
public:
  Init_EncoderArm_arm_node0()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EncoderArm_arm_node1 arm_node0(::msg_interfaces::msg::EncoderArm::_arm_node0_type arg)
  {
    msg_.arm_node0 = std::move(arg);
    return Init_EncoderArm_arm_node1(msg_);
  }

private:
  ::msg_interfaces::msg::EncoderArm msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_interfaces::msg::EncoderArm>()
{
  return msg_interfaces::msg::builder::Init_EncoderArm_arm_node0();
}

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__ENCODER_ARM__BUILDER_HPP_
