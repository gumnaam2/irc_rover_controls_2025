// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msg_interfaces:msg/TargetPose.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__TARGET_POSE__BUILDER_HPP_
#define MSG_INTERFACES__MSG__DETAIL__TARGET_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msg_interfaces/msg/detail/target_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msg_interfaces
{

namespace msg
{

namespace builder
{

class Init_TargetPose_wrist_rot
{
public:
  explicit Init_TargetPose_wrist_rot(::msg_interfaces::msg::TargetPose & msg)
  : msg_(msg)
  {}
  ::msg_interfaces::msg::TargetPose wrist_rot(::msg_interfaces::msg::TargetPose::_wrist_rot_type arg)
  {
    msg_.wrist_rot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msg_interfaces::msg::TargetPose msg_;
};

class Init_TargetPose_wrist_up_down
{
public:
  explicit Init_TargetPose_wrist_up_down(::msg_interfaces::msg::TargetPose & msg)
  : msg_(msg)
  {}
  Init_TargetPose_wrist_rot wrist_up_down(::msg_interfaces::msg::TargetPose::_wrist_up_down_type arg)
  {
    msg_.wrist_up_down = std::move(arg);
    return Init_TargetPose_wrist_rot(msg_);
  }

private:
  ::msg_interfaces::msg::TargetPose msg_;
};

class Init_TargetPose_elbow
{
public:
  explicit Init_TargetPose_elbow(::msg_interfaces::msg::TargetPose & msg)
  : msg_(msg)
  {}
  Init_TargetPose_wrist_up_down elbow(::msg_interfaces::msg::TargetPose::_elbow_type arg)
  {
    msg_.elbow = std::move(arg);
    return Init_TargetPose_wrist_up_down(msg_);
  }

private:
  ::msg_interfaces::msg::TargetPose msg_;
};

class Init_TargetPose_shoulder
{
public:
  explicit Init_TargetPose_shoulder(::msg_interfaces::msg::TargetPose & msg)
  : msg_(msg)
  {}
  Init_TargetPose_elbow shoulder(::msg_interfaces::msg::TargetPose::_shoulder_type arg)
  {
    msg_.shoulder = std::move(arg);
    return Init_TargetPose_elbow(msg_);
  }

private:
  ::msg_interfaces::msg::TargetPose msg_;
};

class Init_TargetPose_base
{
public:
  Init_TargetPose_base()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetPose_shoulder base(::msg_interfaces::msg::TargetPose::_base_type arg)
  {
    msg_.base = std::move(arg);
    return Init_TargetPose_shoulder(msg_);
  }

private:
  ::msg_interfaces::msg::TargetPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msg_interfaces::msg::TargetPose>()
{
  return msg_interfaces::msg::builder::Init_TargetPose_base();
}

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__TARGET_POSE__BUILDER_HPP_
