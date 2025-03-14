// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_interfaces:msg/TargetPose.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__TARGET_POSE__TRAITS_HPP_
#define MSG_INTERFACES__MSG__DETAIL__TARGET_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_interfaces/msg/detail/target_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TargetPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: base
  {
    out << "base: ";
    rosidl_generator_traits::value_to_yaml(msg.base, out);
    out << ", ";
  }

  // member: shoulder
  {
    out << "shoulder: ";
    rosidl_generator_traits::value_to_yaml(msg.shoulder, out);
    out << ", ";
  }

  // member: elbow
  {
    out << "elbow: ";
    rosidl_generator_traits::value_to_yaml(msg.elbow, out);
    out << ", ";
  }

  // member: wrist_up_down
  {
    out << "wrist_up_down: ";
    rosidl_generator_traits::value_to_yaml(msg.wrist_up_down, out);
    out << ", ";
  }

  // member: wrist_rot
  {
    out << "wrist_rot: ";
    rosidl_generator_traits::value_to_yaml(msg.wrist_rot, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TargetPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: base
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "base: ";
    rosidl_generator_traits::value_to_yaml(msg.base, out);
    out << "\n";
  }

  // member: shoulder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shoulder: ";
    rosidl_generator_traits::value_to_yaml(msg.shoulder, out);
    out << "\n";
  }

  // member: elbow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "elbow: ";
    rosidl_generator_traits::value_to_yaml(msg.elbow, out);
    out << "\n";
  }

  // member: wrist_up_down
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wrist_up_down: ";
    rosidl_generator_traits::value_to_yaml(msg.wrist_up_down, out);
    out << "\n";
  }

  // member: wrist_rot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wrist_rot: ";
    rosidl_generator_traits::value_to_yaml(msg.wrist_rot, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TargetPose & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace msg_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use msg_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const msg_interfaces::msg::TargetPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_interfaces::msg::TargetPose & msg)
{
  return msg_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_interfaces::msg::TargetPose>()
{
  return "msg_interfaces::msg::TargetPose";
}

template<>
inline const char * name<msg_interfaces::msg::TargetPose>()
{
  return "msg_interfaces/msg/TargetPose";
}

template<>
struct has_fixed_size<msg_interfaces::msg::TargetPose>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msg_interfaces::msg::TargetPose>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msg_interfaces::msg::TargetPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_INTERFACES__MSG__DETAIL__TARGET_POSE__TRAITS_HPP_
