// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_interfaces:msg/ShoulderJointVelocity.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__SHOULDER_JOINT_VELOCITY__TRAITS_HPP_
#define MSG_INTERFACES__MSG__DETAIL__SHOULDER_JOINT_VELOCITY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_interfaces/msg/detail/shoulder_joint_velocity__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ShoulderJointVelocity & msg,
  std::ostream & out)
{
  out << "{";
  // member: angular_speed
  {
    out << "angular_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_speed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ShoulderJointVelocity & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angular_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_speed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ShoulderJointVelocity & msg, bool use_flow_style = false)
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
  const msg_interfaces::msg::ShoulderJointVelocity & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_interfaces::msg::ShoulderJointVelocity & msg)
{
  return msg_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_interfaces::msg::ShoulderJointVelocity>()
{
  return "msg_interfaces::msg::ShoulderJointVelocity";
}

template<>
inline const char * name<msg_interfaces::msg::ShoulderJointVelocity>()
{
  return "msg_interfaces/msg/ShoulderJointVelocity";
}

template<>
struct has_fixed_size<msg_interfaces::msg::ShoulderJointVelocity>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msg_interfaces::msg::ShoulderJointVelocity>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msg_interfaces::msg::ShoulderJointVelocity>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_INTERFACES__MSG__DETAIL__SHOULDER_JOINT_VELOCITY__TRAITS_HPP_
