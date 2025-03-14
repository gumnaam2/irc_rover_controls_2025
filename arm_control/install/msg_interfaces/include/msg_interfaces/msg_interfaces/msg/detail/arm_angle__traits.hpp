// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_interfaces:msg/ArmAngle.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ARM_ANGLE__TRAITS_HPP_
#define MSG_INTERFACES__MSG__DETAIL__ARM_ANGLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_interfaces/msg/detail/arm_angle__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ArmAngle & msg,
  std::ostream & out)
{
  out << "{";
  // member: alpha
  {
    out << "alpha: ";
    rosidl_generator_traits::value_to_yaml(msg.alpha, out);
    out << ", ";
  }

  // member: beta
  {
    out << "beta: ";
    rosidl_generator_traits::value_to_yaml(msg.beta, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmAngle & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: alpha
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alpha: ";
    rosidl_generator_traits::value_to_yaml(msg.alpha, out);
    out << "\n";
  }

  // member: beta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "beta: ";
    rosidl_generator_traits::value_to_yaml(msg.beta, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmAngle & msg, bool use_flow_style = false)
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
  const msg_interfaces::msg::ArmAngle & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_interfaces::msg::ArmAngle & msg)
{
  return msg_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_interfaces::msg::ArmAngle>()
{
  return "msg_interfaces::msg::ArmAngle";
}

template<>
inline const char * name<msg_interfaces::msg::ArmAngle>()
{
  return "msg_interfaces/msg/ArmAngle";
}

template<>
struct has_fixed_size<msg_interfaces::msg::ArmAngle>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msg_interfaces::msg::ArmAngle>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msg_interfaces::msg::ArmAngle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_INTERFACES__MSG__DETAIL__ARM_ANGLE__TRAITS_HPP_
