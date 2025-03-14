// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_interfaces:msg/FinalVel.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__FINAL_VEL__TRAITS_HPP_
#define MSG_INTERFACES__MSG__DETAIL__FINAL_VEL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_interfaces/msg/detail/final_vel__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const FinalVel & msg,
  std::ostream & out)
{
  out << "{";
  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << ", ";
  }

  // member: direction
  {
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FinalVel & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }

  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FinalVel & msg, bool use_flow_style = false)
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
  const msg_interfaces::msg::FinalVel & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_interfaces::msg::FinalVel & msg)
{
  return msg_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_interfaces::msg::FinalVel>()
{
  return "msg_interfaces::msg::FinalVel";
}

template<>
inline const char * name<msg_interfaces::msg::FinalVel>()
{
  return "msg_interfaces/msg/FinalVel";
}

template<>
struct has_fixed_size<msg_interfaces::msg::FinalVel>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msg_interfaces::msg::FinalVel>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msg_interfaces::msg::FinalVel>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_INTERFACES__MSG__DETAIL__FINAL_VEL__TRAITS_HPP_
