// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_interfaces:msg/BaseMotion.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__BASE_MOTION__TRAITS_HPP_
#define MSG_INTERFACES__MSG__DETAIL__BASE_MOTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_interfaces/msg/detail/base_motion__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const BaseMotion & msg,
  std::ostream & out)
{
  out << "{";
  // member: direction
  {
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BaseMotion & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction: ";
    rosidl_generator_traits::value_to_yaml(msg.direction, out);
    out << "\n";
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BaseMotion & msg, bool use_flow_style = false)
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
  const msg_interfaces::msg::BaseMotion & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_interfaces::msg::BaseMotion & msg)
{
  return msg_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_interfaces::msg::BaseMotion>()
{
  return "msg_interfaces::msg::BaseMotion";
}

template<>
inline const char * name<msg_interfaces::msg::BaseMotion>()
{
  return "msg_interfaces/msg/BaseMotion";
}

template<>
struct has_fixed_size<msg_interfaces::msg::BaseMotion>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msg_interfaces::msg::BaseMotion>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msg_interfaces::msg::BaseMotion>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_INTERFACES__MSG__DETAIL__BASE_MOTION__TRAITS_HPP_
