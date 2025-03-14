// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_interfaces:msg/Drive.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__DRIVE__TRAITS_HPP_
#define MSG_INTERFACES__MSG__DETAIL__DRIVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_interfaces/msg/detail/drive__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Drive & msg,
  std::ostream & out)
{
  out << "{";
  // member: direction
  {
    if (msg.direction.size() == 0) {
      out << "direction: []";
    } else {
      out << "direction: [";
      size_t pending_items = msg.direction.size();
      for (auto item : msg.direction) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: speed
  {
    if (msg.speed.size() == 0) {
      out << "speed: []";
    } else {
      out << "speed: [";
      size_t pending_items = msg.speed.size();
      for (auto item : msg.speed) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: sys_check
  {
    out << "sys_check: ";
    rosidl_generator_traits::value_to_yaml(msg.sys_check, out);
    out << ", ";
  }

  // member: reset
  {
    out << "reset: ";
    rosidl_generator_traits::value_to_yaml(msg.reset, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Drive & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.direction.size() == 0) {
      out << "direction: []\n";
    } else {
      out << "direction:\n";
      for (auto item : msg.direction) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.speed.size() == 0) {
      out << "speed: []\n";
    } else {
      out << "speed:\n";
      for (auto item : msg.speed) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: sys_check
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sys_check: ";
    rosidl_generator_traits::value_to_yaml(msg.sys_check, out);
    out << "\n";
  }

  // member: reset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset: ";
    rosidl_generator_traits::value_to_yaml(msg.reset, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Drive & msg, bool use_flow_style = false)
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
  const msg_interfaces::msg::Drive & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_interfaces::msg::Drive & msg)
{
  return msg_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_interfaces::msg::Drive>()
{
  return "msg_interfaces::msg::Drive";
}

template<>
inline const char * name<msg_interfaces::msg::Drive>()
{
  return "msg_interfaces/msg/Drive";
}

template<>
struct has_fixed_size<msg_interfaces::msg::Drive>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<msg_interfaces::msg::Drive>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<msg_interfaces::msg::Drive>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_INTERFACES__MSG__DETAIL__DRIVE__TRAITS_HPP_
