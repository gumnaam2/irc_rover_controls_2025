// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_interfaces:msg/MotorSix.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__MOTOR_SIX__TRAITS_HPP_
#define MSG_INTERFACES__MSG__DETAIL__MOTOR_SIX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_interfaces/msg/detail/motor_six__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorSix & msg,
  std::ostream & out)
{
  out << "{";
  // member: motor1_speed
  {
    out << "motor1_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.motor1_speed, out);
    out << ", ";
  }

  // member: motor1_direction
  {
    out << "motor1_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.motor1_direction, out);
    out << ", ";
  }

  // member: motor2_speed
  {
    out << "motor2_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.motor2_speed, out);
    out << ", ";
  }

  // member: motor2_direction
  {
    out << "motor2_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.motor2_direction, out);
    out << ", ";
  }

  // member: motor3_speed
  {
    out << "motor3_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.motor3_speed, out);
    out << ", ";
  }

  // member: motor3_direction
  {
    out << "motor3_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.motor3_direction, out);
    out << ", ";
  }

  // member: motor4_speed
  {
    out << "motor4_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.motor4_speed, out);
    out << ", ";
  }

  // member: motor4_direction
  {
    out << "motor4_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.motor4_direction, out);
    out << ", ";
  }

  // member: motor5_speed
  {
    out << "motor5_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.motor5_speed, out);
    out << ", ";
  }

  // member: motor5_direction
  {
    out << "motor5_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.motor5_direction, out);
    out << ", ";
  }

  // member: motor6_speed
  {
    out << "motor6_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.motor6_speed, out);
    out << ", ";
  }

  // member: motor6_direction
  {
    out << "motor6_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.motor6_direction, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorSix & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: motor1_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor1_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.motor1_speed, out);
    out << "\n";
  }

  // member: motor1_direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor1_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.motor1_direction, out);
    out << "\n";
  }

  // member: motor2_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor2_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.motor2_speed, out);
    out << "\n";
  }

  // member: motor2_direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor2_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.motor2_direction, out);
    out << "\n";
  }

  // member: motor3_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor3_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.motor3_speed, out);
    out << "\n";
  }

  // member: motor3_direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor3_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.motor3_direction, out);
    out << "\n";
  }

  // member: motor4_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor4_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.motor4_speed, out);
    out << "\n";
  }

  // member: motor4_direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor4_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.motor4_direction, out);
    out << "\n";
  }

  // member: motor5_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor5_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.motor5_speed, out);
    out << "\n";
  }

  // member: motor5_direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor5_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.motor5_direction, out);
    out << "\n";
  }

  // member: motor6_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor6_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.motor6_speed, out);
    out << "\n";
  }

  // member: motor6_direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor6_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.motor6_direction, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorSix & msg, bool use_flow_style = false)
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
  const msg_interfaces::msg::MotorSix & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_interfaces::msg::MotorSix & msg)
{
  return msg_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_interfaces::msg::MotorSix>()
{
  return "msg_interfaces::msg::MotorSix";
}

template<>
inline const char * name<msg_interfaces::msg::MotorSix>()
{
  return "msg_interfaces/msg/MotorSix";
}

template<>
struct has_fixed_size<msg_interfaces::msg::MotorSix>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<msg_interfaces::msg::MotorSix>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<msg_interfaces::msg::MotorSix>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_INTERFACES__MSG__DETAIL__MOTOR_SIX__TRAITS_HPP_
