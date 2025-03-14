// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_interfaces:msg/EncoderArm.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ENCODER_ARM__TRAITS_HPP_
#define MSG_INTERFACES__MSG__DETAIL__ENCODER_ARM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_interfaces/msg/detail/encoder_arm__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const EncoderArm & msg,
  std::ostream & out)
{
  out << "{";
  // member: arm_node0
  {
    if (msg.arm_node0.size() == 0) {
      out << "arm_node0: []";
    } else {
      out << "arm_node0: [";
      size_t pending_items = msg.arm_node0.size();
      for (auto item : msg.arm_node0) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: arm_node1
  {
    if (msg.arm_node1.size() == 0) {
      out << "arm_node1: []";
    } else {
      out << "arm_node1: [";
      size_t pending_items = msg.arm_node1.size();
      for (auto item : msg.arm_node1) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: arm_node2
  {
    if (msg.arm_node2.size() == 0) {
      out << "arm_node2: []";
    } else {
      out << "arm_node2: [";
      size_t pending_items = msg.arm_node2.size();
      for (auto item : msg.arm_node2) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: arm_node3
  {
    if (msg.arm_node3.size() == 0) {
      out << "arm_node3: []";
    } else {
      out << "arm_node3: [";
      size_t pending_items = msg.arm_node3.size();
      for (auto item : msg.arm_node3) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: arm_node4
  {
    if (msg.arm_node4.size() == 0) {
      out << "arm_node4: []";
    } else {
      out << "arm_node4: [";
      size_t pending_items = msg.arm_node4.size();
      for (auto item : msg.arm_node4) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: arm_node5
  {
    if (msg.arm_node5.size() == 0) {
      out << "arm_node5: []";
    } else {
      out << "arm_node5: [";
      size_t pending_items = msg.arm_node5.size();
      for (auto item : msg.arm_node5) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EncoderArm & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: arm_node0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.arm_node0.size() == 0) {
      out << "arm_node0: []\n";
    } else {
      out << "arm_node0:\n";
      for (auto item : msg.arm_node0) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: arm_node1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.arm_node1.size() == 0) {
      out << "arm_node1: []\n";
    } else {
      out << "arm_node1:\n";
      for (auto item : msg.arm_node1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: arm_node2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.arm_node2.size() == 0) {
      out << "arm_node2: []\n";
    } else {
      out << "arm_node2:\n";
      for (auto item : msg.arm_node2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: arm_node3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.arm_node3.size() == 0) {
      out << "arm_node3: []\n";
    } else {
      out << "arm_node3:\n";
      for (auto item : msg.arm_node3) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: arm_node4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.arm_node4.size() == 0) {
      out << "arm_node4: []\n";
    } else {
      out << "arm_node4:\n";
      for (auto item : msg.arm_node4) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: arm_node5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.arm_node5.size() == 0) {
      out << "arm_node5: []\n";
    } else {
      out << "arm_node5:\n";
      for (auto item : msg.arm_node5) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EncoderArm & msg, bool use_flow_style = false)
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
  const msg_interfaces::msg::EncoderArm & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_interfaces::msg::EncoderArm & msg)
{
  return msg_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_interfaces::msg::EncoderArm>()
{
  return "msg_interfaces::msg::EncoderArm";
}

template<>
inline const char * name<msg_interfaces::msg::EncoderArm>()
{
  return "msg_interfaces/msg/EncoderArm";
}

template<>
struct has_fixed_size<msg_interfaces::msg::EncoderArm>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<msg_interfaces::msg::EncoderArm>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<msg_interfaces::msg::EncoderArm>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_INTERFACES__MSG__DETAIL__ENCODER_ARM__TRAITS_HPP_
