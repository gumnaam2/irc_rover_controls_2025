// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_interfaces:msg/EncoderDrive.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ENCODER_DRIVE__TRAITS_HPP_
#define MSG_INTERFACES__MSG__DETAIL__ENCODER_DRIVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_interfaces/msg/detail/encoder_drive__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const EncoderDrive & msg,
  std::ostream & out)
{
  out << "{";
  // member: drive_node0
  {
    if (msg.drive_node0.size() == 0) {
      out << "drive_node0: []";
    } else {
      out << "drive_node0: [";
      size_t pending_items = msg.drive_node0.size();
      for (auto item : msg.drive_node0) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: drive_node1
  {
    if (msg.drive_node1.size() == 0) {
      out << "drive_node1: []";
    } else {
      out << "drive_node1: [";
      size_t pending_items = msg.drive_node1.size();
      for (auto item : msg.drive_node1) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: drive_node2
  {
    if (msg.drive_node2.size() == 0) {
      out << "drive_node2: []";
    } else {
      out << "drive_node2: [";
      size_t pending_items = msg.drive_node2.size();
      for (auto item : msg.drive_node2) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: drive_node3
  {
    if (msg.drive_node3.size() == 0) {
      out << "drive_node3: []";
    } else {
      out << "drive_node3: [";
      size_t pending_items = msg.drive_node3.size();
      for (auto item : msg.drive_node3) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: drive_node4
  {
    if (msg.drive_node4.size() == 0) {
      out << "drive_node4: []";
    } else {
      out << "drive_node4: [";
      size_t pending_items = msg.drive_node4.size();
      for (auto item : msg.drive_node4) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: drive_node5
  {
    if (msg.drive_node5.size() == 0) {
      out << "drive_node5: []";
    } else {
      out << "drive_node5: [";
      size_t pending_items = msg.drive_node5.size();
      for (auto item : msg.drive_node5) {
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
  const EncoderDrive & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: drive_node0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.drive_node0.size() == 0) {
      out << "drive_node0: []\n";
    } else {
      out << "drive_node0:\n";
      for (auto item : msg.drive_node0) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: drive_node1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.drive_node1.size() == 0) {
      out << "drive_node1: []\n";
    } else {
      out << "drive_node1:\n";
      for (auto item : msg.drive_node1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: drive_node2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.drive_node2.size() == 0) {
      out << "drive_node2: []\n";
    } else {
      out << "drive_node2:\n";
      for (auto item : msg.drive_node2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: drive_node3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.drive_node3.size() == 0) {
      out << "drive_node3: []\n";
    } else {
      out << "drive_node3:\n";
      for (auto item : msg.drive_node3) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: drive_node4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.drive_node4.size() == 0) {
      out << "drive_node4: []\n";
    } else {
      out << "drive_node4:\n";
      for (auto item : msg.drive_node4) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: drive_node5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.drive_node5.size() == 0) {
      out << "drive_node5: []\n";
    } else {
      out << "drive_node5:\n";
      for (auto item : msg.drive_node5) {
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

inline std::string to_yaml(const EncoderDrive & msg, bool use_flow_style = false)
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
  const msg_interfaces::msg::EncoderDrive & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_interfaces::msg::EncoderDrive & msg)
{
  return msg_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_interfaces::msg::EncoderDrive>()
{
  return "msg_interfaces::msg::EncoderDrive";
}

template<>
inline const char * name<msg_interfaces::msg::EncoderDrive>()
{
  return "msg_interfaces/msg/EncoderDrive";
}

template<>
struct has_fixed_size<msg_interfaces::msg::EncoderDrive>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<msg_interfaces::msg::EncoderDrive>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<msg_interfaces::msg::EncoderDrive>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_INTERFACES__MSG__DETAIL__ENCODER_DRIVE__TRAITS_HPP_
