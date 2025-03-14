// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from msg_interfaces:msg/Drive.idl
// generated code does not contain a copyright notice
#include "msg_interfaces/msg/detail/drive__rosidl_typesupport_fastrtps_cpp.hpp"
#include "msg_interfaces/msg/detail/drive__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace msg_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_interfaces
cdr_serialize(
  const msg_interfaces::msg::Drive & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: direction
  {
    cdr << ros_message.direction;
  }
  // Member: speed
  {
    cdr << ros_message.speed;
  }
  // Member: sys_check
  cdr << (ros_message.sys_check ? true : false);
  // Member: reset
  cdr << (ros_message.reset ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  msg_interfaces::msg::Drive & ros_message)
{
  // Member: direction
  {
    cdr >> ros_message.direction;
  }

  // Member: speed
  {
    cdr >> ros_message.speed;
  }

  // Member: sys_check
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.sys_check = tmp ? true : false;
  }

  // Member: reset
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reset = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_interfaces
get_serialized_size(
  const msg_interfaces::msg::Drive & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: direction
  {
    size_t array_size = ros_message.direction.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.direction[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speed
  {
    size_t array_size = ros_message.speed.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.speed[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sys_check
  {
    size_t item_size = sizeof(ros_message.sys_check);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reset
  {
    size_t item_size = sizeof(ros_message.reset);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_interfaces
max_serialized_size_Drive(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: direction
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: speed
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: sys_check
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reset
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = msg_interfaces::msg::Drive;
    is_plain =
      (
      offsetof(DataType, reset) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Drive__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const msg_interfaces::msg::Drive *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Drive__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<msg_interfaces::msg::Drive *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Drive__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const msg_interfaces::msg::Drive *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Drive__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Drive(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Drive__callbacks = {
  "msg_interfaces::msg",
  "Drive",
  _Drive__cdr_serialize,
  _Drive__cdr_deserialize,
  _Drive__get_serialized_size,
  _Drive__max_serialized_size
};

static rosidl_message_type_support_t _Drive__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Drive__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace msg_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_msg_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<msg_interfaces::msg::Drive>()
{
  return &msg_interfaces::msg::typesupport_fastrtps_cpp::_Drive__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, msg_interfaces, msg, Drive)() {
  return &msg_interfaces::msg::typesupport_fastrtps_cpp::_Drive__handle;
}

#ifdef __cplusplus
}
#endif
