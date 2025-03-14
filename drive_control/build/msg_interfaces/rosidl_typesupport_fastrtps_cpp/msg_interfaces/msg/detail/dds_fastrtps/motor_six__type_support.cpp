// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from msg_interfaces:msg/MotorSix.idl
// generated code does not contain a copyright notice
#include "msg_interfaces/msg/detail/motor_six__rosidl_typesupport_fastrtps_cpp.hpp"
#include "msg_interfaces/msg/detail/motor_six__struct.hpp"

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
  const msg_interfaces::msg::MotorSix & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: motor1_speed
  cdr << ros_message.motor1_speed;
  // Member: motor1_direction
  cdr << ros_message.motor1_direction;
  // Member: motor2_speed
  cdr << ros_message.motor2_speed;
  // Member: motor2_direction
  cdr << ros_message.motor2_direction;
  // Member: motor3_speed
  cdr << ros_message.motor3_speed;
  // Member: motor3_direction
  cdr << ros_message.motor3_direction;
  // Member: motor4_speed
  cdr << ros_message.motor4_speed;
  // Member: motor4_direction
  cdr << ros_message.motor4_direction;
  // Member: motor5_speed
  cdr << ros_message.motor5_speed;
  // Member: motor5_direction
  cdr << ros_message.motor5_direction;
  // Member: motor6_speed
  cdr << ros_message.motor6_speed;
  // Member: motor6_direction
  cdr << ros_message.motor6_direction;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  msg_interfaces::msg::MotorSix & ros_message)
{
  // Member: motor1_speed
  cdr >> ros_message.motor1_speed;

  // Member: motor1_direction
  cdr >> ros_message.motor1_direction;

  // Member: motor2_speed
  cdr >> ros_message.motor2_speed;

  // Member: motor2_direction
  cdr >> ros_message.motor2_direction;

  // Member: motor3_speed
  cdr >> ros_message.motor3_speed;

  // Member: motor3_direction
  cdr >> ros_message.motor3_direction;

  // Member: motor4_speed
  cdr >> ros_message.motor4_speed;

  // Member: motor4_direction
  cdr >> ros_message.motor4_direction;

  // Member: motor5_speed
  cdr >> ros_message.motor5_speed;

  // Member: motor5_direction
  cdr >> ros_message.motor5_direction;

  // Member: motor6_speed
  cdr >> ros_message.motor6_speed;

  // Member: motor6_direction
  cdr >> ros_message.motor6_direction;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_interfaces
get_serialized_size(
  const msg_interfaces::msg::MotorSix & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: motor1_speed
  {
    size_t item_size = sizeof(ros_message.motor1_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor1_direction
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.motor1_direction.size() + 1);
  // Member: motor2_speed
  {
    size_t item_size = sizeof(ros_message.motor2_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor2_direction
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.motor2_direction.size() + 1);
  // Member: motor3_speed
  {
    size_t item_size = sizeof(ros_message.motor3_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor3_direction
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.motor3_direction.size() + 1);
  // Member: motor4_speed
  {
    size_t item_size = sizeof(ros_message.motor4_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor4_direction
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.motor4_direction.size() + 1);
  // Member: motor5_speed
  {
    size_t item_size = sizeof(ros_message.motor5_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor5_direction
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.motor5_direction.size() + 1);
  // Member: motor6_speed
  {
    size_t item_size = sizeof(ros_message.motor6_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor6_direction
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.motor6_direction.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_interfaces
max_serialized_size_MotorSix(
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


  // Member: motor1_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: motor1_direction
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: motor2_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: motor2_direction
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: motor3_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: motor3_direction
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: motor4_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: motor4_direction
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: motor5_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: motor5_direction
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: motor6_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: motor6_direction
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = msg_interfaces::msg::MotorSix;
    is_plain =
      (
      offsetof(DataType, motor6_direction) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _MotorSix__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const msg_interfaces::msg::MotorSix *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MotorSix__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<msg_interfaces::msg::MotorSix *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MotorSix__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const msg_interfaces::msg::MotorSix *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MotorSix__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_MotorSix(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _MotorSix__callbacks = {
  "msg_interfaces::msg",
  "MotorSix",
  _MotorSix__cdr_serialize,
  _MotorSix__cdr_deserialize,
  _MotorSix__get_serialized_size,
  _MotorSix__max_serialized_size
};

static rosidl_message_type_support_t _MotorSix__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MotorSix__callbacks,
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
get_message_type_support_handle<msg_interfaces::msg::MotorSix>()
{
  return &msg_interfaces::msg::typesupport_fastrtps_cpp::_MotorSix__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, msg_interfaces, msg, MotorSix)() {
  return &msg_interfaces::msg::typesupport_fastrtps_cpp::_MotorSix__handle;
}

#ifdef __cplusplus
}
#endif
