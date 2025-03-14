// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from msg_interfaces:msg/ArmEndMotion.idl
// generated code does not contain a copyright notice
#include "msg_interfaces/msg/detail/arm_end_motion__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "msg_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "msg_interfaces/msg/detail/arm_end_motion__struct.h"
#include "msg_interfaces/msg/detail/arm_end_motion__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // direction, speed
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // direction, speed

// forward declare type support functions


using _ArmEndMotion__ros_msg_type = msg_interfaces__msg__ArmEndMotion;

static bool _ArmEndMotion__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ArmEndMotion__ros_msg_type * ros_message = static_cast<const _ArmEndMotion__ros_msg_type *>(untyped_ros_message);
  // Field name: direction
  {
    size_t size = ros_message->direction.size;
    auto array_ptr = ros_message->direction.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: speed
  {
    size_t size = ros_message->speed.size;
    auto array_ptr = ros_message->speed.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: sys_check
  {
    cdr << (ros_message->sys_check ? true : false);
  }

  // Field name: reset
  {
    cdr << (ros_message->reset ? true : false);
  }

  return true;
}

static bool _ArmEndMotion__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ArmEndMotion__ros_msg_type * ros_message = static_cast<_ArmEndMotion__ros_msg_type *>(untyped_ros_message);
  // Field name: direction
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->direction.data) {
      rosidl_runtime_c__int64__Sequence__fini(&ros_message->direction);
    }
    if (!rosidl_runtime_c__int64__Sequence__init(&ros_message->direction, size)) {
      fprintf(stderr, "failed to create array for field 'direction'");
      return false;
    }
    auto array_ptr = ros_message->direction.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: speed
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->speed.data) {
      rosidl_runtime_c__int64__Sequence__fini(&ros_message->speed);
    }
    if (!rosidl_runtime_c__int64__Sequence__init(&ros_message->speed, size)) {
      fprintf(stderr, "failed to create array for field 'speed'");
      return false;
    }
    auto array_ptr = ros_message->speed.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: sys_check
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->sys_check = tmp ? true : false;
  }

  // Field name: reset
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->reset = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msg_interfaces
size_t get_serialized_size_msg_interfaces__msg__ArmEndMotion(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ArmEndMotion__ros_msg_type * ros_message = static_cast<const _ArmEndMotion__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name direction
  {
    size_t array_size = ros_message->direction.size;
    auto array_ptr = ros_message->direction.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speed
  {
    size_t array_size = ros_message->speed.size;
    auto array_ptr = ros_message->speed.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sys_check
  {
    size_t item_size = sizeof(ros_message->sys_check);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name reset
  {
    size_t item_size = sizeof(ros_message->reset);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ArmEndMotion__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_msg_interfaces__msg__ArmEndMotion(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msg_interfaces
size_t max_serialized_size_msg_interfaces__msg__ArmEndMotion(
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

  // member: direction
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
  // member: speed
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
  // member: sys_check
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: reset
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
    using DataType = msg_interfaces__msg__ArmEndMotion;
    is_plain =
      (
      offsetof(DataType, reset) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ArmEndMotion__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_msg_interfaces__msg__ArmEndMotion(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ArmEndMotion = {
  "msg_interfaces::msg",
  "ArmEndMotion",
  _ArmEndMotion__cdr_serialize,
  _ArmEndMotion__cdr_deserialize,
  _ArmEndMotion__get_serialized_size,
  _ArmEndMotion__max_serialized_size
};

static rosidl_message_type_support_t _ArmEndMotion__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ArmEndMotion,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, msg_interfaces, msg, ArmEndMotion)() {
  return &_ArmEndMotion__type_support;
}

#if defined(__cplusplus)
}
#endif
