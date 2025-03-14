// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from msg_interfaces:msg/EncoderDrive.idl
// generated code does not contain a copyright notice
#include "msg_interfaces/msg/detail/encoder_drive__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "msg_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "msg_interfaces/msg/detail/encoder_drive__struct.h"
#include "msg_interfaces/msg/detail/encoder_drive__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // drive_node0, drive_node1, drive_node2, drive_node3, drive_node4, drive_node5
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // drive_node0, drive_node1, drive_node2, drive_node3, drive_node4, drive_node5

// forward declare type support functions


using _EncoderDrive__ros_msg_type = msg_interfaces__msg__EncoderDrive;

static bool _EncoderDrive__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _EncoderDrive__ros_msg_type * ros_message = static_cast<const _EncoderDrive__ros_msg_type *>(untyped_ros_message);
  // Field name: drive_node0
  {
    size_t size = ros_message->drive_node0.size;
    auto array_ptr = ros_message->drive_node0.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: drive_node1
  {
    size_t size = ros_message->drive_node1.size;
    auto array_ptr = ros_message->drive_node1.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: drive_node2
  {
    size_t size = ros_message->drive_node2.size;
    auto array_ptr = ros_message->drive_node2.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: drive_node3
  {
    size_t size = ros_message->drive_node3.size;
    auto array_ptr = ros_message->drive_node3.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: drive_node4
  {
    size_t size = ros_message->drive_node4.size;
    auto array_ptr = ros_message->drive_node4.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: drive_node5
  {
    size_t size = ros_message->drive_node5.size;
    auto array_ptr = ros_message->drive_node5.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _EncoderDrive__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _EncoderDrive__ros_msg_type * ros_message = static_cast<_EncoderDrive__ros_msg_type *>(untyped_ros_message);
  // Field name: drive_node0
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->drive_node0.data) {
      rosidl_runtime_c__int64__Sequence__fini(&ros_message->drive_node0);
    }
    if (!rosidl_runtime_c__int64__Sequence__init(&ros_message->drive_node0, size)) {
      fprintf(stderr, "failed to create array for field 'drive_node0'");
      return false;
    }
    auto array_ptr = ros_message->drive_node0.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: drive_node1
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->drive_node1.data) {
      rosidl_runtime_c__int64__Sequence__fini(&ros_message->drive_node1);
    }
    if (!rosidl_runtime_c__int64__Sequence__init(&ros_message->drive_node1, size)) {
      fprintf(stderr, "failed to create array for field 'drive_node1'");
      return false;
    }
    auto array_ptr = ros_message->drive_node1.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: drive_node2
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->drive_node2.data) {
      rosidl_runtime_c__int64__Sequence__fini(&ros_message->drive_node2);
    }
    if (!rosidl_runtime_c__int64__Sequence__init(&ros_message->drive_node2, size)) {
      fprintf(stderr, "failed to create array for field 'drive_node2'");
      return false;
    }
    auto array_ptr = ros_message->drive_node2.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: drive_node3
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->drive_node3.data) {
      rosidl_runtime_c__int64__Sequence__fini(&ros_message->drive_node3);
    }
    if (!rosidl_runtime_c__int64__Sequence__init(&ros_message->drive_node3, size)) {
      fprintf(stderr, "failed to create array for field 'drive_node3'");
      return false;
    }
    auto array_ptr = ros_message->drive_node3.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: drive_node4
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->drive_node4.data) {
      rosidl_runtime_c__int64__Sequence__fini(&ros_message->drive_node4);
    }
    if (!rosidl_runtime_c__int64__Sequence__init(&ros_message->drive_node4, size)) {
      fprintf(stderr, "failed to create array for field 'drive_node4'");
      return false;
    }
    auto array_ptr = ros_message->drive_node4.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: drive_node5
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->drive_node5.data) {
      rosidl_runtime_c__int64__Sequence__fini(&ros_message->drive_node5);
    }
    if (!rosidl_runtime_c__int64__Sequence__init(&ros_message->drive_node5, size)) {
      fprintf(stderr, "failed to create array for field 'drive_node5'");
      return false;
    }
    auto array_ptr = ros_message->drive_node5.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msg_interfaces
size_t get_serialized_size_msg_interfaces__msg__EncoderDrive(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _EncoderDrive__ros_msg_type * ros_message = static_cast<const _EncoderDrive__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name drive_node0
  {
    size_t array_size = ros_message->drive_node0.size;
    auto array_ptr = ros_message->drive_node0.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name drive_node1
  {
    size_t array_size = ros_message->drive_node1.size;
    auto array_ptr = ros_message->drive_node1.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name drive_node2
  {
    size_t array_size = ros_message->drive_node2.size;
    auto array_ptr = ros_message->drive_node2.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name drive_node3
  {
    size_t array_size = ros_message->drive_node3.size;
    auto array_ptr = ros_message->drive_node3.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name drive_node4
  {
    size_t array_size = ros_message->drive_node4.size;
    auto array_ptr = ros_message->drive_node4.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name drive_node5
  {
    size_t array_size = ros_message->drive_node5.size;
    auto array_ptr = ros_message->drive_node5.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _EncoderDrive__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_msg_interfaces__msg__EncoderDrive(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msg_interfaces
size_t max_serialized_size_msg_interfaces__msg__EncoderDrive(
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

  // member: drive_node0
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
  // member: drive_node1
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
  // member: drive_node2
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
  // member: drive_node3
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
  // member: drive_node4
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
  // member: drive_node5
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

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = msg_interfaces__msg__EncoderDrive;
    is_plain =
      (
      offsetof(DataType, drive_node5) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _EncoderDrive__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_msg_interfaces__msg__EncoderDrive(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_EncoderDrive = {
  "msg_interfaces::msg",
  "EncoderDrive",
  _EncoderDrive__cdr_serialize,
  _EncoderDrive__cdr_deserialize,
  _EncoderDrive__get_serialized_size,
  _EncoderDrive__max_serialized_size
};

static rosidl_message_type_support_t _EncoderDrive__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_EncoderDrive,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, msg_interfaces, msg, EncoderDrive)() {
  return &_EncoderDrive__type_support;
}

#if defined(__cplusplus)
}
#endif
