// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from msg_interfaces:msg/MotorSix.idl
// generated code does not contain a copyright notice
#include "msg_interfaces/msg/detail/motor_six__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "msg_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "msg_interfaces/msg/detail/motor_six__struct.h"
#include "msg_interfaces/msg/detail/motor_six__functions.h"
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

#include "rosidl_runtime_c/string.h"  // motor1_direction, motor2_direction, motor3_direction, motor4_direction, motor5_direction, motor6_direction
#include "rosidl_runtime_c/string_functions.h"  // motor1_direction, motor2_direction, motor3_direction, motor4_direction, motor5_direction, motor6_direction

// forward declare type support functions


using _MotorSix__ros_msg_type = msg_interfaces__msg__MotorSix;

static bool _MotorSix__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MotorSix__ros_msg_type * ros_message = static_cast<const _MotorSix__ros_msg_type *>(untyped_ros_message);
  // Field name: motor1_speed
  {
    cdr << ros_message->motor1_speed;
  }

  // Field name: motor1_direction
  {
    const rosidl_runtime_c__String * str = &ros_message->motor1_direction;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: motor2_speed
  {
    cdr << ros_message->motor2_speed;
  }

  // Field name: motor2_direction
  {
    const rosidl_runtime_c__String * str = &ros_message->motor2_direction;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: motor3_speed
  {
    cdr << ros_message->motor3_speed;
  }

  // Field name: motor3_direction
  {
    const rosidl_runtime_c__String * str = &ros_message->motor3_direction;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: motor4_speed
  {
    cdr << ros_message->motor4_speed;
  }

  // Field name: motor4_direction
  {
    const rosidl_runtime_c__String * str = &ros_message->motor4_direction;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: motor5_speed
  {
    cdr << ros_message->motor5_speed;
  }

  // Field name: motor5_direction
  {
    const rosidl_runtime_c__String * str = &ros_message->motor5_direction;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: motor6_speed
  {
    cdr << ros_message->motor6_speed;
  }

  // Field name: motor6_direction
  {
    const rosidl_runtime_c__String * str = &ros_message->motor6_direction;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _MotorSix__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MotorSix__ros_msg_type * ros_message = static_cast<_MotorSix__ros_msg_type *>(untyped_ros_message);
  // Field name: motor1_speed
  {
    cdr >> ros_message->motor1_speed;
  }

  // Field name: motor1_direction
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->motor1_direction.data) {
      rosidl_runtime_c__String__init(&ros_message->motor1_direction);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->motor1_direction,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'motor1_direction'\n");
      return false;
    }
  }

  // Field name: motor2_speed
  {
    cdr >> ros_message->motor2_speed;
  }

  // Field name: motor2_direction
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->motor2_direction.data) {
      rosidl_runtime_c__String__init(&ros_message->motor2_direction);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->motor2_direction,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'motor2_direction'\n");
      return false;
    }
  }

  // Field name: motor3_speed
  {
    cdr >> ros_message->motor3_speed;
  }

  // Field name: motor3_direction
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->motor3_direction.data) {
      rosidl_runtime_c__String__init(&ros_message->motor3_direction);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->motor3_direction,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'motor3_direction'\n");
      return false;
    }
  }

  // Field name: motor4_speed
  {
    cdr >> ros_message->motor4_speed;
  }

  // Field name: motor4_direction
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->motor4_direction.data) {
      rosidl_runtime_c__String__init(&ros_message->motor4_direction);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->motor4_direction,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'motor4_direction'\n");
      return false;
    }
  }

  // Field name: motor5_speed
  {
    cdr >> ros_message->motor5_speed;
  }

  // Field name: motor5_direction
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->motor5_direction.data) {
      rosidl_runtime_c__String__init(&ros_message->motor5_direction);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->motor5_direction,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'motor5_direction'\n");
      return false;
    }
  }

  // Field name: motor6_speed
  {
    cdr >> ros_message->motor6_speed;
  }

  // Field name: motor6_direction
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->motor6_direction.data) {
      rosidl_runtime_c__String__init(&ros_message->motor6_direction);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->motor6_direction,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'motor6_direction'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msg_interfaces
size_t get_serialized_size_msg_interfaces__msg__MotorSix(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MotorSix__ros_msg_type * ros_message = static_cast<const _MotorSix__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name motor1_speed
  {
    size_t item_size = sizeof(ros_message->motor1_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor1_direction
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->motor1_direction.size + 1);
  // field.name motor2_speed
  {
    size_t item_size = sizeof(ros_message->motor2_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor2_direction
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->motor2_direction.size + 1);
  // field.name motor3_speed
  {
    size_t item_size = sizeof(ros_message->motor3_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor3_direction
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->motor3_direction.size + 1);
  // field.name motor4_speed
  {
    size_t item_size = sizeof(ros_message->motor4_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor4_direction
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->motor4_direction.size + 1);
  // field.name motor5_speed
  {
    size_t item_size = sizeof(ros_message->motor5_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor5_direction
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->motor5_direction.size + 1);
  // field.name motor6_speed
  {
    size_t item_size = sizeof(ros_message->motor6_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor6_direction
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->motor6_direction.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _MotorSix__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_msg_interfaces__msg__MotorSix(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_msg_interfaces
size_t max_serialized_size_msg_interfaces__msg__MotorSix(
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

  // member: motor1_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: motor1_direction
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
  // member: motor2_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: motor2_direction
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
  // member: motor3_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: motor3_direction
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
  // member: motor4_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: motor4_direction
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
  // member: motor5_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: motor5_direction
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
  // member: motor6_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: motor6_direction
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
    using DataType = msg_interfaces__msg__MotorSix;
    is_plain =
      (
      offsetof(DataType, motor6_direction) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MotorSix__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_msg_interfaces__msg__MotorSix(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MotorSix = {
  "msg_interfaces::msg",
  "MotorSix",
  _MotorSix__cdr_serialize,
  _MotorSix__cdr_deserialize,
  _MotorSix__get_serialized_size,
  _MotorSix__max_serialized_size
};

static rosidl_message_type_support_t _MotorSix__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MotorSix,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, msg_interfaces, msg, MotorSix)() {
  return &_MotorSix__type_support;
}

#if defined(__cplusplus)
}
#endif
