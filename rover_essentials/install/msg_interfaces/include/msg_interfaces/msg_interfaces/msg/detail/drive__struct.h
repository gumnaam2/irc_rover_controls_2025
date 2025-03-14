// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_interfaces:msg/Drive.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__DRIVE__STRUCT_H_
#define MSG_INTERFACES__MSG__DETAIL__DRIVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'direction'
// Member 'speed'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Drive in the package msg_interfaces.
typedef struct msg_interfaces__msg__Drive
{
  rosidl_runtime_c__int64__Sequence direction;
  rosidl_runtime_c__int64__Sequence speed;
  bool sys_check;
  bool reset;
} msg_interfaces__msg__Drive;

// Struct for a sequence of msg_interfaces__msg__Drive.
typedef struct msg_interfaces__msg__Drive__Sequence
{
  msg_interfaces__msg__Drive * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_interfaces__msg__Drive__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_INTERFACES__MSG__DETAIL__DRIVE__STRUCT_H_
