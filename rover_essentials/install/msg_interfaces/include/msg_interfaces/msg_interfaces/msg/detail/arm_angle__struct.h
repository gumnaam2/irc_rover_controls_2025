// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_interfaces:msg/ArmAngle.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ARM_ANGLE__STRUCT_H_
#define MSG_INTERFACES__MSG__DETAIL__ARM_ANGLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ArmAngle in the package msg_interfaces.
typedef struct msg_interfaces__msg__ArmAngle
{
  double alpha;
  double beta;
} msg_interfaces__msg__ArmAngle;

// Struct for a sequence of msg_interfaces__msg__ArmAngle.
typedef struct msg_interfaces__msg__ArmAngle__Sequence
{
  msg_interfaces__msg__ArmAngle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_interfaces__msg__ArmAngle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_INTERFACES__MSG__DETAIL__ARM_ANGLE__STRUCT_H_
