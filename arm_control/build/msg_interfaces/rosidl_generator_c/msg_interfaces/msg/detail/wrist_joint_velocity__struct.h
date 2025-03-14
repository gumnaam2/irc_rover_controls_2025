// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_interfaces:msg/WristJointVelocity.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__WRIST_JOINT_VELOCITY__STRUCT_H_
#define MSG_INTERFACES__MSG__DETAIL__WRIST_JOINT_VELOCITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'angular_speed'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/WristJointVelocity in the package msg_interfaces.
typedef struct msg_interfaces__msg__WristJointVelocity
{
  rosidl_runtime_c__double__Sequence angular_speed;
} msg_interfaces__msg__WristJointVelocity;

// Struct for a sequence of msg_interfaces__msg__WristJointVelocity.
typedef struct msg_interfaces__msg__WristJointVelocity__Sequence
{
  msg_interfaces__msg__WristJointVelocity * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_interfaces__msg__WristJointVelocity__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_INTERFACES__MSG__DETAIL__WRIST_JOINT_VELOCITY__STRUCT_H_
