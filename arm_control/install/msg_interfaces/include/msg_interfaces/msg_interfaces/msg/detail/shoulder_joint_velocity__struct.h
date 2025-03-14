// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_interfaces:msg/ShoulderJointVelocity.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__SHOULDER_JOINT_VELOCITY__STRUCT_H_
#define MSG_INTERFACES__MSG__DETAIL__SHOULDER_JOINT_VELOCITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ShoulderJointVelocity in the package msg_interfaces.
typedef struct msg_interfaces__msg__ShoulderJointVelocity
{
  double angular_speed;
} msg_interfaces__msg__ShoulderJointVelocity;

// Struct for a sequence of msg_interfaces__msg__ShoulderJointVelocity.
typedef struct msg_interfaces__msg__ShoulderJointVelocity__Sequence
{
  msg_interfaces__msg__ShoulderJointVelocity * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_interfaces__msg__ShoulderJointVelocity__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_INTERFACES__MSG__DETAIL__SHOULDER_JOINT_VELOCITY__STRUCT_H_
