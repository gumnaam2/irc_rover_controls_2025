// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_interfaces:msg/EncoderDrive.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ENCODER_DRIVE__STRUCT_H_
#define MSG_INTERFACES__MSG__DETAIL__ENCODER_DRIVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'drive_node0'
// Member 'drive_node1'
// Member 'drive_node2'
// Member 'drive_node3'
// Member 'drive_node4'
// Member 'drive_node5'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/EncoderDrive in the package msg_interfaces.
typedef struct msg_interfaces__msg__EncoderDrive
{
  rosidl_runtime_c__int64__Sequence drive_node0;
  rosidl_runtime_c__int64__Sequence drive_node1;
  rosidl_runtime_c__int64__Sequence drive_node2;
  rosidl_runtime_c__int64__Sequence drive_node3;
  rosidl_runtime_c__int64__Sequence drive_node4;
  rosidl_runtime_c__int64__Sequence drive_node5;
} msg_interfaces__msg__EncoderDrive;

// Struct for a sequence of msg_interfaces__msg__EncoderDrive.
typedef struct msg_interfaces__msg__EncoderDrive__Sequence
{
  msg_interfaces__msg__EncoderDrive * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_interfaces__msg__EncoderDrive__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_INTERFACES__MSG__DETAIL__ENCODER_DRIVE__STRUCT_H_
