// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_interfaces:msg/CamDrive.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__CAM_DRIVE__STRUCT_H_
#define MSG_INTERFACES__MSG__DETAIL__CAM_DRIVE__STRUCT_H_

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
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/CamDrive in the package msg_interfaces.
typedef struct msg_interfaces__msg__CamDrive
{
  double speed;
  rosidl_runtime_c__String direction;
} msg_interfaces__msg__CamDrive;

// Struct for a sequence of msg_interfaces__msg__CamDrive.
typedef struct msg_interfaces__msg__CamDrive__Sequence
{
  msg_interfaces__msg__CamDrive * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_interfaces__msg__CamDrive__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_INTERFACES__MSG__DETAIL__CAM_DRIVE__STRUCT_H_
