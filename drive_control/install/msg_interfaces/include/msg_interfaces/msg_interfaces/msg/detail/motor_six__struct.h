// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msg_interfaces:msg/MotorSix.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__MOTOR_SIX__STRUCT_H_
#define MSG_INTERFACES__MSG__DETAIL__MOTOR_SIX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'motor1_direction'
// Member 'motor2_direction'
// Member 'motor3_direction'
// Member 'motor4_direction'
// Member 'motor5_direction'
// Member 'motor6_direction'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MotorSix in the package msg_interfaces.
typedef struct msg_interfaces__msg__MotorSix
{
  double motor1_speed;
  rosidl_runtime_c__String motor1_direction;
  double motor2_speed;
  rosidl_runtime_c__String motor2_direction;
  double motor3_speed;
  rosidl_runtime_c__String motor3_direction;
  double motor4_speed;
  rosidl_runtime_c__String motor4_direction;
  double motor5_speed;
  rosidl_runtime_c__String motor5_direction;
  double motor6_speed;
  rosidl_runtime_c__String motor6_direction;
} msg_interfaces__msg__MotorSix;

// Struct for a sequence of msg_interfaces__msg__MotorSix.
typedef struct msg_interfaces__msg__MotorSix__Sequence
{
  msg_interfaces__msg__MotorSix * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msg_interfaces__msg__MotorSix__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSG_INTERFACES__MSG__DETAIL__MOTOR_SIX__STRUCT_H_
