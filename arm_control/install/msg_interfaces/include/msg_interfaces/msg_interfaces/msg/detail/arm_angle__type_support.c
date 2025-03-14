// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from msg_interfaces:msg/ArmAngle.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "msg_interfaces/msg/detail/arm_angle__rosidl_typesupport_introspection_c.h"
#include "msg_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "msg_interfaces/msg/detail/arm_angle__functions.h"
#include "msg_interfaces/msg/detail/arm_angle__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void msg_interfaces__msg__ArmAngle__rosidl_typesupport_introspection_c__ArmAngle_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  msg_interfaces__msg__ArmAngle__init(message_memory);
}

void msg_interfaces__msg__ArmAngle__rosidl_typesupport_introspection_c__ArmAngle_fini_function(void * message_memory)
{
  msg_interfaces__msg__ArmAngle__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember msg_interfaces__msg__ArmAngle__rosidl_typesupport_introspection_c__ArmAngle_message_member_array[2] = {
  {
    "alpha",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces__msg__ArmAngle, alpha),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "beta",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces__msg__ArmAngle, beta),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers msg_interfaces__msg__ArmAngle__rosidl_typesupport_introspection_c__ArmAngle_message_members = {
  "msg_interfaces__msg",  // message namespace
  "ArmAngle",  // message name
  2,  // number of fields
  sizeof(msg_interfaces__msg__ArmAngle),
  msg_interfaces__msg__ArmAngle__rosidl_typesupport_introspection_c__ArmAngle_message_member_array,  // message members
  msg_interfaces__msg__ArmAngle__rosidl_typesupport_introspection_c__ArmAngle_init_function,  // function to initialize message memory (memory has to be allocated)
  msg_interfaces__msg__ArmAngle__rosidl_typesupport_introspection_c__ArmAngle_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t msg_interfaces__msg__ArmAngle__rosidl_typesupport_introspection_c__ArmAngle_message_type_support_handle = {
  0,
  &msg_interfaces__msg__ArmAngle__rosidl_typesupport_introspection_c__ArmAngle_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_msg_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, msg_interfaces, msg, ArmAngle)() {
  if (!msg_interfaces__msg__ArmAngle__rosidl_typesupport_introspection_c__ArmAngle_message_type_support_handle.typesupport_identifier) {
    msg_interfaces__msg__ArmAngle__rosidl_typesupport_introspection_c__ArmAngle_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &msg_interfaces__msg__ArmAngle__rosidl_typesupport_introspection_c__ArmAngle_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
