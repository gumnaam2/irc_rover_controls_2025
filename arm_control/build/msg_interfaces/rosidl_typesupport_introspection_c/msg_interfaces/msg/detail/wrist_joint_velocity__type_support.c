// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from msg_interfaces:msg/WristJointVelocity.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "msg_interfaces/msg/detail/wrist_joint_velocity__rosidl_typesupport_introspection_c.h"
#include "msg_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "msg_interfaces/msg/detail/wrist_joint_velocity__functions.h"
#include "msg_interfaces/msg/detail/wrist_joint_velocity__struct.h"


// Include directives for member types
// Member `angular_speed`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__WristJointVelocity_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  msg_interfaces__msg__WristJointVelocity__init(message_memory);
}

void msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__WristJointVelocity_fini_function(void * message_memory)
{
  msg_interfaces__msg__WristJointVelocity__fini(message_memory);
}

size_t msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__size_function__WristJointVelocity__angular_speed(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__get_const_function__WristJointVelocity__angular_speed(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__get_function__WristJointVelocity__angular_speed(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__fetch_function__WristJointVelocity__angular_speed(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__get_const_function__WristJointVelocity__angular_speed(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__assign_function__WristJointVelocity__angular_speed(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__get_function__WristJointVelocity__angular_speed(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__resize_function__WristJointVelocity__angular_speed(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__WristJointVelocity_message_member_array[1] = {
  {
    "angular_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces__msg__WristJointVelocity, angular_speed),  // bytes offset in struct
    NULL,  // default value
    msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__size_function__WristJointVelocity__angular_speed,  // size() function pointer
    msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__get_const_function__WristJointVelocity__angular_speed,  // get_const(index) function pointer
    msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__get_function__WristJointVelocity__angular_speed,  // get(index) function pointer
    msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__fetch_function__WristJointVelocity__angular_speed,  // fetch(index, &value) function pointer
    msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__assign_function__WristJointVelocity__angular_speed,  // assign(index, value) function pointer
    msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__resize_function__WristJointVelocity__angular_speed  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__WristJointVelocity_message_members = {
  "msg_interfaces__msg",  // message namespace
  "WristJointVelocity",  // message name
  1,  // number of fields
  sizeof(msg_interfaces__msg__WristJointVelocity),
  msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__WristJointVelocity_message_member_array,  // message members
  msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__WristJointVelocity_init_function,  // function to initialize message memory (memory has to be allocated)
  msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__WristJointVelocity_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__WristJointVelocity_message_type_support_handle = {
  0,
  &msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__WristJointVelocity_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_msg_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, msg_interfaces, msg, WristJointVelocity)() {
  if (!msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__WristJointVelocity_message_type_support_handle.typesupport_identifier) {
    msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__WristJointVelocity_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &msg_interfaces__msg__WristJointVelocity__rosidl_typesupport_introspection_c__WristJointVelocity_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
