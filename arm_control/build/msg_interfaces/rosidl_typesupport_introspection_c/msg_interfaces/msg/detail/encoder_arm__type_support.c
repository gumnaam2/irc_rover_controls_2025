// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from msg_interfaces:msg/EncoderArm.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "msg_interfaces/msg/detail/encoder_arm__rosidl_typesupport_introspection_c.h"
#include "msg_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "msg_interfaces/msg/detail/encoder_arm__functions.h"
#include "msg_interfaces/msg/detail/encoder_arm__struct.h"


// Include directives for member types
// Member `arm_node0`
// Member `arm_node1`
// Member `arm_node2`
// Member `arm_node3`
// Member `arm_node4`
// Member `arm_node5`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__EncoderArm_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  msg_interfaces__msg__EncoderArm__init(message_memory);
}

void msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__EncoderArm_fini_function(void * message_memory)
{
  msg_interfaces__msg__EncoderArm__fini(message_memory);
}

size_t msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__size_function__EncoderArm__arm_node0(
  const void * untyped_member)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return member->size;
}

const void * msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node0(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void * msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node0(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__fetch_function__EncoderArm__arm_node0(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int64_t * item =
    ((const int64_t *)
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node0(untyped_member, index));
  int64_t * value =
    (int64_t *)(untyped_value);
  *value = *item;
}

void msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__assign_function__EncoderArm__arm_node0(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int64_t * item =
    ((int64_t *)
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node0(untyped_member, index));
  const int64_t * value =
    (const int64_t *)(untyped_value);
  *item = *value;
}

bool msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__resize_function__EncoderArm__arm_node0(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  rosidl_runtime_c__int64__Sequence__fini(member);
  return rosidl_runtime_c__int64__Sequence__init(member, size);
}

size_t msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__size_function__EncoderArm__arm_node1(
  const void * untyped_member)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return member->size;
}

const void * msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node1(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void * msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node1(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__fetch_function__EncoderArm__arm_node1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int64_t * item =
    ((const int64_t *)
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node1(untyped_member, index));
  int64_t * value =
    (int64_t *)(untyped_value);
  *value = *item;
}

void msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__assign_function__EncoderArm__arm_node1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int64_t * item =
    ((int64_t *)
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node1(untyped_member, index));
  const int64_t * value =
    (const int64_t *)(untyped_value);
  *item = *value;
}

bool msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__resize_function__EncoderArm__arm_node1(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  rosidl_runtime_c__int64__Sequence__fini(member);
  return rosidl_runtime_c__int64__Sequence__init(member, size);
}

size_t msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__size_function__EncoderArm__arm_node2(
  const void * untyped_member)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return member->size;
}

const void * msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node2(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void * msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node2(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__fetch_function__EncoderArm__arm_node2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int64_t * item =
    ((const int64_t *)
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node2(untyped_member, index));
  int64_t * value =
    (int64_t *)(untyped_value);
  *value = *item;
}

void msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__assign_function__EncoderArm__arm_node2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int64_t * item =
    ((int64_t *)
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node2(untyped_member, index));
  const int64_t * value =
    (const int64_t *)(untyped_value);
  *item = *value;
}

bool msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__resize_function__EncoderArm__arm_node2(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  rosidl_runtime_c__int64__Sequence__fini(member);
  return rosidl_runtime_c__int64__Sequence__init(member, size);
}

size_t msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__size_function__EncoderArm__arm_node3(
  const void * untyped_member)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return member->size;
}

const void * msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node3(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void * msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node3(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__fetch_function__EncoderArm__arm_node3(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int64_t * item =
    ((const int64_t *)
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node3(untyped_member, index));
  int64_t * value =
    (int64_t *)(untyped_value);
  *value = *item;
}

void msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__assign_function__EncoderArm__arm_node3(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int64_t * item =
    ((int64_t *)
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node3(untyped_member, index));
  const int64_t * value =
    (const int64_t *)(untyped_value);
  *item = *value;
}

bool msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__resize_function__EncoderArm__arm_node3(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  rosidl_runtime_c__int64__Sequence__fini(member);
  return rosidl_runtime_c__int64__Sequence__init(member, size);
}

size_t msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__size_function__EncoderArm__arm_node4(
  const void * untyped_member)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return member->size;
}

const void * msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node4(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void * msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node4(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__fetch_function__EncoderArm__arm_node4(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int64_t * item =
    ((const int64_t *)
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node4(untyped_member, index));
  int64_t * value =
    (int64_t *)(untyped_value);
  *value = *item;
}

void msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__assign_function__EncoderArm__arm_node4(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int64_t * item =
    ((int64_t *)
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node4(untyped_member, index));
  const int64_t * value =
    (const int64_t *)(untyped_value);
  *item = *value;
}

bool msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__resize_function__EncoderArm__arm_node4(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  rosidl_runtime_c__int64__Sequence__fini(member);
  return rosidl_runtime_c__int64__Sequence__init(member, size);
}

size_t msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__size_function__EncoderArm__arm_node5(
  const void * untyped_member)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return member->size;
}

const void * msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node5(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void * msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node5(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__fetch_function__EncoderArm__arm_node5(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int64_t * item =
    ((const int64_t *)
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node5(untyped_member, index));
  int64_t * value =
    (int64_t *)(untyped_value);
  *value = *item;
}

void msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__assign_function__EncoderArm__arm_node5(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int64_t * item =
    ((int64_t *)
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node5(untyped_member, index));
  const int64_t * value =
    (const int64_t *)(untyped_value);
  *item = *value;
}

bool msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__resize_function__EncoderArm__arm_node5(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  rosidl_runtime_c__int64__Sequence__fini(member);
  return rosidl_runtime_c__int64__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__EncoderArm_message_member_array[6] = {
  {
    "arm_node0",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces__msg__EncoderArm, arm_node0),  // bytes offset in struct
    NULL,  // default value
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__size_function__EncoderArm__arm_node0,  // size() function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node0,  // get_const(index) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node0,  // get(index) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__fetch_function__EncoderArm__arm_node0,  // fetch(index, &value) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__assign_function__EncoderArm__arm_node0,  // assign(index, value) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__resize_function__EncoderArm__arm_node0  // resize(index) function pointer
  },
  {
    "arm_node1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces__msg__EncoderArm, arm_node1),  // bytes offset in struct
    NULL,  // default value
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__size_function__EncoderArm__arm_node1,  // size() function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node1,  // get_const(index) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node1,  // get(index) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__fetch_function__EncoderArm__arm_node1,  // fetch(index, &value) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__assign_function__EncoderArm__arm_node1,  // assign(index, value) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__resize_function__EncoderArm__arm_node1  // resize(index) function pointer
  },
  {
    "arm_node2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces__msg__EncoderArm, arm_node2),  // bytes offset in struct
    NULL,  // default value
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__size_function__EncoderArm__arm_node2,  // size() function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node2,  // get_const(index) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node2,  // get(index) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__fetch_function__EncoderArm__arm_node2,  // fetch(index, &value) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__assign_function__EncoderArm__arm_node2,  // assign(index, value) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__resize_function__EncoderArm__arm_node2  // resize(index) function pointer
  },
  {
    "arm_node3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces__msg__EncoderArm, arm_node3),  // bytes offset in struct
    NULL,  // default value
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__size_function__EncoderArm__arm_node3,  // size() function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node3,  // get_const(index) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node3,  // get(index) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__fetch_function__EncoderArm__arm_node3,  // fetch(index, &value) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__assign_function__EncoderArm__arm_node3,  // assign(index, value) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__resize_function__EncoderArm__arm_node3  // resize(index) function pointer
  },
  {
    "arm_node4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces__msg__EncoderArm, arm_node4),  // bytes offset in struct
    NULL,  // default value
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__size_function__EncoderArm__arm_node4,  // size() function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node4,  // get_const(index) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node4,  // get(index) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__fetch_function__EncoderArm__arm_node4,  // fetch(index, &value) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__assign_function__EncoderArm__arm_node4,  // assign(index, value) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__resize_function__EncoderArm__arm_node4  // resize(index) function pointer
  },
  {
    "arm_node5",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces__msg__EncoderArm, arm_node5),  // bytes offset in struct
    NULL,  // default value
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__size_function__EncoderArm__arm_node5,  // size() function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_const_function__EncoderArm__arm_node5,  // get_const(index) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__get_function__EncoderArm__arm_node5,  // get(index) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__fetch_function__EncoderArm__arm_node5,  // fetch(index, &value) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__assign_function__EncoderArm__arm_node5,  // assign(index, value) function pointer
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__resize_function__EncoderArm__arm_node5  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__EncoderArm_message_members = {
  "msg_interfaces__msg",  // message namespace
  "EncoderArm",  // message name
  6,  // number of fields
  sizeof(msg_interfaces__msg__EncoderArm),
  msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__EncoderArm_message_member_array,  // message members
  msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__EncoderArm_init_function,  // function to initialize message memory (memory has to be allocated)
  msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__EncoderArm_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__EncoderArm_message_type_support_handle = {
  0,
  &msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__EncoderArm_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_msg_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, msg_interfaces, msg, EncoderArm)() {
  if (!msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__EncoderArm_message_type_support_handle.typesupport_identifier) {
    msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__EncoderArm_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &msg_interfaces__msg__EncoderArm__rosidl_typesupport_introspection_c__EncoderArm_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
