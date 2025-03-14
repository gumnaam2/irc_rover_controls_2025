// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from msg_interfaces:msg/ArmAngle.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "msg_interfaces/msg/detail/arm_angle__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace msg_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ArmAngle_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) msg_interfaces::msg::ArmAngle(_init);
}

void ArmAngle_fini_function(void * message_memory)
{
  auto typed_message = static_cast<msg_interfaces::msg::ArmAngle *>(message_memory);
  typed_message->~ArmAngle();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ArmAngle_message_member_array[2] = {
  {
    "alpha",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces::msg::ArmAngle, alpha),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "beta",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces::msg::ArmAngle, beta),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ArmAngle_message_members = {
  "msg_interfaces::msg",  // message namespace
  "ArmAngle",  // message name
  2,  // number of fields
  sizeof(msg_interfaces::msg::ArmAngle),
  ArmAngle_message_member_array,  // message members
  ArmAngle_init_function,  // function to initialize message memory (memory has to be allocated)
  ArmAngle_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ArmAngle_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ArmAngle_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace msg_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<msg_interfaces::msg::ArmAngle>()
{
  return &::msg_interfaces::msg::rosidl_typesupport_introspection_cpp::ArmAngle_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, msg_interfaces, msg, ArmAngle)() {
  return &::msg_interfaces::msg::rosidl_typesupport_introspection_cpp::ArmAngle_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
