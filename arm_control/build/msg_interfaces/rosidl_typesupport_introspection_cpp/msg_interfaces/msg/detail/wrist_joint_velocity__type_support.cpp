// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from msg_interfaces:msg/WristJointVelocity.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "msg_interfaces/msg/detail/wrist_joint_velocity__struct.hpp"
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

void WristJointVelocity_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) msg_interfaces::msg::WristJointVelocity(_init);
}

void WristJointVelocity_fini_function(void * message_memory)
{
  auto typed_message = static_cast<msg_interfaces::msg::WristJointVelocity *>(message_memory);
  typed_message->~WristJointVelocity();
}

size_t size_function__WristJointVelocity__angular_speed(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__WristJointVelocity__angular_speed(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__WristJointVelocity__angular_speed(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__WristJointVelocity__angular_speed(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__WristJointVelocity__angular_speed(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__WristJointVelocity__angular_speed(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__WristJointVelocity__angular_speed(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__WristJointVelocity__angular_speed(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember WristJointVelocity_message_member_array[1] = {
  {
    "angular_speed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces::msg::WristJointVelocity, angular_speed),  // bytes offset in struct
    nullptr,  // default value
    size_function__WristJointVelocity__angular_speed,  // size() function pointer
    get_const_function__WristJointVelocity__angular_speed,  // get_const(index) function pointer
    get_function__WristJointVelocity__angular_speed,  // get(index) function pointer
    fetch_function__WristJointVelocity__angular_speed,  // fetch(index, &value) function pointer
    assign_function__WristJointVelocity__angular_speed,  // assign(index, value) function pointer
    resize_function__WristJointVelocity__angular_speed  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers WristJointVelocity_message_members = {
  "msg_interfaces::msg",  // message namespace
  "WristJointVelocity",  // message name
  1,  // number of fields
  sizeof(msg_interfaces::msg::WristJointVelocity),
  WristJointVelocity_message_member_array,  // message members
  WristJointVelocity_init_function,  // function to initialize message memory (memory has to be allocated)
  WristJointVelocity_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t WristJointVelocity_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &WristJointVelocity_message_members,
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
get_message_type_support_handle<msg_interfaces::msg::WristJointVelocity>()
{
  return &::msg_interfaces::msg::rosidl_typesupport_introspection_cpp::WristJointVelocity_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, msg_interfaces, msg, WristJointVelocity)() {
  return &::msg_interfaces::msg::rosidl_typesupport_introspection_cpp::WristJointVelocity_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
