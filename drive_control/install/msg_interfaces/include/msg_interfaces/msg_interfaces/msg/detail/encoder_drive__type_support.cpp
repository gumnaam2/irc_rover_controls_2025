// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from msg_interfaces:msg/EncoderDrive.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "msg_interfaces/msg/detail/encoder_drive__struct.hpp"
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

void EncoderDrive_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) msg_interfaces::msg::EncoderDrive(_init);
}

void EncoderDrive_fini_function(void * message_memory)
{
  auto typed_message = static_cast<msg_interfaces::msg::EncoderDrive *>(message_memory);
  typed_message->~EncoderDrive();
}

size_t size_function__EncoderDrive__drive_node0(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__EncoderDrive__drive_node0(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void * get_function__EncoderDrive__drive_node0(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__EncoderDrive__drive_node0(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int64_t *>(
    get_const_function__EncoderDrive__drive_node0(untyped_member, index));
  auto & value = *reinterpret_cast<int64_t *>(untyped_value);
  value = item;
}

void assign_function__EncoderDrive__drive_node0(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int64_t *>(
    get_function__EncoderDrive__drive_node0(untyped_member, index));
  const auto & value = *reinterpret_cast<const int64_t *>(untyped_value);
  item = value;
}

void resize_function__EncoderDrive__drive_node0(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__EncoderDrive__drive_node1(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__EncoderDrive__drive_node1(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void * get_function__EncoderDrive__drive_node1(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__EncoderDrive__drive_node1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int64_t *>(
    get_const_function__EncoderDrive__drive_node1(untyped_member, index));
  auto & value = *reinterpret_cast<int64_t *>(untyped_value);
  value = item;
}

void assign_function__EncoderDrive__drive_node1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int64_t *>(
    get_function__EncoderDrive__drive_node1(untyped_member, index));
  const auto & value = *reinterpret_cast<const int64_t *>(untyped_value);
  item = value;
}

void resize_function__EncoderDrive__drive_node1(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__EncoderDrive__drive_node2(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__EncoderDrive__drive_node2(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void * get_function__EncoderDrive__drive_node2(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__EncoderDrive__drive_node2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int64_t *>(
    get_const_function__EncoderDrive__drive_node2(untyped_member, index));
  auto & value = *reinterpret_cast<int64_t *>(untyped_value);
  value = item;
}

void assign_function__EncoderDrive__drive_node2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int64_t *>(
    get_function__EncoderDrive__drive_node2(untyped_member, index));
  const auto & value = *reinterpret_cast<const int64_t *>(untyped_value);
  item = value;
}

void resize_function__EncoderDrive__drive_node2(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__EncoderDrive__drive_node3(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__EncoderDrive__drive_node3(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void * get_function__EncoderDrive__drive_node3(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__EncoderDrive__drive_node3(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int64_t *>(
    get_const_function__EncoderDrive__drive_node3(untyped_member, index));
  auto & value = *reinterpret_cast<int64_t *>(untyped_value);
  value = item;
}

void assign_function__EncoderDrive__drive_node3(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int64_t *>(
    get_function__EncoderDrive__drive_node3(untyped_member, index));
  const auto & value = *reinterpret_cast<const int64_t *>(untyped_value);
  item = value;
}

void resize_function__EncoderDrive__drive_node3(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__EncoderDrive__drive_node4(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__EncoderDrive__drive_node4(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void * get_function__EncoderDrive__drive_node4(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__EncoderDrive__drive_node4(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int64_t *>(
    get_const_function__EncoderDrive__drive_node4(untyped_member, index));
  auto & value = *reinterpret_cast<int64_t *>(untyped_value);
  value = item;
}

void assign_function__EncoderDrive__drive_node4(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int64_t *>(
    get_function__EncoderDrive__drive_node4(untyped_member, index));
  const auto & value = *reinterpret_cast<const int64_t *>(untyped_value);
  item = value;
}

void resize_function__EncoderDrive__drive_node4(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__EncoderDrive__drive_node5(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__EncoderDrive__drive_node5(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void * get_function__EncoderDrive__drive_node5(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__EncoderDrive__drive_node5(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int64_t *>(
    get_const_function__EncoderDrive__drive_node5(untyped_member, index));
  auto & value = *reinterpret_cast<int64_t *>(untyped_value);
  value = item;
}

void assign_function__EncoderDrive__drive_node5(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int64_t *>(
    get_function__EncoderDrive__drive_node5(untyped_member, index));
  const auto & value = *reinterpret_cast<const int64_t *>(untyped_value);
  item = value;
}

void resize_function__EncoderDrive__drive_node5(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember EncoderDrive_message_member_array[6] = {
  {
    "drive_node0",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces::msg::EncoderDrive, drive_node0),  // bytes offset in struct
    nullptr,  // default value
    size_function__EncoderDrive__drive_node0,  // size() function pointer
    get_const_function__EncoderDrive__drive_node0,  // get_const(index) function pointer
    get_function__EncoderDrive__drive_node0,  // get(index) function pointer
    fetch_function__EncoderDrive__drive_node0,  // fetch(index, &value) function pointer
    assign_function__EncoderDrive__drive_node0,  // assign(index, value) function pointer
    resize_function__EncoderDrive__drive_node0  // resize(index) function pointer
  },
  {
    "drive_node1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces::msg::EncoderDrive, drive_node1),  // bytes offset in struct
    nullptr,  // default value
    size_function__EncoderDrive__drive_node1,  // size() function pointer
    get_const_function__EncoderDrive__drive_node1,  // get_const(index) function pointer
    get_function__EncoderDrive__drive_node1,  // get(index) function pointer
    fetch_function__EncoderDrive__drive_node1,  // fetch(index, &value) function pointer
    assign_function__EncoderDrive__drive_node1,  // assign(index, value) function pointer
    resize_function__EncoderDrive__drive_node1  // resize(index) function pointer
  },
  {
    "drive_node2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces::msg::EncoderDrive, drive_node2),  // bytes offset in struct
    nullptr,  // default value
    size_function__EncoderDrive__drive_node2,  // size() function pointer
    get_const_function__EncoderDrive__drive_node2,  // get_const(index) function pointer
    get_function__EncoderDrive__drive_node2,  // get(index) function pointer
    fetch_function__EncoderDrive__drive_node2,  // fetch(index, &value) function pointer
    assign_function__EncoderDrive__drive_node2,  // assign(index, value) function pointer
    resize_function__EncoderDrive__drive_node2  // resize(index) function pointer
  },
  {
    "drive_node3",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces::msg::EncoderDrive, drive_node3),  // bytes offset in struct
    nullptr,  // default value
    size_function__EncoderDrive__drive_node3,  // size() function pointer
    get_const_function__EncoderDrive__drive_node3,  // get_const(index) function pointer
    get_function__EncoderDrive__drive_node3,  // get(index) function pointer
    fetch_function__EncoderDrive__drive_node3,  // fetch(index, &value) function pointer
    assign_function__EncoderDrive__drive_node3,  // assign(index, value) function pointer
    resize_function__EncoderDrive__drive_node3  // resize(index) function pointer
  },
  {
    "drive_node4",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces::msg::EncoderDrive, drive_node4),  // bytes offset in struct
    nullptr,  // default value
    size_function__EncoderDrive__drive_node4,  // size() function pointer
    get_const_function__EncoderDrive__drive_node4,  // get_const(index) function pointer
    get_function__EncoderDrive__drive_node4,  // get(index) function pointer
    fetch_function__EncoderDrive__drive_node4,  // fetch(index, &value) function pointer
    assign_function__EncoderDrive__drive_node4,  // assign(index, value) function pointer
    resize_function__EncoderDrive__drive_node4  // resize(index) function pointer
  },
  {
    "drive_node5",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msg_interfaces::msg::EncoderDrive, drive_node5),  // bytes offset in struct
    nullptr,  // default value
    size_function__EncoderDrive__drive_node5,  // size() function pointer
    get_const_function__EncoderDrive__drive_node5,  // get_const(index) function pointer
    get_function__EncoderDrive__drive_node5,  // get(index) function pointer
    fetch_function__EncoderDrive__drive_node5,  // fetch(index, &value) function pointer
    assign_function__EncoderDrive__drive_node5,  // assign(index, value) function pointer
    resize_function__EncoderDrive__drive_node5  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers EncoderDrive_message_members = {
  "msg_interfaces::msg",  // message namespace
  "EncoderDrive",  // message name
  6,  // number of fields
  sizeof(msg_interfaces::msg::EncoderDrive),
  EncoderDrive_message_member_array,  // message members
  EncoderDrive_init_function,  // function to initialize message memory (memory has to be allocated)
  EncoderDrive_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t EncoderDrive_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &EncoderDrive_message_members,
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
get_message_type_support_handle<msg_interfaces::msg::EncoderDrive>()
{
  return &::msg_interfaces::msg::rosidl_typesupport_introspection_cpp::EncoderDrive_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, msg_interfaces, msg, EncoderDrive)() {
  return &::msg_interfaces::msg::rosidl_typesupport_introspection_cpp::EncoderDrive_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
