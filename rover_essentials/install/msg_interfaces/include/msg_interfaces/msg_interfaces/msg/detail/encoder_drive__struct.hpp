// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_interfaces:msg/EncoderDrive.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ENCODER_DRIVE__STRUCT_HPP_
#define MSG_INTERFACES__MSG__DETAIL__ENCODER_DRIVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msg_interfaces__msg__EncoderDrive __attribute__((deprecated))
#else
# define DEPRECATED__msg_interfaces__msg__EncoderDrive __declspec(deprecated)
#endif

namespace msg_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EncoderDrive_
{
  using Type = EncoderDrive_<ContainerAllocator>;

  explicit EncoderDrive_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit EncoderDrive_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _drive_node0_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _drive_node0_type drive_node0;
  using _drive_node1_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _drive_node1_type drive_node1;
  using _drive_node2_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _drive_node2_type drive_node2;
  using _drive_node3_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _drive_node3_type drive_node3;
  using _drive_node4_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _drive_node4_type drive_node4;
  using _drive_node5_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _drive_node5_type drive_node5;

  // setters for named parameter idiom
  Type & set__drive_node0(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->drive_node0 = _arg;
    return *this;
  }
  Type & set__drive_node1(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->drive_node1 = _arg;
    return *this;
  }
  Type & set__drive_node2(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->drive_node2 = _arg;
    return *this;
  }
  Type & set__drive_node3(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->drive_node3 = _arg;
    return *this;
  }
  Type & set__drive_node4(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->drive_node4 = _arg;
    return *this;
  }
  Type & set__drive_node5(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->drive_node5 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_interfaces::msg::EncoderDrive_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_interfaces::msg::EncoderDrive_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_interfaces::msg::EncoderDrive_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_interfaces::msg::EncoderDrive_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::EncoderDrive_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::EncoderDrive_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::EncoderDrive_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::EncoderDrive_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_interfaces::msg::EncoderDrive_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_interfaces::msg::EncoderDrive_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_interfaces__msg__EncoderDrive
    std::shared_ptr<msg_interfaces::msg::EncoderDrive_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_interfaces__msg__EncoderDrive
    std::shared_ptr<msg_interfaces::msg::EncoderDrive_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EncoderDrive_ & other) const
  {
    if (this->drive_node0 != other.drive_node0) {
      return false;
    }
    if (this->drive_node1 != other.drive_node1) {
      return false;
    }
    if (this->drive_node2 != other.drive_node2) {
      return false;
    }
    if (this->drive_node3 != other.drive_node3) {
      return false;
    }
    if (this->drive_node4 != other.drive_node4) {
      return false;
    }
    if (this->drive_node5 != other.drive_node5) {
      return false;
    }
    return true;
  }
  bool operator!=(const EncoderDrive_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EncoderDrive_

// alias to use template instance with default allocator
using EncoderDrive =
  msg_interfaces::msg::EncoderDrive_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__ENCODER_DRIVE__STRUCT_HPP_
