// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msg_interfaces:msg/EncoderArm.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__ENCODER_ARM__STRUCT_HPP_
#define MSG_INTERFACES__MSG__DETAIL__ENCODER_ARM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msg_interfaces__msg__EncoderArm __attribute__((deprecated))
#else
# define DEPRECATED__msg_interfaces__msg__EncoderArm __declspec(deprecated)
#endif

namespace msg_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EncoderArm_
{
  using Type = EncoderArm_<ContainerAllocator>;

  explicit EncoderArm_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit EncoderArm_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _arm_node0_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _arm_node0_type arm_node0;
  using _arm_node1_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _arm_node1_type arm_node1;
  using _arm_node2_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _arm_node2_type arm_node2;
  using _arm_node3_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _arm_node3_type arm_node3;
  using _arm_node4_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _arm_node4_type arm_node4;
  using _arm_node5_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _arm_node5_type arm_node5;

  // setters for named parameter idiom
  Type & set__arm_node0(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->arm_node0 = _arg;
    return *this;
  }
  Type & set__arm_node1(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->arm_node1 = _arg;
    return *this;
  }
  Type & set__arm_node2(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->arm_node2 = _arg;
    return *this;
  }
  Type & set__arm_node3(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->arm_node3 = _arg;
    return *this;
  }
  Type & set__arm_node4(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->arm_node4 = _arg;
    return *this;
  }
  Type & set__arm_node5(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->arm_node5 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msg_interfaces::msg::EncoderArm_<ContainerAllocator> *;
  using ConstRawPtr =
    const msg_interfaces::msg::EncoderArm_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msg_interfaces::msg::EncoderArm_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msg_interfaces::msg::EncoderArm_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::EncoderArm_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::EncoderArm_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msg_interfaces::msg::EncoderArm_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msg_interfaces::msg::EncoderArm_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msg_interfaces::msg::EncoderArm_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msg_interfaces::msg::EncoderArm_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msg_interfaces__msg__EncoderArm
    std::shared_ptr<msg_interfaces::msg::EncoderArm_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msg_interfaces__msg__EncoderArm
    std::shared_ptr<msg_interfaces::msg::EncoderArm_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EncoderArm_ & other) const
  {
    if (this->arm_node0 != other.arm_node0) {
      return false;
    }
    if (this->arm_node1 != other.arm_node1) {
      return false;
    }
    if (this->arm_node2 != other.arm_node2) {
      return false;
    }
    if (this->arm_node3 != other.arm_node3) {
      return false;
    }
    if (this->arm_node4 != other.arm_node4) {
      return false;
    }
    if (this->arm_node5 != other.arm_node5) {
      return false;
    }
    return true;
  }
  bool operator!=(const EncoderArm_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EncoderArm_

// alias to use template instance with default allocator
using EncoderArm =
  msg_interfaces::msg::EncoderArm_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msg_interfaces

#endif  // MSG_INTERFACES__MSG__DETAIL__ENCODER_ARM__STRUCT_HPP_
