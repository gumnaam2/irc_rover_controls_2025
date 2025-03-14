// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from msg_interfaces:msg/EncoderDrive.idl
// generated code does not contain a copyright notice
#include "msg_interfaces/msg/detail/encoder_drive__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `drive_node0`
// Member `drive_node1`
// Member `drive_node2`
// Member `drive_node3`
// Member `drive_node4`
// Member `drive_node5`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
msg_interfaces__msg__EncoderDrive__init(msg_interfaces__msg__EncoderDrive * msg)
{
  if (!msg) {
    return false;
  }
  // drive_node0
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->drive_node0, 0)) {
    msg_interfaces__msg__EncoderDrive__fini(msg);
    return false;
  }
  // drive_node1
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->drive_node1, 0)) {
    msg_interfaces__msg__EncoderDrive__fini(msg);
    return false;
  }
  // drive_node2
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->drive_node2, 0)) {
    msg_interfaces__msg__EncoderDrive__fini(msg);
    return false;
  }
  // drive_node3
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->drive_node3, 0)) {
    msg_interfaces__msg__EncoderDrive__fini(msg);
    return false;
  }
  // drive_node4
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->drive_node4, 0)) {
    msg_interfaces__msg__EncoderDrive__fini(msg);
    return false;
  }
  // drive_node5
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->drive_node5, 0)) {
    msg_interfaces__msg__EncoderDrive__fini(msg);
    return false;
  }
  return true;
}

void
msg_interfaces__msg__EncoderDrive__fini(msg_interfaces__msg__EncoderDrive * msg)
{
  if (!msg) {
    return;
  }
  // drive_node0
  rosidl_runtime_c__int64__Sequence__fini(&msg->drive_node0);
  // drive_node1
  rosidl_runtime_c__int64__Sequence__fini(&msg->drive_node1);
  // drive_node2
  rosidl_runtime_c__int64__Sequence__fini(&msg->drive_node2);
  // drive_node3
  rosidl_runtime_c__int64__Sequence__fini(&msg->drive_node3);
  // drive_node4
  rosidl_runtime_c__int64__Sequence__fini(&msg->drive_node4);
  // drive_node5
  rosidl_runtime_c__int64__Sequence__fini(&msg->drive_node5);
}

bool
msg_interfaces__msg__EncoderDrive__are_equal(const msg_interfaces__msg__EncoderDrive * lhs, const msg_interfaces__msg__EncoderDrive * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // drive_node0
  if (!rosidl_runtime_c__int64__Sequence__are_equal(
      &(lhs->drive_node0), &(rhs->drive_node0)))
  {
    return false;
  }
  // drive_node1
  if (!rosidl_runtime_c__int64__Sequence__are_equal(
      &(lhs->drive_node1), &(rhs->drive_node1)))
  {
    return false;
  }
  // drive_node2
  if (!rosidl_runtime_c__int64__Sequence__are_equal(
      &(lhs->drive_node2), &(rhs->drive_node2)))
  {
    return false;
  }
  // drive_node3
  if (!rosidl_runtime_c__int64__Sequence__are_equal(
      &(lhs->drive_node3), &(rhs->drive_node3)))
  {
    return false;
  }
  // drive_node4
  if (!rosidl_runtime_c__int64__Sequence__are_equal(
      &(lhs->drive_node4), &(rhs->drive_node4)))
  {
    return false;
  }
  // drive_node5
  if (!rosidl_runtime_c__int64__Sequence__are_equal(
      &(lhs->drive_node5), &(rhs->drive_node5)))
  {
    return false;
  }
  return true;
}

bool
msg_interfaces__msg__EncoderDrive__copy(
  const msg_interfaces__msg__EncoderDrive * input,
  msg_interfaces__msg__EncoderDrive * output)
{
  if (!input || !output) {
    return false;
  }
  // drive_node0
  if (!rosidl_runtime_c__int64__Sequence__copy(
      &(input->drive_node0), &(output->drive_node0)))
  {
    return false;
  }
  // drive_node1
  if (!rosidl_runtime_c__int64__Sequence__copy(
      &(input->drive_node1), &(output->drive_node1)))
  {
    return false;
  }
  // drive_node2
  if (!rosidl_runtime_c__int64__Sequence__copy(
      &(input->drive_node2), &(output->drive_node2)))
  {
    return false;
  }
  // drive_node3
  if (!rosidl_runtime_c__int64__Sequence__copy(
      &(input->drive_node3), &(output->drive_node3)))
  {
    return false;
  }
  // drive_node4
  if (!rosidl_runtime_c__int64__Sequence__copy(
      &(input->drive_node4), &(output->drive_node4)))
  {
    return false;
  }
  // drive_node5
  if (!rosidl_runtime_c__int64__Sequence__copy(
      &(input->drive_node5), &(output->drive_node5)))
  {
    return false;
  }
  return true;
}

msg_interfaces__msg__EncoderDrive *
msg_interfaces__msg__EncoderDrive__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__EncoderDrive * msg = (msg_interfaces__msg__EncoderDrive *)allocator.allocate(sizeof(msg_interfaces__msg__EncoderDrive), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(msg_interfaces__msg__EncoderDrive));
  bool success = msg_interfaces__msg__EncoderDrive__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
msg_interfaces__msg__EncoderDrive__destroy(msg_interfaces__msg__EncoderDrive * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    msg_interfaces__msg__EncoderDrive__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
msg_interfaces__msg__EncoderDrive__Sequence__init(msg_interfaces__msg__EncoderDrive__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__EncoderDrive * data = NULL;

  if (size) {
    data = (msg_interfaces__msg__EncoderDrive *)allocator.zero_allocate(size, sizeof(msg_interfaces__msg__EncoderDrive), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = msg_interfaces__msg__EncoderDrive__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        msg_interfaces__msg__EncoderDrive__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
msg_interfaces__msg__EncoderDrive__Sequence__fini(msg_interfaces__msg__EncoderDrive__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      msg_interfaces__msg__EncoderDrive__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

msg_interfaces__msg__EncoderDrive__Sequence *
msg_interfaces__msg__EncoderDrive__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__EncoderDrive__Sequence * array = (msg_interfaces__msg__EncoderDrive__Sequence *)allocator.allocate(sizeof(msg_interfaces__msg__EncoderDrive__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = msg_interfaces__msg__EncoderDrive__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
msg_interfaces__msg__EncoderDrive__Sequence__destroy(msg_interfaces__msg__EncoderDrive__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    msg_interfaces__msg__EncoderDrive__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
msg_interfaces__msg__EncoderDrive__Sequence__are_equal(const msg_interfaces__msg__EncoderDrive__Sequence * lhs, const msg_interfaces__msg__EncoderDrive__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!msg_interfaces__msg__EncoderDrive__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
msg_interfaces__msg__EncoderDrive__Sequence__copy(
  const msg_interfaces__msg__EncoderDrive__Sequence * input,
  msg_interfaces__msg__EncoderDrive__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(msg_interfaces__msg__EncoderDrive);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    msg_interfaces__msg__EncoderDrive * data =
      (msg_interfaces__msg__EncoderDrive *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!msg_interfaces__msg__EncoderDrive__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          msg_interfaces__msg__EncoderDrive__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!msg_interfaces__msg__EncoderDrive__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
