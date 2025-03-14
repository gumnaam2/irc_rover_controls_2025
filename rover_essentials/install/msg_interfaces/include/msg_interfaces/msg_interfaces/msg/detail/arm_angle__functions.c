// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from msg_interfaces:msg/ArmAngle.idl
// generated code does not contain a copyright notice
#include "msg_interfaces/msg/detail/arm_angle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
msg_interfaces__msg__ArmAngle__init(msg_interfaces__msg__ArmAngle * msg)
{
  if (!msg) {
    return false;
  }
  // alpha
  // beta
  return true;
}

void
msg_interfaces__msg__ArmAngle__fini(msg_interfaces__msg__ArmAngle * msg)
{
  if (!msg) {
    return;
  }
  // alpha
  // beta
}

bool
msg_interfaces__msg__ArmAngle__are_equal(const msg_interfaces__msg__ArmAngle * lhs, const msg_interfaces__msg__ArmAngle * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // alpha
  if (lhs->alpha != rhs->alpha) {
    return false;
  }
  // beta
  if (lhs->beta != rhs->beta) {
    return false;
  }
  return true;
}

bool
msg_interfaces__msg__ArmAngle__copy(
  const msg_interfaces__msg__ArmAngle * input,
  msg_interfaces__msg__ArmAngle * output)
{
  if (!input || !output) {
    return false;
  }
  // alpha
  output->alpha = input->alpha;
  // beta
  output->beta = input->beta;
  return true;
}

msg_interfaces__msg__ArmAngle *
msg_interfaces__msg__ArmAngle__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__ArmAngle * msg = (msg_interfaces__msg__ArmAngle *)allocator.allocate(sizeof(msg_interfaces__msg__ArmAngle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(msg_interfaces__msg__ArmAngle));
  bool success = msg_interfaces__msg__ArmAngle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
msg_interfaces__msg__ArmAngle__destroy(msg_interfaces__msg__ArmAngle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    msg_interfaces__msg__ArmAngle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
msg_interfaces__msg__ArmAngle__Sequence__init(msg_interfaces__msg__ArmAngle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__ArmAngle * data = NULL;

  if (size) {
    data = (msg_interfaces__msg__ArmAngle *)allocator.zero_allocate(size, sizeof(msg_interfaces__msg__ArmAngle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = msg_interfaces__msg__ArmAngle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        msg_interfaces__msg__ArmAngle__fini(&data[i - 1]);
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
msg_interfaces__msg__ArmAngle__Sequence__fini(msg_interfaces__msg__ArmAngle__Sequence * array)
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
      msg_interfaces__msg__ArmAngle__fini(&array->data[i]);
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

msg_interfaces__msg__ArmAngle__Sequence *
msg_interfaces__msg__ArmAngle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__ArmAngle__Sequence * array = (msg_interfaces__msg__ArmAngle__Sequence *)allocator.allocate(sizeof(msg_interfaces__msg__ArmAngle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = msg_interfaces__msg__ArmAngle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
msg_interfaces__msg__ArmAngle__Sequence__destroy(msg_interfaces__msg__ArmAngle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    msg_interfaces__msg__ArmAngle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
msg_interfaces__msg__ArmAngle__Sequence__are_equal(const msg_interfaces__msg__ArmAngle__Sequence * lhs, const msg_interfaces__msg__ArmAngle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!msg_interfaces__msg__ArmAngle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
msg_interfaces__msg__ArmAngle__Sequence__copy(
  const msg_interfaces__msg__ArmAngle__Sequence * input,
  msg_interfaces__msg__ArmAngle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(msg_interfaces__msg__ArmAngle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    msg_interfaces__msg__ArmAngle * data =
      (msg_interfaces__msg__ArmAngle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!msg_interfaces__msg__ArmAngle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          msg_interfaces__msg__ArmAngle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!msg_interfaces__msg__ArmAngle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
