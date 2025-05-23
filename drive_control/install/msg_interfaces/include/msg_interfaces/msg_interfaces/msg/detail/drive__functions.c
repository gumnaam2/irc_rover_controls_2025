// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from msg_interfaces:msg/Drive.idl
// generated code does not contain a copyright notice
#include "msg_interfaces/msg/detail/drive__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `direction`
// Member `speed`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
msg_interfaces__msg__Drive__init(msg_interfaces__msg__Drive * msg)
{
  if (!msg) {
    return false;
  }
  // direction
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->direction, 0)) {
    msg_interfaces__msg__Drive__fini(msg);
    return false;
  }
  // speed
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->speed, 0)) {
    msg_interfaces__msg__Drive__fini(msg);
    return false;
  }
  // sys_check
  return true;
}

void
msg_interfaces__msg__Drive__fini(msg_interfaces__msg__Drive * msg)
{
  if (!msg) {
    return;
  }
  // direction
  rosidl_runtime_c__int64__Sequence__fini(&msg->direction);
  // speed
  rosidl_runtime_c__int64__Sequence__fini(&msg->speed);
  // sys_check
}

bool
msg_interfaces__msg__Drive__are_equal(const msg_interfaces__msg__Drive * lhs, const msg_interfaces__msg__Drive * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // direction
  if (!rosidl_runtime_c__int64__Sequence__are_equal(
      &(lhs->direction), &(rhs->direction)))
  {
    return false;
  }
  // speed
  if (!rosidl_runtime_c__int64__Sequence__are_equal(
      &(lhs->speed), &(rhs->speed)))
  {
    return false;
  }
  // sys_check
  if (lhs->sys_check != rhs->sys_check) {
    return false;
  }
  return true;
}

bool
msg_interfaces__msg__Drive__copy(
  const msg_interfaces__msg__Drive * input,
  msg_interfaces__msg__Drive * output)
{
  if (!input || !output) {
    return false;
  }
  // direction
  if (!rosidl_runtime_c__int64__Sequence__copy(
      &(input->direction), &(output->direction)))
  {
    return false;
  }
  // speed
  if (!rosidl_runtime_c__int64__Sequence__copy(
      &(input->speed), &(output->speed)))
  {
    return false;
  }
  // sys_check
  output->sys_check = input->sys_check;
  return true;
}

msg_interfaces__msg__Drive *
msg_interfaces__msg__Drive__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__Drive * msg = (msg_interfaces__msg__Drive *)allocator.allocate(sizeof(msg_interfaces__msg__Drive), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(msg_interfaces__msg__Drive));
  bool success = msg_interfaces__msg__Drive__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
msg_interfaces__msg__Drive__destroy(msg_interfaces__msg__Drive * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    msg_interfaces__msg__Drive__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
msg_interfaces__msg__Drive__Sequence__init(msg_interfaces__msg__Drive__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__Drive * data = NULL;

  if (size) {
    data = (msg_interfaces__msg__Drive *)allocator.zero_allocate(size, sizeof(msg_interfaces__msg__Drive), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = msg_interfaces__msg__Drive__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        msg_interfaces__msg__Drive__fini(&data[i - 1]);
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
msg_interfaces__msg__Drive__Sequence__fini(msg_interfaces__msg__Drive__Sequence * array)
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
      msg_interfaces__msg__Drive__fini(&array->data[i]);
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

msg_interfaces__msg__Drive__Sequence *
msg_interfaces__msg__Drive__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__Drive__Sequence * array = (msg_interfaces__msg__Drive__Sequence *)allocator.allocate(sizeof(msg_interfaces__msg__Drive__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = msg_interfaces__msg__Drive__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
msg_interfaces__msg__Drive__Sequence__destroy(msg_interfaces__msg__Drive__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    msg_interfaces__msg__Drive__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
msg_interfaces__msg__Drive__Sequence__are_equal(const msg_interfaces__msg__Drive__Sequence * lhs, const msg_interfaces__msg__Drive__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!msg_interfaces__msg__Drive__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
msg_interfaces__msg__Drive__Sequence__copy(
  const msg_interfaces__msg__Drive__Sequence * input,
  msg_interfaces__msg__Drive__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(msg_interfaces__msg__Drive);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    msg_interfaces__msg__Drive * data =
      (msg_interfaces__msg__Drive *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!msg_interfaces__msg__Drive__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          msg_interfaces__msg__Drive__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!msg_interfaces__msg__Drive__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
