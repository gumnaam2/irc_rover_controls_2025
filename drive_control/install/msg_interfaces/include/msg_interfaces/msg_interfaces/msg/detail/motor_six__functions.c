// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from msg_interfaces:msg/MotorSix.idl
// generated code does not contain a copyright notice
#include "msg_interfaces/msg/detail/motor_six__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `motor1_direction`
// Member `motor2_direction`
// Member `motor3_direction`
// Member `motor4_direction`
// Member `motor5_direction`
// Member `motor6_direction`
#include "rosidl_runtime_c/string_functions.h"

bool
msg_interfaces__msg__MotorSix__init(msg_interfaces__msg__MotorSix * msg)
{
  if (!msg) {
    return false;
  }
  // motor1_speed
  // motor1_direction
  if (!rosidl_runtime_c__String__init(&msg->motor1_direction)) {
    msg_interfaces__msg__MotorSix__fini(msg);
    return false;
  }
  // motor2_speed
  // motor2_direction
  if (!rosidl_runtime_c__String__init(&msg->motor2_direction)) {
    msg_interfaces__msg__MotorSix__fini(msg);
    return false;
  }
  // motor3_speed
  // motor3_direction
  if (!rosidl_runtime_c__String__init(&msg->motor3_direction)) {
    msg_interfaces__msg__MotorSix__fini(msg);
    return false;
  }
  // motor4_speed
  // motor4_direction
  if (!rosidl_runtime_c__String__init(&msg->motor4_direction)) {
    msg_interfaces__msg__MotorSix__fini(msg);
    return false;
  }
  // motor5_speed
  // motor5_direction
  if (!rosidl_runtime_c__String__init(&msg->motor5_direction)) {
    msg_interfaces__msg__MotorSix__fini(msg);
    return false;
  }
  // motor6_speed
  // motor6_direction
  if (!rosidl_runtime_c__String__init(&msg->motor6_direction)) {
    msg_interfaces__msg__MotorSix__fini(msg);
    return false;
  }
  return true;
}

void
msg_interfaces__msg__MotorSix__fini(msg_interfaces__msg__MotorSix * msg)
{
  if (!msg) {
    return;
  }
  // motor1_speed
  // motor1_direction
  rosidl_runtime_c__String__fini(&msg->motor1_direction);
  // motor2_speed
  // motor2_direction
  rosidl_runtime_c__String__fini(&msg->motor2_direction);
  // motor3_speed
  // motor3_direction
  rosidl_runtime_c__String__fini(&msg->motor3_direction);
  // motor4_speed
  // motor4_direction
  rosidl_runtime_c__String__fini(&msg->motor4_direction);
  // motor5_speed
  // motor5_direction
  rosidl_runtime_c__String__fini(&msg->motor5_direction);
  // motor6_speed
  // motor6_direction
  rosidl_runtime_c__String__fini(&msg->motor6_direction);
}

bool
msg_interfaces__msg__MotorSix__are_equal(const msg_interfaces__msg__MotorSix * lhs, const msg_interfaces__msg__MotorSix * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // motor1_speed
  if (lhs->motor1_speed != rhs->motor1_speed) {
    return false;
  }
  // motor1_direction
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->motor1_direction), &(rhs->motor1_direction)))
  {
    return false;
  }
  // motor2_speed
  if (lhs->motor2_speed != rhs->motor2_speed) {
    return false;
  }
  // motor2_direction
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->motor2_direction), &(rhs->motor2_direction)))
  {
    return false;
  }
  // motor3_speed
  if (lhs->motor3_speed != rhs->motor3_speed) {
    return false;
  }
  // motor3_direction
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->motor3_direction), &(rhs->motor3_direction)))
  {
    return false;
  }
  // motor4_speed
  if (lhs->motor4_speed != rhs->motor4_speed) {
    return false;
  }
  // motor4_direction
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->motor4_direction), &(rhs->motor4_direction)))
  {
    return false;
  }
  // motor5_speed
  if (lhs->motor5_speed != rhs->motor5_speed) {
    return false;
  }
  // motor5_direction
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->motor5_direction), &(rhs->motor5_direction)))
  {
    return false;
  }
  // motor6_speed
  if (lhs->motor6_speed != rhs->motor6_speed) {
    return false;
  }
  // motor6_direction
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->motor6_direction), &(rhs->motor6_direction)))
  {
    return false;
  }
  return true;
}

bool
msg_interfaces__msg__MotorSix__copy(
  const msg_interfaces__msg__MotorSix * input,
  msg_interfaces__msg__MotorSix * output)
{
  if (!input || !output) {
    return false;
  }
  // motor1_speed
  output->motor1_speed = input->motor1_speed;
  // motor1_direction
  if (!rosidl_runtime_c__String__copy(
      &(input->motor1_direction), &(output->motor1_direction)))
  {
    return false;
  }
  // motor2_speed
  output->motor2_speed = input->motor2_speed;
  // motor2_direction
  if (!rosidl_runtime_c__String__copy(
      &(input->motor2_direction), &(output->motor2_direction)))
  {
    return false;
  }
  // motor3_speed
  output->motor3_speed = input->motor3_speed;
  // motor3_direction
  if (!rosidl_runtime_c__String__copy(
      &(input->motor3_direction), &(output->motor3_direction)))
  {
    return false;
  }
  // motor4_speed
  output->motor4_speed = input->motor4_speed;
  // motor4_direction
  if (!rosidl_runtime_c__String__copy(
      &(input->motor4_direction), &(output->motor4_direction)))
  {
    return false;
  }
  // motor5_speed
  output->motor5_speed = input->motor5_speed;
  // motor5_direction
  if (!rosidl_runtime_c__String__copy(
      &(input->motor5_direction), &(output->motor5_direction)))
  {
    return false;
  }
  // motor6_speed
  output->motor6_speed = input->motor6_speed;
  // motor6_direction
  if (!rosidl_runtime_c__String__copy(
      &(input->motor6_direction), &(output->motor6_direction)))
  {
    return false;
  }
  return true;
}

msg_interfaces__msg__MotorSix *
msg_interfaces__msg__MotorSix__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__MotorSix * msg = (msg_interfaces__msg__MotorSix *)allocator.allocate(sizeof(msg_interfaces__msg__MotorSix), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(msg_interfaces__msg__MotorSix));
  bool success = msg_interfaces__msg__MotorSix__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
msg_interfaces__msg__MotorSix__destroy(msg_interfaces__msg__MotorSix * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    msg_interfaces__msg__MotorSix__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
msg_interfaces__msg__MotorSix__Sequence__init(msg_interfaces__msg__MotorSix__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__MotorSix * data = NULL;

  if (size) {
    data = (msg_interfaces__msg__MotorSix *)allocator.zero_allocate(size, sizeof(msg_interfaces__msg__MotorSix), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = msg_interfaces__msg__MotorSix__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        msg_interfaces__msg__MotorSix__fini(&data[i - 1]);
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
msg_interfaces__msg__MotorSix__Sequence__fini(msg_interfaces__msg__MotorSix__Sequence * array)
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
      msg_interfaces__msg__MotorSix__fini(&array->data[i]);
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

msg_interfaces__msg__MotorSix__Sequence *
msg_interfaces__msg__MotorSix__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__MotorSix__Sequence * array = (msg_interfaces__msg__MotorSix__Sequence *)allocator.allocate(sizeof(msg_interfaces__msg__MotorSix__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = msg_interfaces__msg__MotorSix__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
msg_interfaces__msg__MotorSix__Sequence__destroy(msg_interfaces__msg__MotorSix__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    msg_interfaces__msg__MotorSix__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
msg_interfaces__msg__MotorSix__Sequence__are_equal(const msg_interfaces__msg__MotorSix__Sequence * lhs, const msg_interfaces__msg__MotorSix__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!msg_interfaces__msg__MotorSix__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
msg_interfaces__msg__MotorSix__Sequence__copy(
  const msg_interfaces__msg__MotorSix__Sequence * input,
  msg_interfaces__msg__MotorSix__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(msg_interfaces__msg__MotorSix);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    msg_interfaces__msg__MotorSix * data =
      (msg_interfaces__msg__MotorSix *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!msg_interfaces__msg__MotorSix__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          msg_interfaces__msg__MotorSix__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!msg_interfaces__msg__MotorSix__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
