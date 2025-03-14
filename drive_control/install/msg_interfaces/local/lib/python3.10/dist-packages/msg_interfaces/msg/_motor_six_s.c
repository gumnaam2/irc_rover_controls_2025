// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from msg_interfaces:msg/MotorSix.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "msg_interfaces/msg/detail/motor_six__struct.h"
#include "msg_interfaces/msg/detail/motor_six__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool msg_interfaces__msg__motor_six__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[39];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("msg_interfaces.msg._motor_six.MotorSix", full_classname_dest, 38) == 0);
  }
  msg_interfaces__msg__MotorSix * ros_message = _ros_message;
  {  // motor1_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor1_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->motor1_speed = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor1_direction
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor1_direction");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->motor1_direction, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // motor2_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor2_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->motor2_speed = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor2_direction
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor2_direction");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->motor2_direction, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // motor3_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor3_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->motor3_speed = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor3_direction
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor3_direction");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->motor3_direction, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // motor4_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor4_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->motor4_speed = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor4_direction
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor4_direction");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->motor4_direction, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // motor5_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor5_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->motor5_speed = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor5_direction
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor5_direction");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->motor5_direction, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // motor6_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor6_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->motor6_speed = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor6_direction
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor6_direction");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->motor6_direction, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * msg_interfaces__msg__motor_six__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MotorSix */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("msg_interfaces.msg._motor_six");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MotorSix");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  msg_interfaces__msg__MotorSix * ros_message = (msg_interfaces__msg__MotorSix *)raw_ros_message;
  {  // motor1_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->motor1_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor1_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor1_direction
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->motor1_direction.data,
      strlen(ros_message->motor1_direction.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor1_direction", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor2_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->motor2_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor2_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor2_direction
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->motor2_direction.data,
      strlen(ros_message->motor2_direction.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor2_direction", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor3_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->motor3_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor3_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor3_direction
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->motor3_direction.data,
      strlen(ros_message->motor3_direction.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor3_direction", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor4_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->motor4_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor4_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor4_direction
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->motor4_direction.data,
      strlen(ros_message->motor4_direction.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor4_direction", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor5_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->motor5_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor5_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor5_direction
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->motor5_direction.data,
      strlen(ros_message->motor5_direction.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor5_direction", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor6_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->motor6_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor6_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor6_direction
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->motor6_direction.data,
      strlen(ros_message->motor6_direction.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor6_direction", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
