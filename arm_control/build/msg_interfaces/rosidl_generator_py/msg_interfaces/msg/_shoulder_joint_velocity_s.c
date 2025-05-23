// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from msg_interfaces:msg/ShoulderJointVelocity.idl
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
#include "msg_interfaces/msg/detail/shoulder_joint_velocity__struct.h"
#include "msg_interfaces/msg/detail/shoulder_joint_velocity__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool msg_interfaces__msg__shoulder_joint_velocity__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[66];
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
    assert(strncmp("msg_interfaces.msg._shoulder_joint_velocity.ShoulderJointVelocity", full_classname_dest, 65) == 0);
  }
  msg_interfaces__msg__ShoulderJointVelocity * ros_message = _ros_message;
  {  // angular_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "angular_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angular_speed = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * msg_interfaces__msg__shoulder_joint_velocity__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ShoulderJointVelocity */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("msg_interfaces.msg._shoulder_joint_velocity");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ShoulderJointVelocity");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  msg_interfaces__msg__ShoulderJointVelocity * ros_message = (msg_interfaces__msg__ShoulderJointVelocity *)raw_ros_message;
  {  // angular_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angular_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angular_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
