// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from msg_interfaces:msg/TargetPose.idl
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
#include "msg_interfaces/msg/detail/target_pose__struct.h"
#include "msg_interfaces/msg/detail/target_pose__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool msg_interfaces__msg__target_pose__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[43];
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
    assert(strncmp("msg_interfaces.msg._target_pose.TargetPose", full_classname_dest, 42) == 0);
  }
  msg_interfaces__msg__TargetPose * ros_message = _ros_message;
  {  // base
    PyObject * field = PyObject_GetAttrString(_pymsg, "base");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->base = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // shoulder
    PyObject * field = PyObject_GetAttrString(_pymsg, "shoulder");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->shoulder = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // elbow
    PyObject * field = PyObject_GetAttrString(_pymsg, "elbow");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->elbow = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // wrist_up_down
    PyObject * field = PyObject_GetAttrString(_pymsg, "wrist_up_down");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->wrist_up_down = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // wrist_rot
    PyObject * field = PyObject_GetAttrString(_pymsg, "wrist_rot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->wrist_rot = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * msg_interfaces__msg__target_pose__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TargetPose */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("msg_interfaces.msg._target_pose");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TargetPose");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  msg_interfaces__msg__TargetPose * ros_message = (msg_interfaces__msg__TargetPose *)raw_ros_message;
  {  // base
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->base);
    {
      int rc = PyObject_SetAttrString(_pymessage, "base", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // shoulder
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->shoulder);
    {
      int rc = PyObject_SetAttrString(_pymessage, "shoulder", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // elbow
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->elbow);
    {
      int rc = PyObject_SetAttrString(_pymessage, "elbow", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wrist_up_down
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->wrist_up_down);
    {
      int rc = PyObject_SetAttrString(_pymessage, "wrist_up_down", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wrist_rot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->wrist_rot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "wrist_rot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
