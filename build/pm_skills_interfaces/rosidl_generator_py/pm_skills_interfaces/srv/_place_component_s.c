// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from pm_skills_interfaces:srv/PlaceComponent.idl
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
#include "pm_skills_interfaces/srv/detail/place_component__struct.h"
#include "pm_skills_interfaces/srv/detail/place_component__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__place_component__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[65];
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
    assert(strncmp("pm_skills_interfaces.srv._place_component.PlaceComponent_Request", full_classname_dest, 64) == 0);
  }
  pm_skills_interfaces__srv__PlaceComponent_Request * ros_message = _ros_message;
  {  // align_orientation
    PyObject * field = PyObject_GetAttrString(_pymsg, "align_orientation");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->align_orientation = (Py_True == field);
    Py_DECREF(field);
  }
  {  // x_offset_um
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_offset_um");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_offset_um = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_offset_um
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_offset_um");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_offset_um = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_offset_um
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_offset_um");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_offset_um = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rx_offset_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "rx_offset_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rx_offset_deg = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ry_offset_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "ry_offset_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ry_offset_deg = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rz_offset_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "rz_offset_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rz_offset_deg = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * pm_skills_interfaces__srv__place_component__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PlaceComponent_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("pm_skills_interfaces.srv._place_component");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PlaceComponent_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  pm_skills_interfaces__srv__PlaceComponent_Request * ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)raw_ros_message;
  {  // align_orientation
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->align_orientation ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "align_orientation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_offset_um
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_offset_um);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_offset_um", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_offset_um
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_offset_um);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_offset_um", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_offset_um
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_offset_um);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_offset_um", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rx_offset_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rx_offset_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rx_offset_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ry_offset_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ry_offset_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ry_offset_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rz_offset_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rz_offset_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rz_offset_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "pm_skills_interfaces/srv/detail/place_component__struct.h"
// already included above
// #include "pm_skills_interfaces/srv/detail/place_component__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__place_component__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("pm_skills_interfaces.srv._place_component.PlaceComponent_Response", full_classname_dest, 65) == 0);
  }
  pm_skills_interfaces__srv__PlaceComponent_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // message
    PyObject * field = PyObject_GetAttrString(_pymsg, "message");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->message, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * pm_skills_interfaces__srv__place_component__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PlaceComponent_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("pm_skills_interfaces.srv._place_component");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PlaceComponent_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  pm_skills_interfaces__srv__PlaceComponent_Response * ros_message = (pm_skills_interfaces__srv__PlaceComponent_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // message
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->message.data,
      strlen(ros_message->message.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
