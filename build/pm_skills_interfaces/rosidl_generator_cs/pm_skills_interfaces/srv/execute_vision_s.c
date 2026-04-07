// generated from rosidl_generator_cs/resource/idl.c.em
// with input from pm_skills_interfaces:srv/ExecuteVision.idl
// generated code does not contain a copyright notice




#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <pm_skills_interfaces/srv/execute_vision.h>
#include <rosidl_runtime_c/visibility_control.h>

#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__ExecuteVision_Request_native_read_field_process_filename(void *message_handle)
{
  pm_skills_interfaces__srv__ExecuteVision_Request *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Request *)message_handle;
  return ros_message->process_filename.data;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__ExecuteVision_Request_native_read_field_camera_config_filename(void *message_handle)
{
  pm_skills_interfaces__srv__ExecuteVision_Request *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Request *)message_handle;
  return ros_message->camera_config_filename.data;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__ExecuteVision_Request_native_read_field_process_uid(void *message_handle)
{
  pm_skills_interfaces__srv__ExecuteVision_Request *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Request *)message_handle;
  return ros_message->process_uid.data;
}
ROSIDL_GENERATOR_C_EXPORT
float pm_skills_interfaces__srv__ExecuteVision_Request_native_read_field_image_display_time(void *message_handle)
{
  pm_skills_interfaces__srv__ExecuteVision_Request *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Request *)message_handle;
  return ros_message->image_display_time;
}
ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__ExecuteVision_Request_native_read_field_run_cross_validation(void *message_handle)
{
  pm_skills_interfaces__srv__ExecuteVision_Request *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Request *)message_handle;
  return ros_message->run_cross_validation;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__ExecuteVision_Request_native_write_field_process_filename(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__ExecuteVision_Request *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Request *)message_handle;
  if (&ros_message->process_filename.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->process_filename);
    rosidl_runtime_c__String__init(&ros_message->process_filename);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->process_filename, value);
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__ExecuteVision_Request_native_write_field_camera_config_filename(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__ExecuteVision_Request *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Request *)message_handle;
  if (&ros_message->camera_config_filename.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->camera_config_filename);
    rosidl_runtime_c__String__init(&ros_message->camera_config_filename);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->camera_config_filename, value);
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__ExecuteVision_Request_native_write_field_process_uid(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__ExecuteVision_Request *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Request *)message_handle;
  if (&ros_message->process_uid.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->process_uid);
    rosidl_runtime_c__String__init(&ros_message->process_uid);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->process_uid, value);
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__ExecuteVision_Request_native_write_field_image_display_time(void *message_handle, float value)
{
  pm_skills_interfaces__srv__ExecuteVision_Request *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Request *)message_handle;
  ros_message->image_display_time = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__ExecuteVision_Request_native_write_field_run_cross_validation(void *message_handle, bool value)
{
  pm_skills_interfaces__srv__ExecuteVision_Request *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Request *)message_handle;
  ros_message->run_cross_validation = value;
}











#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__ExecuteVision_Response_native_read_field_success(void *message_handle)
{
  pm_skills_interfaces__srv__ExecuteVision_Response *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Response *)message_handle;
  return ros_message->success;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__ExecuteVision_Response_native_read_field_results_dict(void *message_handle)
{
  pm_skills_interfaces__srv__ExecuteVision_Response *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Response *)message_handle;
  return ros_message->results_dict.data;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__ExecuteVision_Response_native_read_field_results_path(void *message_handle)
{
  pm_skills_interfaces__srv__ExecuteVision_Response *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Response *)message_handle;
  return ros_message->results_path.data;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__ExecuteVision_Response_native_read_field_process_uid(void *message_handle)
{
  pm_skills_interfaces__srv__ExecuteVision_Response *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Response *)message_handle;
  return ros_message->process_uid.data;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__ExecuteVision_Response_native_write_field_success(void *message_handle, bool value)
{
  pm_skills_interfaces__srv__ExecuteVision_Response *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Response *)message_handle;
  ros_message->success = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__ExecuteVision_Response_native_write_field_results_dict(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__ExecuteVision_Response *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Response *)message_handle;
  if (&ros_message->results_dict.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->results_dict);
    rosidl_runtime_c__String__init(&ros_message->results_dict);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->results_dict, value);
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__ExecuteVision_Response_native_write_field_results_path(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__ExecuteVision_Response *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Response *)message_handle;
  if (&ros_message->results_path.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->results_path);
    rosidl_runtime_c__String__init(&ros_message->results_path);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->results_path, value);
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__ExecuteVision_Response_native_write_field_process_uid(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__ExecuteVision_Response *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Response *)message_handle;
  if (&ros_message->process_uid.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->process_uid);
    rosidl_runtime_c__String__init(&ros_message->process_uid);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->process_uid, value);
}

ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__ExecuteVision_Response_native_write_field_points(float *value, int size, void *message_handle)
{
  pm_skills_interfaces__srv__ExecuteVision_Response *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Response *)message_handle;
  size_t previous_sequence_size = ros_message->points.size;
  bool size_changed = previous_sequence_size != (size_t)size;
  if (size_changed && previous_sequence_size != 0)
  {
    rosidl_runtime_c__float__Sequence__fini(&ros_message->points);
  }
  if (size_changed)
  {
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->points, size))
      return false;
  }
  float *dest = ros_message->points.data;
  memcpy(dest, value, sizeof(float)*size);
  return true;
}

ROSIDL_GENERATOR_C_EXPORT
float *pm_skills_interfaces__srv__ExecuteVision_Response_native_read_field_points(int *size, void *message_handle)
{
  pm_skills_interfaces__srv__ExecuteVision_Response *ros_message = (pm_skills_interfaces__srv__ExecuteVision_Response *)message_handle;
  *size = ros_message->points.size;
  return ros_message->points.data;
}






