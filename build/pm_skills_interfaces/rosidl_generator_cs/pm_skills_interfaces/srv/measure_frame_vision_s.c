// generated from rosidl_generator_cs/resource/idl.c.em
// with input from pm_skills_interfaces:srv/MeasureFrameVision.idl
// generated code does not contain a copyright notice




#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <pm_skills_interfaces/srv/measure_frame_vision.h>
#include <rosidl_runtime_c/visibility_control.h>

#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__MeasureFrameVision_Request_native_read_field_frame_name(void *message_handle)
{
  pm_skills_interfaces__srv__MeasureFrameVision_Request *ros_message = (pm_skills_interfaces__srv__MeasureFrameVision_Request *)message_handle;
  return ros_message->frame_name.data;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__MeasureFrameVision_Request_native_read_field_vision_process_file_name(void *message_handle)
{
  pm_skills_interfaces__srv__MeasureFrameVision_Request *ros_message = (pm_skills_interfaces__srv__MeasureFrameVision_Request *)message_handle;
  return ros_message->vision_process_file_name.data;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__MeasureFrameVision_Request_native_write_field_frame_name(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__MeasureFrameVision_Request *ros_message = (pm_skills_interfaces__srv__MeasureFrameVision_Request *)message_handle;
  if (&ros_message->frame_name.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->frame_name);
    rosidl_runtime_c__String__init(&ros_message->frame_name);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->frame_name, value);
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__MeasureFrameVision_Request_native_write_field_vision_process_file_name(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__MeasureFrameVision_Request *ros_message = (pm_skills_interfaces__srv__MeasureFrameVision_Request *)message_handle;
  if (&ros_message->vision_process_file_name.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->vision_process_file_name);
    rosidl_runtime_c__String__init(&ros_message->vision_process_file_name);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->vision_process_file_name, value);
}











#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__MeasureFrameVision_Response_native_read_field_success(void *message_handle)
{
  pm_skills_interfaces__srv__MeasureFrameVision_Response *ros_message = (pm_skills_interfaces__srv__MeasureFrameVision_Response *)message_handle;
  return ros_message->success;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__MeasureFrameVision_Response_native_read_field_message(void *message_handle)
{
  pm_skills_interfaces__srv__MeasureFrameVision_Response *ros_message = (pm_skills_interfaces__srv__MeasureFrameVision_Response *)message_handle;
  return ros_message->message.data;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__MeasureFrameVision_Response_native_read_field_component_name(void *message_handle)
{
  pm_skills_interfaces__srv__MeasureFrameVision_Response *ros_message = (pm_skills_interfaces__srv__MeasureFrameVision_Response *)message_handle;
  return ros_message->component_name.data;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__MeasureFrameVision_Response_native_read_field_component_uuid(void *message_handle)
{
  pm_skills_interfaces__srv__MeasureFrameVision_Response *ros_message = (pm_skills_interfaces__srv__MeasureFrameVision_Response *)message_handle;
  return ros_message->component_uuid.data;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__MeasureFrameVision_Response_native_write_field_success(void *message_handle, bool value)
{
  pm_skills_interfaces__srv__MeasureFrameVision_Response *ros_message = (pm_skills_interfaces__srv__MeasureFrameVision_Response *)message_handle;
  ros_message->success = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__MeasureFrameVision_Response_native_write_field_message(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__MeasureFrameVision_Response *ros_message = (pm_skills_interfaces__srv__MeasureFrameVision_Response *)message_handle;
  if (&ros_message->message.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->message);
    rosidl_runtime_c__String__init(&ros_message->message);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->message, value);
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__MeasureFrameVision_Response_native_write_field_component_name(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__MeasureFrameVision_Response *ros_message = (pm_skills_interfaces__srv__MeasureFrameVision_Response *)message_handle;
  if (&ros_message->component_name.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->component_name);
    rosidl_runtime_c__String__init(&ros_message->component_name);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->component_name, value);
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__MeasureFrameVision_Response_native_write_field_component_uuid(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__MeasureFrameVision_Response *ros_message = (pm_skills_interfaces__srv__MeasureFrameVision_Response *)message_handle;
  if (&ros_message->component_uuid.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->component_uuid);
    rosidl_runtime_c__String__init(&ros_message->component_uuid);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->component_uuid, value);
}







ROSIDL_GENERATOR_C_EXPORT
void * pm_skills_interfaces__srv__MeasureFrameVision_Response_native_get_nested_message_handle_result_vector(void *message_handle)
{
  pm_skills_interfaces__srv__MeasureFrameVision_Response *ros_message = (pm_skills_interfaces__srv__MeasureFrameVision_Response *)message_handle;
  geometry_msgs__msg__Vector3 *nested_message = &(ros_message->result_vector);
  return (void *)nested_message;
}


ROSIDL_GENERATOR_C_EXPORT
void * pm_skills_interfaces__srv__MeasureFrameVision_Response_native_get_nested_message_handle_vision_response(void *message_handle)
{
  pm_skills_interfaces__srv__MeasureFrameVision_Response *ros_message = (pm_skills_interfaces__srv__MeasureFrameVision_Response *)message_handle;
  pm_vision_interfaces__msg__VisionResponse *nested_message = &(ros_message->vision_response);
  return (void *)nested_message;
}



