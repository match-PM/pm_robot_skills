// generated from rosidl_generator_cs/resource/idl.c.em
// with input from pm_skills_interfaces:srv/CorrectFrame.idl
// generated code does not contain a copyright notice




#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <pm_skills_interfaces/srv/correct_frame.h>
#include <rosidl_runtime_c/visibility_control.h>


ROSIDL_GENERATOR_C_EXPORT
uint8_t pm_skills_interfaces__srv__CorrectFrame_Request_native_read_field_structure_needs_at_least_one_member(void *message_handle)
{
  pm_skills_interfaces__srv__CorrectFrame_Request *ros_message = (pm_skills_interfaces__srv__CorrectFrame_Request *)message_handle;
  return ros_message->structure_needs_at_least_one_member;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__CorrectFrame_Request_native_write_field_structure_needs_at_least_one_member(void *message_handle, uint8_t value)
{
  pm_skills_interfaces__srv__CorrectFrame_Request *ros_message = (pm_skills_interfaces__srv__CorrectFrame_Request *)message_handle;
  ros_message->structure_needs_at_least_one_member = value;
}











#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__CorrectFrame_Response_native_read_field_success(void *message_handle)
{
  pm_skills_interfaces__srv__CorrectFrame_Response *ros_message = (pm_skills_interfaces__srv__CorrectFrame_Response *)message_handle;
  return ros_message->success;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__CorrectFrame_Response_native_read_field_message(void *message_handle)
{
  pm_skills_interfaces__srv__CorrectFrame_Response *ros_message = (pm_skills_interfaces__srv__CorrectFrame_Response *)message_handle;
  return ros_message->message.data;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__CorrectFrame_Response_native_write_field_success(void *message_handle, bool value)
{
  pm_skills_interfaces__srv__CorrectFrame_Response *ros_message = (pm_skills_interfaces__srv__CorrectFrame_Response *)message_handle;
  ros_message->success = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__CorrectFrame_Response_native_write_field_message(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__CorrectFrame_Response *ros_message = (pm_skills_interfaces__srv__CorrectFrame_Response *)message_handle;
  if (&ros_message->message.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->message);
    rosidl_runtime_c__String__init(&ros_message->message);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->message, value);
}







ROSIDL_GENERATOR_C_EXPORT
void * pm_skills_interfaces__srv__CorrectFrame_Response_native_get_nested_message_handle_correction_values(void *message_handle)
{
  pm_skills_interfaces__srv__CorrectFrame_Response *ros_message = (pm_skills_interfaces__srv__CorrectFrame_Response *)message_handle;
  geometry_msgs__msg__Vector3 *nested_message = &(ros_message->correction_values);
  return (void *)nested_message;
}



