// generated from rosidl_generator_cs/resource/idl.c.em
// with input from pm_skills_interfaces:srv/ConfocalLaser.idl
// generated code does not contain a copyright notice




#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <pm_skills_interfaces/srv/confocal_laser.h>
#include <rosidl_runtime_c/visibility_control.h>

#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__ConfocalLaser_Request_native_read_field_laser(void *message_handle)
{
  pm_skills_interfaces__srv__ConfocalLaser_Request *ros_message = (pm_skills_interfaces__srv__ConfocalLaser_Request *)message_handle;
  return ros_message->laser.data;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__ConfocalLaser_Request_native_write_field_laser(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__ConfocalLaser_Request *ros_message = (pm_skills_interfaces__srv__ConfocalLaser_Request *)message_handle;
  if (&ros_message->laser.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->laser);
    rosidl_runtime_c__String__init(&ros_message->laser);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->laser, value);
}











#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__ConfocalLaser_Response_native_read_field_success(void *message_handle)
{
  pm_skills_interfaces__srv__ConfocalLaser_Response *ros_message = (pm_skills_interfaces__srv__ConfocalLaser_Response *)message_handle;
  return ros_message->success;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__ConfocalLaser_Response_native_read_field_result(void *message_handle)
{
  pm_skills_interfaces__srv__ConfocalLaser_Response *ros_message = (pm_skills_interfaces__srv__ConfocalLaser_Response *)message_handle;
  return ros_message->result.data;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__ConfocalLaser_Response_native_write_field_success(void *message_handle, bool value)
{
  pm_skills_interfaces__srv__ConfocalLaser_Response *ros_message = (pm_skills_interfaces__srv__ConfocalLaser_Response *)message_handle;
  ros_message->success = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__ConfocalLaser_Response_native_write_field_result(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__ConfocalLaser_Response *ros_message = (pm_skills_interfaces__srv__ConfocalLaser_Response *)message_handle;
  if (&ros_message->result.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->result);
    rosidl_runtime_c__String__init(&ros_message->result);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->result, value);
}








