// generated from rosidl_generator_cs/resource/idl.c.em
// with input from pm_skills_interfaces:srv/DispenseAdhesive.idl
// generated code does not contain a copyright notice




#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <pm_skills_interfaces/srv/dispense_adhesive.h>
#include <rosidl_runtime_c/visibility_control.h>

#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__DispenseAdhesive_Request_native_read_field_dispenser(void *message_handle)
{
  pm_skills_interfaces__srv__DispenseAdhesive_Request *ros_message = (pm_skills_interfaces__srv__DispenseAdhesive_Request *)message_handle;
  return ros_message->dispenser.data;
}
ROSIDL_GENERATOR_C_EXPORT
float pm_skills_interfaces__srv__DispenseAdhesive_Request_native_read_field_dispense_time(void *message_handle)
{
  pm_skills_interfaces__srv__DispenseAdhesive_Request *ros_message = (pm_skills_interfaces__srv__DispenseAdhesive_Request *)message_handle;
  return ros_message->dispense_time;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__DispenseAdhesive_Request_native_write_field_dispenser(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__DispenseAdhesive_Request *ros_message = (pm_skills_interfaces__srv__DispenseAdhesive_Request *)message_handle;
  if (&ros_message->dispenser.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->dispenser);
    rosidl_runtime_c__String__init(&ros_message->dispenser);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->dispenser, value);
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__DispenseAdhesive_Request_native_write_field_dispense_time(void *message_handle, float value)
{
  pm_skills_interfaces__srv__DispenseAdhesive_Request *ros_message = (pm_skills_interfaces__srv__DispenseAdhesive_Request *)message_handle;
  ros_message->dispense_time = value;
}











#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__DispenseAdhesive_Response_native_read_field_success(void *message_handle)
{
  pm_skills_interfaces__srv__DispenseAdhesive_Response *ros_message = (pm_skills_interfaces__srv__DispenseAdhesive_Response *)message_handle;
  return ros_message->success;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__DispenseAdhesive_Response_native_read_field_message(void *message_handle)
{
  pm_skills_interfaces__srv__DispenseAdhesive_Response *ros_message = (pm_skills_interfaces__srv__DispenseAdhesive_Response *)message_handle;
  return ros_message->message.data;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__DispenseAdhesive_Response_native_write_field_success(void *message_handle, bool value)
{
  pm_skills_interfaces__srv__DispenseAdhesive_Response *ros_message = (pm_skills_interfaces__srv__DispenseAdhesive_Response *)message_handle;
  ros_message->success = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__DispenseAdhesive_Response_native_write_field_message(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__DispenseAdhesive_Response *ros_message = (pm_skills_interfaces__srv__DispenseAdhesive_Response *)message_handle;
  if (&ros_message->message.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->message);
    rosidl_runtime_c__String__init(&ros_message->message);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->message, value);
}








