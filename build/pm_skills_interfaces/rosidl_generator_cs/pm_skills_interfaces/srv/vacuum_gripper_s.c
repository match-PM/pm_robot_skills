// generated from rosidl_generator_cs/resource/idl.c.em
// with input from pm_skills_interfaces:srv/VacuumGripper.idl
// generated code does not contain a copyright notice




#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <pm_skills_interfaces/srv/vacuum_gripper.h>
#include <rosidl_runtime_c/visibility_control.h>


ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__VacuumGripper_Request_native_read_field_activate_vacuum(void *message_handle)
{
  pm_skills_interfaces__srv__VacuumGripper_Request *ros_message = (pm_skills_interfaces__srv__VacuumGripper_Request *)message_handle;
  return ros_message->activate_vacuum;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__VacuumGripper_Request_native_write_field_activate_vacuum(void *message_handle, bool value)
{
  pm_skills_interfaces__srv__VacuumGripper_Request *ros_message = (pm_skills_interfaces__srv__VacuumGripper_Request *)message_handle;
  ros_message->activate_vacuum = value;
}











#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__VacuumGripper_Response_native_read_field_success(void *message_handle)
{
  pm_skills_interfaces__srv__VacuumGripper_Response *ros_message = (pm_skills_interfaces__srv__VacuumGripper_Response *)message_handle;
  return ros_message->success;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__VacuumGripper_Response_native_read_field_message(void *message_handle)
{
  pm_skills_interfaces__srv__VacuumGripper_Response *ros_message = (pm_skills_interfaces__srv__VacuumGripper_Response *)message_handle;
  return ros_message->message.data;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__VacuumGripper_Response_native_write_field_success(void *message_handle, bool value)
{
  pm_skills_interfaces__srv__VacuumGripper_Response *ros_message = (pm_skills_interfaces__srv__VacuumGripper_Response *)message_handle;
  ros_message->success = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__VacuumGripper_Response_native_write_field_message(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__VacuumGripper_Response *ros_message = (pm_skills_interfaces__srv__VacuumGripper_Response *)message_handle;
  if (&ros_message->message.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->message);
    rosidl_runtime_c__String__init(&ros_message->message);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->message, value);
}








