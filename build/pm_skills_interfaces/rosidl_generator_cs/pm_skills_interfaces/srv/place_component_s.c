// generated from rosidl_generator_cs/resource/idl.c.em
// with input from pm_skills_interfaces:srv/PlaceComponent.idl
// generated code does not contain a copyright notice




#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <pm_skills_interfaces/srv/place_component.h>
#include <rosidl_runtime_c/visibility_control.h>


ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__PlaceComponent_Request_native_read_field_align_orientation(void *message_handle)
{
  pm_skills_interfaces__srv__PlaceComponent_Request *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)message_handle;
  return ros_message->align_orientation;
}
ROSIDL_GENERATOR_C_EXPORT
float pm_skills_interfaces__srv__PlaceComponent_Request_native_read_field_x_offset_um(void *message_handle)
{
  pm_skills_interfaces__srv__PlaceComponent_Request *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)message_handle;
  return ros_message->x_offset_um;
}
ROSIDL_GENERATOR_C_EXPORT
float pm_skills_interfaces__srv__PlaceComponent_Request_native_read_field_y_offset_um(void *message_handle)
{
  pm_skills_interfaces__srv__PlaceComponent_Request *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)message_handle;
  return ros_message->y_offset_um;
}
ROSIDL_GENERATOR_C_EXPORT
float pm_skills_interfaces__srv__PlaceComponent_Request_native_read_field_z_offset_um(void *message_handle)
{
  pm_skills_interfaces__srv__PlaceComponent_Request *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)message_handle;
  return ros_message->z_offset_um;
}
ROSIDL_GENERATOR_C_EXPORT
float pm_skills_interfaces__srv__PlaceComponent_Request_native_read_field_rx_offset_deg(void *message_handle)
{
  pm_skills_interfaces__srv__PlaceComponent_Request *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)message_handle;
  return ros_message->rx_offset_deg;
}
ROSIDL_GENERATOR_C_EXPORT
float pm_skills_interfaces__srv__PlaceComponent_Request_native_read_field_ry_offset_deg(void *message_handle)
{
  pm_skills_interfaces__srv__PlaceComponent_Request *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)message_handle;
  return ros_message->ry_offset_deg;
}
ROSIDL_GENERATOR_C_EXPORT
float pm_skills_interfaces__srv__PlaceComponent_Request_native_read_field_rz_offset_deg(void *message_handle)
{
  pm_skills_interfaces__srv__PlaceComponent_Request *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)message_handle;
  return ros_message->rz_offset_deg;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__PlaceComponent_Request_native_write_field_align_orientation(void *message_handle, bool value)
{
  pm_skills_interfaces__srv__PlaceComponent_Request *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)message_handle;
  ros_message->align_orientation = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__PlaceComponent_Request_native_write_field_x_offset_um(void *message_handle, float value)
{
  pm_skills_interfaces__srv__PlaceComponent_Request *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)message_handle;
  ros_message->x_offset_um = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__PlaceComponent_Request_native_write_field_y_offset_um(void *message_handle, float value)
{
  pm_skills_interfaces__srv__PlaceComponent_Request *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)message_handle;
  ros_message->y_offset_um = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__PlaceComponent_Request_native_write_field_z_offset_um(void *message_handle, float value)
{
  pm_skills_interfaces__srv__PlaceComponent_Request *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)message_handle;
  ros_message->z_offset_um = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__PlaceComponent_Request_native_write_field_rx_offset_deg(void *message_handle, float value)
{
  pm_skills_interfaces__srv__PlaceComponent_Request *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)message_handle;
  ros_message->rx_offset_deg = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__PlaceComponent_Request_native_write_field_ry_offset_deg(void *message_handle, float value)
{
  pm_skills_interfaces__srv__PlaceComponent_Request *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)message_handle;
  ros_message->ry_offset_deg = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__PlaceComponent_Request_native_write_field_rz_offset_deg(void *message_handle, float value)
{
  pm_skills_interfaces__srv__PlaceComponent_Request *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Request *)message_handle;
  ros_message->rz_offset_deg = value;
}











#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__PlaceComponent_Response_native_read_field_success(void *message_handle)
{
  pm_skills_interfaces__srv__PlaceComponent_Response *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Response *)message_handle;
  return ros_message->success;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__PlaceComponent_Response_native_read_field_message(void *message_handle)
{
  pm_skills_interfaces__srv__PlaceComponent_Response *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Response *)message_handle;
  return ros_message->message.data;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__PlaceComponent_Response_native_write_field_success(void *message_handle, bool value)
{
  pm_skills_interfaces__srv__PlaceComponent_Response *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Response *)message_handle;
  ros_message->success = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__PlaceComponent_Response_native_write_field_message(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__PlaceComponent_Response *ros_message = (pm_skills_interfaces__srv__PlaceComponent_Response *)message_handle;
  if (&ros_message->message.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->message);
    rosidl_runtime_c__String__init(&ros_message->message);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->message, value);
}








