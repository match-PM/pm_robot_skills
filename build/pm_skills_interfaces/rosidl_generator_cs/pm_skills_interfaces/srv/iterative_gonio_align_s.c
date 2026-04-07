// generated from rosidl_generator_cs/resource/idl.c.em
// with input from pm_skills_interfaces:srv/IterativeGonioAlign.idl
// generated code does not contain a copyright notice




#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <pm_skills_interfaces/srv/iterative_gonio_align.h>
#include <rosidl_runtime_c/visibility_control.h>

#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_read_field_confocal_laser(void *message_handle)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Request *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Request *)message_handle;
  return ros_message->confocal_laser;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_read_field_target_alignment_frame(void *message_handle)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Request *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Request *)message_handle;
  return ros_message->target_alignment_frame.data;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_read_field_gonio_endeffector_frame(void *message_handle)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Request *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Request *)message_handle;
  return ros_message->gonio_endeffector_frame.data;
}
ROSIDL_GENERATOR_C_EXPORT
int32_t pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_read_field_num_iterations(void *message_handle)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Request *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Request *)message_handle;
  return ros_message->num_iterations;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_write_field_confocal_laser(void *message_handle, bool value)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Request *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Request *)message_handle;
  ros_message->confocal_laser = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_write_field_target_alignment_frame(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Request *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Request *)message_handle;
  if (&ros_message->target_alignment_frame.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->target_alignment_frame);
    rosidl_runtime_c__String__init(&ros_message->target_alignment_frame);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->target_alignment_frame, value);
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_write_field_gonio_endeffector_frame(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Request *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Request *)message_handle;
  if (&ros_message->gonio_endeffector_frame.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->gonio_endeffector_frame);
    rosidl_runtime_c__String__init(&ros_message->gonio_endeffector_frame);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->gonio_endeffector_frame, value);
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_write_field_num_iterations(void *message_handle, int32_t value)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Request *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Request *)message_handle;
  ros_message->num_iterations = value;
}



ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_write_field_frames_to_measure(char *value, int index, void *message_handle)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Request *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Request *)message_handle;
  if (&ros_message->frames_to_measure.data[index].data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->frames_to_measure.data[index]);
    rosidl_runtime_c__String__init(&ros_message->frames_to_measure.data[index]);
  }
  rosidl_runtime_c__String__assign(&ros_message->frames_to_measure.data[index], value);
  return true;
}

ROSIDL_GENERATOR_C_EXPORT
char *pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_read_field_frames_to_measure(int index, void *message_handle)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Request *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Request *)message_handle;
  return ros_message->frames_to_measure.data[index].data;
}




ROSIDL_GENERATOR_C_EXPORT
int pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_get_array_size_frames_to_measure(void *message_handle)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Request *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Request *)message_handle;
  return ros_message->frames_to_measure.size;
}

ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_init_sequence_frames_to_measure(void *message_handle, int size)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Request *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Request *)message_handle;
  size_t previous_sequence_size = ros_message->frames_to_measure.size;
  bool size_changed = previous_sequence_size != (size_t)size;
  if (size_changed && previous_sequence_size != 0)
  {
    rosidl_runtime_c__String__Sequence__fini(&ros_message->frames_to_measure); //Supports same message reuse
  }
  if (size_changed)
  {
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->frames_to_measure, size))
      return false;
  }
  return true;
}





#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

ROSIDL_GENERATOR_C_EXPORT
bool pm_skills_interfaces__srv__IterativeGonioAlign_Response_native_read_field_success(void *message_handle)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Response *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Response *)message_handle;
  return ros_message->success;
}
ROSIDL_GENERATOR_C_EXPORT
const char * pm_skills_interfaces__srv__IterativeGonioAlign_Response_native_read_field_message(void *message_handle)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Response *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Response *)message_handle;
  return ros_message->message.data;
}


ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__IterativeGonioAlign_Response_native_write_field_success(void *message_handle, bool value)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Response *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Response *)message_handle;
  ros_message->success = value;
}
ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__IterativeGonioAlign_Response_native_write_field_message(void *message_handle, const char * value)
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Response *ros_message = (pm_skills_interfaces__srv__IterativeGonioAlign_Response *)message_handle;
  if (&ros_message->message.data)
  { // reinitializing string if message is being reused
    rosidl_runtime_c__String__fini(&ros_message->message);
    rosidl_runtime_c__String__init(&ros_message->message);
  }
  rosidl_runtime_c__String__assign(
    &ros_message->message, value);
}








