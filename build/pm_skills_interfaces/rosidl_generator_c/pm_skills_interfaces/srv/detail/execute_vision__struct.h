// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pm_skills_interfaces:srv/ExecuteVision.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__EXECUTE_VISION__STRUCT_H_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__EXECUTE_VISION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'process_filename'
// Member 'camera_config_filename'
// Member 'process_uid'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ExecuteVision in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__ExecuteVision_Request
{
  rosidl_runtime_c__String process_filename;
  rosidl_runtime_c__String camera_config_filename;
  rosidl_runtime_c__String process_uid;
  float image_display_time;
  bool run_cross_validation;
} pm_skills_interfaces__srv__ExecuteVision_Request;

// Struct for a sequence of pm_skills_interfaces__srv__ExecuteVision_Request.
typedef struct pm_skills_interfaces__srv__ExecuteVision_Request__Sequence
{
  pm_skills_interfaces__srv__ExecuteVision_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__ExecuteVision_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'results_dict'
// Member 'results_path'
// Member 'process_uid'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'points'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/ExecuteVision in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__ExecuteVision_Response
{
  bool success;
  rosidl_runtime_c__String results_dict;
  rosidl_runtime_c__String results_path;
  rosidl_runtime_c__float__Sequence points;
  rosidl_runtime_c__String process_uid;
} pm_skills_interfaces__srv__ExecuteVision_Response;

// Struct for a sequence of pm_skills_interfaces__srv__ExecuteVision_Response.
typedef struct pm_skills_interfaces__srv__ExecuteVision_Response__Sequence
{
  pm_skills_interfaces__srv__ExecuteVision_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__ExecuteVision_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__EXECUTE_VISION__STRUCT_H_
