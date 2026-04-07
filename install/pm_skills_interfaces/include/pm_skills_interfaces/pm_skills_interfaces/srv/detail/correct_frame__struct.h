// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pm_skills_interfaces:srv/CorrectFrame.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME__STRUCT_H_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/CorrectFrame in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__CorrectFrame_Request
{
  uint8_t structure_needs_at_least_one_member;
} pm_skills_interfaces__srv__CorrectFrame_Request;

// Struct for a sequence of pm_skills_interfaces__srv__CorrectFrame_Request.
typedef struct pm_skills_interfaces__srv__CorrectFrame_Request__Sequence
{
  pm_skills_interfaces__srv__CorrectFrame_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__CorrectFrame_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'correction_values'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CorrectFrame in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__CorrectFrame_Response
{
  bool success;
  geometry_msgs__msg__Vector3 correction_values;
  rosidl_runtime_c__String message;
} pm_skills_interfaces__srv__CorrectFrame_Response;

// Struct for a sequence of pm_skills_interfaces__srv__CorrectFrame_Response.
typedef struct pm_skills_interfaces__srv__CorrectFrame_Response__Sequence
{
  pm_skills_interfaces__srv__CorrectFrame_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__CorrectFrame_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME__STRUCT_H_
