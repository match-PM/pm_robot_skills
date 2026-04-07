// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pm_skills_interfaces:srv/CorrectFrameLaser.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME_LASER__STRUCT_H_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME_LASER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'frame_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CorrectFrameLaser in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__CorrectFrameLaser_Request
{
  rosidl_runtime_c__String frame_name;
  bool remeasure_after_correction;
  bool use_iterative_sensing;
} pm_skills_interfaces__srv__CorrectFrameLaser_Request;

// Struct for a sequence of pm_skills_interfaces__srv__CorrectFrameLaser_Request.
typedef struct pm_skills_interfaces__srv__CorrectFrameLaser_Request__Sequence
{
  pm_skills_interfaces__srv__CorrectFrameLaser_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__CorrectFrameLaser_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'correction_values'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'message'
// Member 'component_name'
// Member 'component_uuid'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CorrectFrameLaser in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__CorrectFrameLaser_Response
{
  bool success;
  geometry_msgs__msg__Vector3 correction_values;
  rosidl_runtime_c__String message;
  rosidl_runtime_c__String component_name;
  rosidl_runtime_c__String component_uuid;
} pm_skills_interfaces__srv__CorrectFrameLaser_Response;

// Struct for a sequence of pm_skills_interfaces__srv__CorrectFrameLaser_Response.
typedef struct pm_skills_interfaces__srv__CorrectFrameLaser_Response__Sequence
{
  pm_skills_interfaces__srv__CorrectFrameLaser_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__CorrectFrameLaser_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME_LASER__STRUCT_H_
