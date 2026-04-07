// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pm_skills_interfaces:srv/GripComponent.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__GRIP_COMPONENT__STRUCT_H_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__GRIP_COMPONENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'component_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GripComponent in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__GripComponent_Request
{
  rosidl_runtime_c__String component_name;
  bool align_orientation;
} pm_skills_interfaces__srv__GripComponent_Request;

// Struct for a sequence of pm_skills_interfaces__srv__GripComponent_Request.
typedef struct pm_skills_interfaces__srv__GripComponent_Request__Sequence
{
  pm_skills_interfaces__srv__GripComponent_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__GripComponent_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GripComponent in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__GripComponent_Response
{
  bool success;
  rosidl_runtime_c__String message;
} pm_skills_interfaces__srv__GripComponent_Response;

// Struct for a sequence of pm_skills_interfaces__srv__GripComponent_Response.
typedef struct pm_skills_interfaces__srv__GripComponent_Response__Sequence
{
  pm_skills_interfaces__srv__GripComponent_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__GripComponent_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__GRIP_COMPONENT__STRUCT_H_
