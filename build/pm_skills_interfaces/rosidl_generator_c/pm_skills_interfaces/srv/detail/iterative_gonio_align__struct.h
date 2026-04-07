// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pm_skills_interfaces:srv/IterativeGonioAlign.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__ITERATIVE_GONIO_ALIGN__STRUCT_H_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__ITERATIVE_GONIO_ALIGN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_alignment_frame'
// Member 'gonio_endeffector_frame'
// Member 'frames_to_measure'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/IterativeGonioAlign in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__IterativeGonioAlign_Request
{
  bool confocal_laser;
  rosidl_runtime_c__String target_alignment_frame;
  rosidl_runtime_c__String gonio_endeffector_frame;
  int32_t num_iterations;
  rosidl_runtime_c__String__Sequence frames_to_measure;
} pm_skills_interfaces__srv__IterativeGonioAlign_Request;

// Struct for a sequence of pm_skills_interfaces__srv__IterativeGonioAlign_Request.
typedef struct pm_skills_interfaces__srv__IterativeGonioAlign_Request__Sequence
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__IterativeGonioAlign_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/IterativeGonioAlign in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__IterativeGonioAlign_Response
{
  bool success;
  rosidl_runtime_c__String message;
} pm_skills_interfaces__srv__IterativeGonioAlign_Response;

// Struct for a sequence of pm_skills_interfaces__srv__IterativeGonioAlign_Response.
typedef struct pm_skills_interfaces__srv__IterativeGonioAlign_Response__Sequence
{
  pm_skills_interfaces__srv__IterativeGonioAlign_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__IterativeGonioAlign_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__ITERATIVE_GONIO_ALIGN__STRUCT_H_
