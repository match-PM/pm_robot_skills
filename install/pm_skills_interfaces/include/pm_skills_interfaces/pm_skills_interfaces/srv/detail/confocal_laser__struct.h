// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pm_skills_interfaces:srv/ConfocalLaser.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__CONFOCAL_LASER__STRUCT_H_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__CONFOCAL_LASER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'laser'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ConfocalLaser in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__ConfocalLaser_Request
{
  rosidl_runtime_c__String laser;
} pm_skills_interfaces__srv__ConfocalLaser_Request;

// Struct for a sequence of pm_skills_interfaces__srv__ConfocalLaser_Request.
typedef struct pm_skills_interfaces__srv__ConfocalLaser_Request__Sequence
{
  pm_skills_interfaces__srv__ConfocalLaser_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__ConfocalLaser_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ConfocalLaser in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__ConfocalLaser_Response
{
  bool success;
  rosidl_runtime_c__String result;
} pm_skills_interfaces__srv__ConfocalLaser_Response;

// Struct for a sequence of pm_skills_interfaces__srv__ConfocalLaser_Response.
typedef struct pm_skills_interfaces__srv__ConfocalLaser_Response__Sequence
{
  pm_skills_interfaces__srv__ConfocalLaser_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__ConfocalLaser_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__CONFOCAL_LASER__STRUCT_H_
