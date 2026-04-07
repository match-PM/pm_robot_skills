// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pm_skills_interfaces:srv/PlaceComponent.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__PLACE_COMPONENT__STRUCT_H_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__PLACE_COMPONENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/PlaceComponent in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__PlaceComponent_Request
{
  bool align_orientation;
  float x_offset_um;
  float y_offset_um;
  float z_offset_um;
  float rx_offset_deg;
  float ry_offset_deg;
  float rz_offset_deg;
} pm_skills_interfaces__srv__PlaceComponent_Request;

// Struct for a sequence of pm_skills_interfaces__srv__PlaceComponent_Request.
typedef struct pm_skills_interfaces__srv__PlaceComponent_Request__Sequence
{
  pm_skills_interfaces__srv__PlaceComponent_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__PlaceComponent_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PlaceComponent in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__PlaceComponent_Response
{
  bool success;
  rosidl_runtime_c__String message;
} pm_skills_interfaces__srv__PlaceComponent_Response;

// Struct for a sequence of pm_skills_interfaces__srv__PlaceComponent_Response.
typedef struct pm_skills_interfaces__srv__PlaceComponent_Response__Sequence
{
  pm_skills_interfaces__srv__PlaceComponent_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__PlaceComponent_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__PLACE_COMPONENT__STRUCT_H_
