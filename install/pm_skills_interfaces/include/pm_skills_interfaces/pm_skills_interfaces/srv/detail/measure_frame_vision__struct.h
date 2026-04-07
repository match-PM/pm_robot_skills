// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pm_skills_interfaces:srv/MeasureFrameVision.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__MEASURE_FRAME_VISION__STRUCT_H_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__MEASURE_FRAME_VISION__STRUCT_H_

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
// Member 'vision_process_file_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MeasureFrameVision in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__MeasureFrameVision_Request
{
  rosidl_runtime_c__String frame_name;
  rosidl_runtime_c__String vision_process_file_name;
} pm_skills_interfaces__srv__MeasureFrameVision_Request;

// Struct for a sequence of pm_skills_interfaces__srv__MeasureFrameVision_Request.
typedef struct pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence
{
  pm_skills_interfaces__srv__MeasureFrameVision_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result_vector'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'message'
// Member 'component_name'
// Member 'component_uuid'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'vision_response'
#include "pm_vision_interfaces/msg/detail/vision_response__struct.h"

/// Struct defined in srv/MeasureFrameVision in the package pm_skills_interfaces.
typedef struct pm_skills_interfaces__srv__MeasureFrameVision_Response
{
  bool success;
  geometry_msgs__msg__Vector3 result_vector;
  rosidl_runtime_c__String message;
  pm_vision_interfaces__msg__VisionResponse vision_response;
  rosidl_runtime_c__String component_name;
  rosidl_runtime_c__String component_uuid;
} pm_skills_interfaces__srv__MeasureFrameVision_Response;

// Struct for a sequence of pm_skills_interfaces__srv__MeasureFrameVision_Response.
typedef struct pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence
{
  pm_skills_interfaces__srv__MeasureFrameVision_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__MEASURE_FRAME_VISION__STRUCT_H_
