// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pm_skills_interfaces:srv/MeasureFrameLaser.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pm_skills_interfaces/srv/detail/measure_frame_laser__rosidl_typesupport_introspection_c.h"
#include "pm_skills_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pm_skills_interfaces/srv/detail/measure_frame_laser__functions.h"
#include "pm_skills_interfaces/srv/detail/measure_frame_laser__struct.h"


// Include directives for member types
// Member `frame_name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pm_skills_interfaces__srv__MeasureFrameLaser_Request__rosidl_typesupport_introspection_c__MeasureFrameLaser_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pm_skills_interfaces__srv__MeasureFrameLaser_Request__init(message_memory);
}

void pm_skills_interfaces__srv__MeasureFrameLaser_Request__rosidl_typesupport_introspection_c__MeasureFrameLaser_Request_fini_function(void * message_memory)
{
  pm_skills_interfaces__srv__MeasureFrameLaser_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pm_skills_interfaces__srv__MeasureFrameLaser_Request__rosidl_typesupport_introspection_c__MeasureFrameLaser_Request_message_member_array[1] = {
  {
    "frame_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__MeasureFrameLaser_Request, frame_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pm_skills_interfaces__srv__MeasureFrameLaser_Request__rosidl_typesupport_introspection_c__MeasureFrameLaser_Request_message_members = {
  "pm_skills_interfaces__srv",  // message namespace
  "MeasureFrameLaser_Request",  // message name
  1,  // number of fields
  sizeof(pm_skills_interfaces__srv__MeasureFrameLaser_Request),
  pm_skills_interfaces__srv__MeasureFrameLaser_Request__rosidl_typesupport_introspection_c__MeasureFrameLaser_Request_message_member_array,  // message members
  pm_skills_interfaces__srv__MeasureFrameLaser_Request__rosidl_typesupport_introspection_c__MeasureFrameLaser_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  pm_skills_interfaces__srv__MeasureFrameLaser_Request__rosidl_typesupport_introspection_c__MeasureFrameLaser_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pm_skills_interfaces__srv__MeasureFrameLaser_Request__rosidl_typesupport_introspection_c__MeasureFrameLaser_Request_message_type_support_handle = {
  0,
  &pm_skills_interfaces__srv__MeasureFrameLaser_Request__rosidl_typesupport_introspection_c__MeasureFrameLaser_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pm_skills_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pm_skills_interfaces, srv, MeasureFrameLaser_Request)() {
  if (!pm_skills_interfaces__srv__MeasureFrameLaser_Request__rosidl_typesupport_introspection_c__MeasureFrameLaser_Request_message_type_support_handle.typesupport_identifier) {
    pm_skills_interfaces__srv__MeasureFrameLaser_Request__rosidl_typesupport_introspection_c__MeasureFrameLaser_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pm_skills_interfaces__srv__MeasureFrameLaser_Request__rosidl_typesupport_introspection_c__MeasureFrameLaser_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "pm_skills_interfaces/srv/detail/measure_frame_laser__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pm_skills_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pm_skills_interfaces/srv/detail/measure_frame_laser__functions.h"
// already included above
// #include "pm_skills_interfaces/srv/detail/measure_frame_laser__struct.h"


// Include directives for member types
// Member `result_vector`
#include "geometry_msgs/msg/vector3.h"
// Member `result_vector`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"
// Member `message`
// Member `component_name`
// Member `component_uuid`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pm_skills_interfaces__srv__MeasureFrameLaser_Response__rosidl_typesupport_introspection_c__MeasureFrameLaser_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pm_skills_interfaces__srv__MeasureFrameLaser_Response__init(message_memory);
}

void pm_skills_interfaces__srv__MeasureFrameLaser_Response__rosidl_typesupport_introspection_c__MeasureFrameLaser_Response_fini_function(void * message_memory)
{
  pm_skills_interfaces__srv__MeasureFrameLaser_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pm_skills_interfaces__srv__MeasureFrameLaser_Response__rosidl_typesupport_introspection_c__MeasureFrameLaser_Response_message_member_array[5] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__MeasureFrameLaser_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result_vector",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__MeasureFrameLaser_Response, result_vector),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__MeasureFrameLaser_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "component_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__MeasureFrameLaser_Response, component_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "component_uuid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__MeasureFrameLaser_Response, component_uuid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pm_skills_interfaces__srv__MeasureFrameLaser_Response__rosidl_typesupport_introspection_c__MeasureFrameLaser_Response_message_members = {
  "pm_skills_interfaces__srv",  // message namespace
  "MeasureFrameLaser_Response",  // message name
  5,  // number of fields
  sizeof(pm_skills_interfaces__srv__MeasureFrameLaser_Response),
  pm_skills_interfaces__srv__MeasureFrameLaser_Response__rosidl_typesupport_introspection_c__MeasureFrameLaser_Response_message_member_array,  // message members
  pm_skills_interfaces__srv__MeasureFrameLaser_Response__rosidl_typesupport_introspection_c__MeasureFrameLaser_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  pm_skills_interfaces__srv__MeasureFrameLaser_Response__rosidl_typesupport_introspection_c__MeasureFrameLaser_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pm_skills_interfaces__srv__MeasureFrameLaser_Response__rosidl_typesupport_introspection_c__MeasureFrameLaser_Response_message_type_support_handle = {
  0,
  &pm_skills_interfaces__srv__MeasureFrameLaser_Response__rosidl_typesupport_introspection_c__MeasureFrameLaser_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pm_skills_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pm_skills_interfaces, srv, MeasureFrameLaser_Response)() {
  pm_skills_interfaces__srv__MeasureFrameLaser_Response__rosidl_typesupport_introspection_c__MeasureFrameLaser_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!pm_skills_interfaces__srv__MeasureFrameLaser_Response__rosidl_typesupport_introspection_c__MeasureFrameLaser_Response_message_type_support_handle.typesupport_identifier) {
    pm_skills_interfaces__srv__MeasureFrameLaser_Response__rosidl_typesupport_introspection_c__MeasureFrameLaser_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pm_skills_interfaces__srv__MeasureFrameLaser_Response__rosidl_typesupport_introspection_c__MeasureFrameLaser_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "pm_skills_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "pm_skills_interfaces/srv/detail/measure_frame_laser__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers pm_skills_interfaces__srv__detail__measure_frame_laser__rosidl_typesupport_introspection_c__MeasureFrameLaser_service_members = {
  "pm_skills_interfaces__srv",  // service namespace
  "MeasureFrameLaser",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // pm_skills_interfaces__srv__detail__measure_frame_laser__rosidl_typesupport_introspection_c__MeasureFrameLaser_Request_message_type_support_handle,
  NULL  // response message
  // pm_skills_interfaces__srv__detail__measure_frame_laser__rosidl_typesupport_introspection_c__MeasureFrameLaser_Response_message_type_support_handle
};

static rosidl_service_type_support_t pm_skills_interfaces__srv__detail__measure_frame_laser__rosidl_typesupport_introspection_c__MeasureFrameLaser_service_type_support_handle = {
  0,
  &pm_skills_interfaces__srv__detail__measure_frame_laser__rosidl_typesupport_introspection_c__MeasureFrameLaser_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pm_skills_interfaces, srv, MeasureFrameLaser_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pm_skills_interfaces, srv, MeasureFrameLaser_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pm_skills_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pm_skills_interfaces, srv, MeasureFrameLaser)() {
  if (!pm_skills_interfaces__srv__detail__measure_frame_laser__rosidl_typesupport_introspection_c__MeasureFrameLaser_service_type_support_handle.typesupport_identifier) {
    pm_skills_interfaces__srv__detail__measure_frame_laser__rosidl_typesupport_introspection_c__MeasureFrameLaser_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)pm_skills_interfaces__srv__detail__measure_frame_laser__rosidl_typesupport_introspection_c__MeasureFrameLaser_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pm_skills_interfaces, srv, MeasureFrameLaser_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pm_skills_interfaces, srv, MeasureFrameLaser_Response)()->data;
  }

  return &pm_skills_interfaces__srv__detail__measure_frame_laser__rosidl_typesupport_introspection_c__MeasureFrameLaser_service_type_support_handle;
}
