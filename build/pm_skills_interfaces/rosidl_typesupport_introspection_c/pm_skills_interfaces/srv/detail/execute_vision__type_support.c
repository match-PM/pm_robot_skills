// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pm_skills_interfaces:srv/ExecuteVision.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pm_skills_interfaces/srv/detail/execute_vision__rosidl_typesupport_introspection_c.h"
#include "pm_skills_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pm_skills_interfaces/srv/detail/execute_vision__functions.h"
#include "pm_skills_interfaces/srv/detail/execute_vision__struct.h"


// Include directives for member types
// Member `process_filename`
// Member `camera_config_filename`
// Member `process_uid`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pm_skills_interfaces__srv__ExecuteVision_Request__rosidl_typesupport_introspection_c__ExecuteVision_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pm_skills_interfaces__srv__ExecuteVision_Request__init(message_memory);
}

void pm_skills_interfaces__srv__ExecuteVision_Request__rosidl_typesupport_introspection_c__ExecuteVision_Request_fini_function(void * message_memory)
{
  pm_skills_interfaces__srv__ExecuteVision_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pm_skills_interfaces__srv__ExecuteVision_Request__rosidl_typesupport_introspection_c__ExecuteVision_Request_message_member_array[5] = {
  {
    "process_filename",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__ExecuteVision_Request, process_filename),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "camera_config_filename",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__ExecuteVision_Request, camera_config_filename),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "process_uid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__ExecuteVision_Request, process_uid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "image_display_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__ExecuteVision_Request, image_display_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "run_cross_validation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__ExecuteVision_Request, run_cross_validation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pm_skills_interfaces__srv__ExecuteVision_Request__rosidl_typesupport_introspection_c__ExecuteVision_Request_message_members = {
  "pm_skills_interfaces__srv",  // message namespace
  "ExecuteVision_Request",  // message name
  5,  // number of fields
  sizeof(pm_skills_interfaces__srv__ExecuteVision_Request),
  pm_skills_interfaces__srv__ExecuteVision_Request__rosidl_typesupport_introspection_c__ExecuteVision_Request_message_member_array,  // message members
  pm_skills_interfaces__srv__ExecuteVision_Request__rosidl_typesupport_introspection_c__ExecuteVision_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  pm_skills_interfaces__srv__ExecuteVision_Request__rosidl_typesupport_introspection_c__ExecuteVision_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pm_skills_interfaces__srv__ExecuteVision_Request__rosidl_typesupport_introspection_c__ExecuteVision_Request_message_type_support_handle = {
  0,
  &pm_skills_interfaces__srv__ExecuteVision_Request__rosidl_typesupport_introspection_c__ExecuteVision_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pm_skills_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pm_skills_interfaces, srv, ExecuteVision_Request)() {
  if (!pm_skills_interfaces__srv__ExecuteVision_Request__rosidl_typesupport_introspection_c__ExecuteVision_Request_message_type_support_handle.typesupport_identifier) {
    pm_skills_interfaces__srv__ExecuteVision_Request__rosidl_typesupport_introspection_c__ExecuteVision_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pm_skills_interfaces__srv__ExecuteVision_Request__rosidl_typesupport_introspection_c__ExecuteVision_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "pm_skills_interfaces/srv/detail/execute_vision__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pm_skills_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pm_skills_interfaces/srv/detail/execute_vision__functions.h"
// already included above
// #include "pm_skills_interfaces/srv/detail/execute_vision__struct.h"


// Include directives for member types
// Member `results_dict`
// Member `results_path`
// Member `process_uid`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `points`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__ExecuteVision_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pm_skills_interfaces__srv__ExecuteVision_Response__init(message_memory);
}

void pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__ExecuteVision_Response_fini_function(void * message_memory)
{
  pm_skills_interfaces__srv__ExecuteVision_Response__fini(message_memory);
}

size_t pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__size_function__ExecuteVision_Response__points(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__get_const_function__ExecuteVision_Response__points(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__get_function__ExecuteVision_Response__points(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__fetch_function__ExecuteVision_Response__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__get_const_function__ExecuteVision_Response__points(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__assign_function__ExecuteVision_Response__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__get_function__ExecuteVision_Response__points(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__resize_function__ExecuteVision_Response__points(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__ExecuteVision_Response_message_member_array[5] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__ExecuteVision_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "results_dict",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__ExecuteVision_Response, results_dict),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "results_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__ExecuteVision_Response, results_path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__ExecuteVision_Response, points),  // bytes offset in struct
    NULL,  // default value
    pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__size_function__ExecuteVision_Response__points,  // size() function pointer
    pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__get_const_function__ExecuteVision_Response__points,  // get_const(index) function pointer
    pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__get_function__ExecuteVision_Response__points,  // get(index) function pointer
    pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__fetch_function__ExecuteVision_Response__points,  // fetch(index, &value) function pointer
    pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__assign_function__ExecuteVision_Response__points,  // assign(index, value) function pointer
    pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__resize_function__ExecuteVision_Response__points  // resize(index) function pointer
  },
  {
    "process_uid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pm_skills_interfaces__srv__ExecuteVision_Response, process_uid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__ExecuteVision_Response_message_members = {
  "pm_skills_interfaces__srv",  // message namespace
  "ExecuteVision_Response",  // message name
  5,  // number of fields
  sizeof(pm_skills_interfaces__srv__ExecuteVision_Response),
  pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__ExecuteVision_Response_message_member_array,  // message members
  pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__ExecuteVision_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__ExecuteVision_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__ExecuteVision_Response_message_type_support_handle = {
  0,
  &pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__ExecuteVision_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pm_skills_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pm_skills_interfaces, srv, ExecuteVision_Response)() {
  if (!pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__ExecuteVision_Response_message_type_support_handle.typesupport_identifier) {
    pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__ExecuteVision_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pm_skills_interfaces__srv__ExecuteVision_Response__rosidl_typesupport_introspection_c__ExecuteVision_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "pm_skills_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "pm_skills_interfaces/srv/detail/execute_vision__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers pm_skills_interfaces__srv__detail__execute_vision__rosidl_typesupport_introspection_c__ExecuteVision_service_members = {
  "pm_skills_interfaces__srv",  // service namespace
  "ExecuteVision",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // pm_skills_interfaces__srv__detail__execute_vision__rosidl_typesupport_introspection_c__ExecuteVision_Request_message_type_support_handle,
  NULL  // response message
  // pm_skills_interfaces__srv__detail__execute_vision__rosidl_typesupport_introspection_c__ExecuteVision_Response_message_type_support_handle
};

static rosidl_service_type_support_t pm_skills_interfaces__srv__detail__execute_vision__rosidl_typesupport_introspection_c__ExecuteVision_service_type_support_handle = {
  0,
  &pm_skills_interfaces__srv__detail__execute_vision__rosidl_typesupport_introspection_c__ExecuteVision_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pm_skills_interfaces, srv, ExecuteVision_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pm_skills_interfaces, srv, ExecuteVision_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pm_skills_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pm_skills_interfaces, srv, ExecuteVision)() {
  if (!pm_skills_interfaces__srv__detail__execute_vision__rosidl_typesupport_introspection_c__ExecuteVision_service_type_support_handle.typesupport_identifier) {
    pm_skills_interfaces__srv__detail__execute_vision__rosidl_typesupport_introspection_c__ExecuteVision_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)pm_skills_interfaces__srv__detail__execute_vision__rosidl_typesupport_introspection_c__ExecuteVision_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pm_skills_interfaces, srv, ExecuteVision_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pm_skills_interfaces, srv, ExecuteVision_Response)()->data;
  }

  return &pm_skills_interfaces__srv__detail__execute_vision__rosidl_typesupport_introspection_c__ExecuteVision_service_type_support_handle;
}
