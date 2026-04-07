// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from pm_skills_interfaces:srv/CorrectFrameVision.idl
// generated code does not contain a copyright notice
#include "pm_skills_interfaces/srv/detail/correct_frame_vision__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "pm_skills_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "pm_skills_interfaces/srv/detail/correct_frame_vision__struct.h"
#include "pm_skills_interfaces/srv/detail/correct_frame_vision__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // frame_name
#include "rosidl_runtime_c/string_functions.h"  // frame_name

// forward declare type support functions


using _CorrectFrameVision_Request__ros_msg_type = pm_skills_interfaces__srv__CorrectFrameVision_Request;

static bool _CorrectFrameVision_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CorrectFrameVision_Request__ros_msg_type * ros_message = static_cast<const _CorrectFrameVision_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: frame_name
  {
    const rosidl_runtime_c__String * str = &ros_message->frame_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: remeasure_after_correction
  {
    cdr << (ros_message->remeasure_after_correction ? true : false);
  }

  return true;
}

static bool _CorrectFrameVision_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CorrectFrameVision_Request__ros_msg_type * ros_message = static_cast<_CorrectFrameVision_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: frame_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->frame_name.data) {
      rosidl_runtime_c__String__init(&ros_message->frame_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->frame_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'frame_name'\n");
      return false;
    }
  }

  // Field name: remeasure_after_correction
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->remeasure_after_correction = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pm_skills_interfaces
size_t get_serialized_size_pm_skills_interfaces__srv__CorrectFrameVision_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CorrectFrameVision_Request__ros_msg_type * ros_message = static_cast<const _CorrectFrameVision_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name frame_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->frame_name.size + 1);
  // field.name remeasure_after_correction
  {
    size_t item_size = sizeof(ros_message->remeasure_after_correction);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _CorrectFrameVision_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_pm_skills_interfaces__srv__CorrectFrameVision_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pm_skills_interfaces
size_t max_serialized_size_pm_skills_interfaces__srv__CorrectFrameVision_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: frame_name
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: remeasure_after_correction
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = pm_skills_interfaces__srv__CorrectFrameVision_Request;
    is_plain =
      (
      offsetof(DataType, remeasure_after_correction) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _CorrectFrameVision_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_pm_skills_interfaces__srv__CorrectFrameVision_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_CorrectFrameVision_Request = {
  "pm_skills_interfaces::srv",
  "CorrectFrameVision_Request",
  _CorrectFrameVision_Request__cdr_serialize,
  _CorrectFrameVision_Request__cdr_deserialize,
  _CorrectFrameVision_Request__get_serialized_size,
  _CorrectFrameVision_Request__max_serialized_size
};

static rosidl_message_type_support_t _CorrectFrameVision_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CorrectFrameVision_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, CorrectFrameVision_Request)() {
  return &_CorrectFrameVision_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "pm_skills_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "pm_skills_interfaces/srv/detail/correct_frame_vision__struct.h"
// already included above
// #include "pm_skills_interfaces/srv/detail/correct_frame_vision__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/vector3__functions.h"  // correction_values
#include "pm_vision_interfaces/msg/detail/vision_response__functions.h"  // vision_response
// already included above
// #include "rosidl_runtime_c/string.h"  // component_name, component_uuid, message
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // component_name, component_uuid, message

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pm_skills_interfaces
size_t get_serialized_size_geometry_msgs__msg__Vector3(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pm_skills_interfaces
size_t max_serialized_size_geometry_msgs__msg__Vector3(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pm_skills_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pm_skills_interfaces
size_t get_serialized_size_pm_vision_interfaces__msg__VisionResponse(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pm_skills_interfaces
size_t max_serialized_size_pm_vision_interfaces__msg__VisionResponse(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pm_skills_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_vision_interfaces, msg, VisionResponse)();


using _CorrectFrameVision_Response__ros_msg_type = pm_skills_interfaces__srv__CorrectFrameVision_Response;

static bool _CorrectFrameVision_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CorrectFrameVision_Response__ros_msg_type * ros_message = static_cast<const _CorrectFrameVision_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: correction_values
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->correction_values, cdr))
    {
      return false;
    }
  }

  // Field name: message
  {
    const rosidl_runtime_c__String * str = &ros_message->message;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: vision_response
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, pm_vision_interfaces, msg, VisionResponse
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->vision_response, cdr))
    {
      return false;
    }
  }

  // Field name: component_name
  {
    const rosidl_runtime_c__String * str = &ros_message->component_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: component_uuid
  {
    const rosidl_runtime_c__String * str = &ros_message->component_uuid;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _CorrectFrameVision_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CorrectFrameVision_Response__ros_msg_type * ros_message = static_cast<_CorrectFrameVision_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: correction_values
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->correction_values))
    {
      return false;
    }
  }

  // Field name: message
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->message.data) {
      rosidl_runtime_c__String__init(&ros_message->message);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->message,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'message'\n");
      return false;
    }
  }

  // Field name: vision_response
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, pm_vision_interfaces, msg, VisionResponse
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->vision_response))
    {
      return false;
    }
  }

  // Field name: component_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->component_name.data) {
      rosidl_runtime_c__String__init(&ros_message->component_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->component_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'component_name'\n");
      return false;
    }
  }

  // Field name: component_uuid
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->component_uuid.data) {
      rosidl_runtime_c__String__init(&ros_message->component_uuid);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->component_uuid,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'component_uuid'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pm_skills_interfaces
size_t get_serialized_size_pm_skills_interfaces__srv__CorrectFrameVision_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CorrectFrameVision_Response__ros_msg_type * ros_message = static_cast<const _CorrectFrameVision_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name correction_values

  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->correction_values), current_alignment);
  // field.name message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message.size + 1);
  // field.name vision_response

  current_alignment += get_serialized_size_pm_vision_interfaces__msg__VisionResponse(
    &(ros_message->vision_response), current_alignment);
  // field.name component_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->component_name.size + 1);
  // field.name component_uuid
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->component_uuid.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _CorrectFrameVision_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_pm_skills_interfaces__srv__CorrectFrameVision_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pm_skills_interfaces
size_t max_serialized_size_pm_skills_interfaces__srv__CorrectFrameVision_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: correction_values
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: message
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: vision_response
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_pm_vision_interfaces__msg__VisionResponse(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: component_name
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: component_uuid
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = pm_skills_interfaces__srv__CorrectFrameVision_Response;
    is_plain =
      (
      offsetof(DataType, component_uuid) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _CorrectFrameVision_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_pm_skills_interfaces__srv__CorrectFrameVision_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_CorrectFrameVision_Response = {
  "pm_skills_interfaces::srv",
  "CorrectFrameVision_Response",
  _CorrectFrameVision_Response__cdr_serialize,
  _CorrectFrameVision_Response__cdr_deserialize,
  _CorrectFrameVision_Response__get_serialized_size,
  _CorrectFrameVision_Response__max_serialized_size
};

static rosidl_message_type_support_t _CorrectFrameVision_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CorrectFrameVision_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, CorrectFrameVision_Response)() {
  return &_CorrectFrameVision_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "pm_skills_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "pm_skills_interfaces/srv/correct_frame_vision.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t CorrectFrameVision__callbacks = {
  "pm_skills_interfaces::srv",
  "CorrectFrameVision",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, CorrectFrameVision_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, CorrectFrameVision_Response)(),
};

static rosidl_service_type_support_t CorrectFrameVision__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &CorrectFrameVision__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, CorrectFrameVision)() {
  return &CorrectFrameVision__handle;
}

#if defined(__cplusplus)
}
#endif
