// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from pm_skills_interfaces:srv/IterativeGonioAlign.idl
// generated code does not contain a copyright notice
#include "pm_skills_interfaces/srv/detail/iterative_gonio_align__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "pm_skills_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "pm_skills_interfaces/srv/detail/iterative_gonio_align__struct.h"
#include "pm_skills_interfaces/srv/detail/iterative_gonio_align__functions.h"
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

#include "rosidl_runtime_c/string.h"  // frames_to_measure, gonio_endeffector_frame, target_alignment_frame
#include "rosidl_runtime_c/string_functions.h"  // frames_to_measure, gonio_endeffector_frame, target_alignment_frame

// forward declare type support functions


using _IterativeGonioAlign_Request__ros_msg_type = pm_skills_interfaces__srv__IterativeGonioAlign_Request;

static bool _IterativeGonioAlign_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _IterativeGonioAlign_Request__ros_msg_type * ros_message = static_cast<const _IterativeGonioAlign_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: confocal_laser
  {
    cdr << (ros_message->confocal_laser ? true : false);
  }

  // Field name: target_alignment_frame
  {
    const rosidl_runtime_c__String * str = &ros_message->target_alignment_frame;
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

  // Field name: gonio_endeffector_frame
  {
    const rosidl_runtime_c__String * str = &ros_message->gonio_endeffector_frame;
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

  // Field name: num_iterations
  {
    cdr << ros_message->num_iterations;
  }

  // Field name: frames_to_measure
  {
    size_t size = ros_message->frames_to_measure.size;
    auto array_ptr = ros_message->frames_to_measure.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
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
  }

  return true;
}

static bool _IterativeGonioAlign_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _IterativeGonioAlign_Request__ros_msg_type * ros_message = static_cast<_IterativeGonioAlign_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: confocal_laser
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->confocal_laser = tmp ? true : false;
  }

  // Field name: target_alignment_frame
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->target_alignment_frame.data) {
      rosidl_runtime_c__String__init(&ros_message->target_alignment_frame);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->target_alignment_frame,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'target_alignment_frame'\n");
      return false;
    }
  }

  // Field name: gonio_endeffector_frame
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->gonio_endeffector_frame.data) {
      rosidl_runtime_c__String__init(&ros_message->gonio_endeffector_frame);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->gonio_endeffector_frame,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'gonio_endeffector_frame'\n");
      return false;
    }
  }

  // Field name: num_iterations
  {
    cdr >> ros_message->num_iterations;
  }

  // Field name: frames_to_measure
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->frames_to_measure.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->frames_to_measure);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->frames_to_measure, size)) {
      fprintf(stderr, "failed to create array for field 'frames_to_measure'");
      return false;
    }
    auto array_ptr = ros_message->frames_to_measure.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'frames_to_measure'\n");
        return false;
      }
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pm_skills_interfaces
size_t get_serialized_size_pm_skills_interfaces__srv__IterativeGonioAlign_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _IterativeGonioAlign_Request__ros_msg_type * ros_message = static_cast<const _IterativeGonioAlign_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name confocal_laser
  {
    size_t item_size = sizeof(ros_message->confocal_laser);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name target_alignment_frame
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->target_alignment_frame.size + 1);
  // field.name gonio_endeffector_frame
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->gonio_endeffector_frame.size + 1);
  // field.name num_iterations
  {
    size_t item_size = sizeof(ros_message->num_iterations);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name frames_to_measure
  {
    size_t array_size = ros_message->frames_to_measure.size;
    auto array_ptr = ros_message->frames_to_measure.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _IterativeGonioAlign_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_pm_skills_interfaces__srv__IterativeGonioAlign_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pm_skills_interfaces
size_t max_serialized_size_pm_skills_interfaces__srv__IterativeGonioAlign_Request(
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

  // member: confocal_laser
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: target_alignment_frame
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
  // member: gonio_endeffector_frame
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
  // member: num_iterations
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: frames_to_measure
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

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
    using DataType = pm_skills_interfaces__srv__IterativeGonioAlign_Request;
    is_plain =
      (
      offsetof(DataType, frames_to_measure) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _IterativeGonioAlign_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_pm_skills_interfaces__srv__IterativeGonioAlign_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_IterativeGonioAlign_Request = {
  "pm_skills_interfaces::srv",
  "IterativeGonioAlign_Request",
  _IterativeGonioAlign_Request__cdr_serialize,
  _IterativeGonioAlign_Request__cdr_deserialize,
  _IterativeGonioAlign_Request__get_serialized_size,
  _IterativeGonioAlign_Request__max_serialized_size
};

static rosidl_message_type_support_t _IterativeGonioAlign_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_IterativeGonioAlign_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, IterativeGonioAlign_Request)() {
  return &_IterativeGonioAlign_Request__type_support;
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
// #include "pm_skills_interfaces/srv/detail/iterative_gonio_align__struct.h"
// already included above
// #include "pm_skills_interfaces/srv/detail/iterative_gonio_align__functions.h"
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

// already included above
// #include "rosidl_runtime_c/string.h"  // message
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // message

// forward declare type support functions


using _IterativeGonioAlign_Response__ros_msg_type = pm_skills_interfaces__srv__IterativeGonioAlign_Response;

static bool _IterativeGonioAlign_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _IterativeGonioAlign_Response__ros_msg_type * ros_message = static_cast<const _IterativeGonioAlign_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
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

  return true;
}

static bool _IterativeGonioAlign_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _IterativeGonioAlign_Response__ros_msg_type * ros_message = static_cast<_IterativeGonioAlign_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
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

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pm_skills_interfaces
size_t get_serialized_size_pm_skills_interfaces__srv__IterativeGonioAlign_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _IterativeGonioAlign_Response__ros_msg_type * ros_message = static_cast<const _IterativeGonioAlign_Response__ros_msg_type *>(untyped_ros_message);
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
  // field.name message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _IterativeGonioAlign_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_pm_skills_interfaces__srv__IterativeGonioAlign_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pm_skills_interfaces
size_t max_serialized_size_pm_skills_interfaces__srv__IterativeGonioAlign_Response(
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

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = pm_skills_interfaces__srv__IterativeGonioAlign_Response;
    is_plain =
      (
      offsetof(DataType, message) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _IterativeGonioAlign_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_pm_skills_interfaces__srv__IterativeGonioAlign_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_IterativeGonioAlign_Response = {
  "pm_skills_interfaces::srv",
  "IterativeGonioAlign_Response",
  _IterativeGonioAlign_Response__cdr_serialize,
  _IterativeGonioAlign_Response__cdr_deserialize,
  _IterativeGonioAlign_Response__get_serialized_size,
  _IterativeGonioAlign_Response__max_serialized_size
};

static rosidl_message_type_support_t _IterativeGonioAlign_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_IterativeGonioAlign_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, IterativeGonioAlign_Response)() {
  return &_IterativeGonioAlign_Response__type_support;
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
#include "pm_skills_interfaces/srv/iterative_gonio_align.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t IterativeGonioAlign__callbacks = {
  "pm_skills_interfaces::srv",
  "IterativeGonioAlign",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, IterativeGonioAlign_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, IterativeGonioAlign_Response)(),
};

static rosidl_service_type_support_t IterativeGonioAlign__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &IterativeGonioAlign__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, IterativeGonioAlign)() {
  return &IterativeGonioAlign__handle;
}

#if defined(__cplusplus)
}
#endif
