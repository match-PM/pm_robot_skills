// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from pm_skills_interfaces:srv/DispenseAdhesive.idl
// generated code does not contain a copyright notice
#include "pm_skills_interfaces/srv/detail/dispense_adhesive__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "pm_skills_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "pm_skills_interfaces/srv/detail/dispense_adhesive__struct.h"
#include "pm_skills_interfaces/srv/detail/dispense_adhesive__functions.h"
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

#include "rosidl_runtime_c/string.h"  // dispenser
#include "rosidl_runtime_c/string_functions.h"  // dispenser

// forward declare type support functions


using _DispenseAdhesive_Request__ros_msg_type = pm_skills_interfaces__srv__DispenseAdhesive_Request;

static bool _DispenseAdhesive_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _DispenseAdhesive_Request__ros_msg_type * ros_message = static_cast<const _DispenseAdhesive_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: dispenser
  {
    const rosidl_runtime_c__String * str = &ros_message->dispenser;
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

  // Field name: dispense_time
  {
    cdr << ros_message->dispense_time;
  }

  return true;
}

static bool _DispenseAdhesive_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _DispenseAdhesive_Request__ros_msg_type * ros_message = static_cast<_DispenseAdhesive_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: dispenser
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->dispenser.data) {
      rosidl_runtime_c__String__init(&ros_message->dispenser);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->dispenser,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'dispenser'\n");
      return false;
    }
  }

  // Field name: dispense_time
  {
    cdr >> ros_message->dispense_time;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pm_skills_interfaces
size_t get_serialized_size_pm_skills_interfaces__srv__DispenseAdhesive_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DispenseAdhesive_Request__ros_msg_type * ros_message = static_cast<const _DispenseAdhesive_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name dispenser
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->dispenser.size + 1);
  // field.name dispense_time
  {
    size_t item_size = sizeof(ros_message->dispense_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _DispenseAdhesive_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_pm_skills_interfaces__srv__DispenseAdhesive_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pm_skills_interfaces
size_t max_serialized_size_pm_skills_interfaces__srv__DispenseAdhesive_Request(
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

  // member: dispenser
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
  // member: dispense_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = pm_skills_interfaces__srv__DispenseAdhesive_Request;
    is_plain =
      (
      offsetof(DataType, dispense_time) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _DispenseAdhesive_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_pm_skills_interfaces__srv__DispenseAdhesive_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_DispenseAdhesive_Request = {
  "pm_skills_interfaces::srv",
  "DispenseAdhesive_Request",
  _DispenseAdhesive_Request__cdr_serialize,
  _DispenseAdhesive_Request__cdr_deserialize,
  _DispenseAdhesive_Request__get_serialized_size,
  _DispenseAdhesive_Request__max_serialized_size
};

static rosidl_message_type_support_t _DispenseAdhesive_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_DispenseAdhesive_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, DispenseAdhesive_Request)() {
  return &_DispenseAdhesive_Request__type_support;
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
// #include "pm_skills_interfaces/srv/detail/dispense_adhesive__struct.h"
// already included above
// #include "pm_skills_interfaces/srv/detail/dispense_adhesive__functions.h"
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


using _DispenseAdhesive_Response__ros_msg_type = pm_skills_interfaces__srv__DispenseAdhesive_Response;

static bool _DispenseAdhesive_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _DispenseAdhesive_Response__ros_msg_type * ros_message = static_cast<const _DispenseAdhesive_Response__ros_msg_type *>(untyped_ros_message);
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

static bool _DispenseAdhesive_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _DispenseAdhesive_Response__ros_msg_type * ros_message = static_cast<_DispenseAdhesive_Response__ros_msg_type *>(untyped_ros_message);
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
size_t get_serialized_size_pm_skills_interfaces__srv__DispenseAdhesive_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DispenseAdhesive_Response__ros_msg_type * ros_message = static_cast<const _DispenseAdhesive_Response__ros_msg_type *>(untyped_ros_message);
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

static uint32_t _DispenseAdhesive_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_pm_skills_interfaces__srv__DispenseAdhesive_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pm_skills_interfaces
size_t max_serialized_size_pm_skills_interfaces__srv__DispenseAdhesive_Response(
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
    using DataType = pm_skills_interfaces__srv__DispenseAdhesive_Response;
    is_plain =
      (
      offsetof(DataType, message) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _DispenseAdhesive_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_pm_skills_interfaces__srv__DispenseAdhesive_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_DispenseAdhesive_Response = {
  "pm_skills_interfaces::srv",
  "DispenseAdhesive_Response",
  _DispenseAdhesive_Response__cdr_serialize,
  _DispenseAdhesive_Response__cdr_deserialize,
  _DispenseAdhesive_Response__get_serialized_size,
  _DispenseAdhesive_Response__max_serialized_size
};

static rosidl_message_type_support_t _DispenseAdhesive_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_DispenseAdhesive_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, DispenseAdhesive_Response)() {
  return &_DispenseAdhesive_Response__type_support;
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
#include "pm_skills_interfaces/srv/dispense_adhesive.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t DispenseAdhesive__callbacks = {
  "pm_skills_interfaces::srv",
  "DispenseAdhesive",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, DispenseAdhesive_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, DispenseAdhesive_Response)(),
};

static rosidl_service_type_support_t DispenseAdhesive__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &DispenseAdhesive__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pm_skills_interfaces, srv, DispenseAdhesive)() {
  return &DispenseAdhesive__handle;
}

#if defined(__cplusplus)
}
#endif
