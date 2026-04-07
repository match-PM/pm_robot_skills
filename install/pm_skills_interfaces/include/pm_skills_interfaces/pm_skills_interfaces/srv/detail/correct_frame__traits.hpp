// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pm_skills_interfaces:srv/CorrectFrame.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME__TRAITS_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pm_skills_interfaces/srv/detail/correct_frame__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pm_skills_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CorrectFrame_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CorrectFrame_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CorrectFrame_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pm_skills_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use pm_skills_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pm_skills_interfaces::srv::CorrectFrame_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pm_skills_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pm_skills_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pm_skills_interfaces::srv::CorrectFrame_Request & msg)
{
  return pm_skills_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pm_skills_interfaces::srv::CorrectFrame_Request>()
{
  return "pm_skills_interfaces::srv::CorrectFrame_Request";
}

template<>
inline const char * name<pm_skills_interfaces::srv::CorrectFrame_Request>()
{
  return "pm_skills_interfaces/srv/CorrectFrame_Request";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::CorrectFrame_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::CorrectFrame_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pm_skills_interfaces::srv::CorrectFrame_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'correction_values'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace pm_skills_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CorrectFrame_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: correction_values
  {
    out << "correction_values: ";
    to_flow_style_yaml(msg.correction_values, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CorrectFrame_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: correction_values
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "correction_values:\n";
    to_block_style_yaml(msg.correction_values, out, indentation + 2);
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CorrectFrame_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pm_skills_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use pm_skills_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pm_skills_interfaces::srv::CorrectFrame_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pm_skills_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pm_skills_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pm_skills_interfaces::srv::CorrectFrame_Response & msg)
{
  return pm_skills_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pm_skills_interfaces::srv::CorrectFrame_Response>()
{
  return "pm_skills_interfaces::srv::CorrectFrame_Response";
}

template<>
inline const char * name<pm_skills_interfaces::srv::CorrectFrame_Response>()
{
  return "pm_skills_interfaces/srv/CorrectFrame_Response";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::CorrectFrame_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::CorrectFrame_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pm_skills_interfaces::srv::CorrectFrame_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pm_skills_interfaces::srv::CorrectFrame>()
{
  return "pm_skills_interfaces::srv::CorrectFrame";
}

template<>
inline const char * name<pm_skills_interfaces::srv::CorrectFrame>()
{
  return "pm_skills_interfaces/srv/CorrectFrame";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::CorrectFrame>
  : std::integral_constant<
    bool,
    has_fixed_size<pm_skills_interfaces::srv::CorrectFrame_Request>::value &&
    has_fixed_size<pm_skills_interfaces::srv::CorrectFrame_Response>::value
  >
{
};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::CorrectFrame>
  : std::integral_constant<
    bool,
    has_bounded_size<pm_skills_interfaces::srv::CorrectFrame_Request>::value &&
    has_bounded_size<pm_skills_interfaces::srv::CorrectFrame_Response>::value
  >
{
};

template<>
struct is_service<pm_skills_interfaces::srv::CorrectFrame>
  : std::true_type
{
};

template<>
struct is_service_request<pm_skills_interfaces::srv::CorrectFrame_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pm_skills_interfaces::srv::CorrectFrame_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME__TRAITS_HPP_
