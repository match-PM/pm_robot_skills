// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pm_skills_interfaces:srv/CheckFrameMeasurable.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__CHECK_FRAME_MEASURABLE__TRAITS_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__CHECK_FRAME_MEASURABLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pm_skills_interfaces/srv/detail/check_frame_measurable__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pm_skills_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CheckFrameMeasurable_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: frame_name
  {
    out << "frame_name: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CheckFrameMeasurable_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: frame_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_name: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CheckFrameMeasurable_Request & msg, bool use_flow_style = false)
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
  const pm_skills_interfaces::srv::CheckFrameMeasurable_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pm_skills_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pm_skills_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pm_skills_interfaces::srv::CheckFrameMeasurable_Request & msg)
{
  return pm_skills_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pm_skills_interfaces::srv::CheckFrameMeasurable_Request>()
{
  return "pm_skills_interfaces::srv::CheckFrameMeasurable_Request";
}

template<>
inline const char * name<pm_skills_interfaces::srv::CheckFrameMeasurable_Request>()
{
  return "pm_skills_interfaces/srv/CheckFrameMeasurable_Request";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::CheckFrameMeasurable_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::CheckFrameMeasurable_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pm_skills_interfaces::srv::CheckFrameMeasurable_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace pm_skills_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CheckFrameMeasurable_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
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
  const CheckFrameMeasurable_Response & msg,
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

inline std::string to_yaml(const CheckFrameMeasurable_Response & msg, bool use_flow_style = false)
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
  const pm_skills_interfaces::srv::CheckFrameMeasurable_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pm_skills_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pm_skills_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pm_skills_interfaces::srv::CheckFrameMeasurable_Response & msg)
{
  return pm_skills_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pm_skills_interfaces::srv::CheckFrameMeasurable_Response>()
{
  return "pm_skills_interfaces::srv::CheckFrameMeasurable_Response";
}

template<>
inline const char * name<pm_skills_interfaces::srv::CheckFrameMeasurable_Response>()
{
  return "pm_skills_interfaces/srv/CheckFrameMeasurable_Response";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::CheckFrameMeasurable_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::CheckFrameMeasurable_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pm_skills_interfaces::srv::CheckFrameMeasurable_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pm_skills_interfaces::srv::CheckFrameMeasurable>()
{
  return "pm_skills_interfaces::srv::CheckFrameMeasurable";
}

template<>
inline const char * name<pm_skills_interfaces::srv::CheckFrameMeasurable>()
{
  return "pm_skills_interfaces/srv/CheckFrameMeasurable";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::CheckFrameMeasurable>
  : std::integral_constant<
    bool,
    has_fixed_size<pm_skills_interfaces::srv::CheckFrameMeasurable_Request>::value &&
    has_fixed_size<pm_skills_interfaces::srv::CheckFrameMeasurable_Response>::value
  >
{
};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::CheckFrameMeasurable>
  : std::integral_constant<
    bool,
    has_bounded_size<pm_skills_interfaces::srv::CheckFrameMeasurable_Request>::value &&
    has_bounded_size<pm_skills_interfaces::srv::CheckFrameMeasurable_Response>::value
  >
{
};

template<>
struct is_service<pm_skills_interfaces::srv::CheckFrameMeasurable>
  : std::true_type
{
};

template<>
struct is_service_request<pm_skills_interfaces::srv::CheckFrameMeasurable_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pm_skills_interfaces::srv::CheckFrameMeasurable_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__CHECK_FRAME_MEASURABLE__TRAITS_HPP_
