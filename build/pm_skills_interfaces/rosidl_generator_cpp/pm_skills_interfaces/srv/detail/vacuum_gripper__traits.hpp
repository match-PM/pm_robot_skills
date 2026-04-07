// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pm_skills_interfaces:srv/VacuumGripper.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__VACUUM_GRIPPER__TRAITS_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__VACUUM_GRIPPER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pm_skills_interfaces/srv/detail/vacuum_gripper__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pm_skills_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const VacuumGripper_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: activate_vacuum
  {
    out << "activate_vacuum: ";
    rosidl_generator_traits::value_to_yaml(msg.activate_vacuum, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VacuumGripper_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: activate_vacuum
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "activate_vacuum: ";
    rosidl_generator_traits::value_to_yaml(msg.activate_vacuum, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VacuumGripper_Request & msg, bool use_flow_style = false)
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
  const pm_skills_interfaces::srv::VacuumGripper_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pm_skills_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pm_skills_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pm_skills_interfaces::srv::VacuumGripper_Request & msg)
{
  return pm_skills_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pm_skills_interfaces::srv::VacuumGripper_Request>()
{
  return "pm_skills_interfaces::srv::VacuumGripper_Request";
}

template<>
inline const char * name<pm_skills_interfaces::srv::VacuumGripper_Request>()
{
  return "pm_skills_interfaces/srv/VacuumGripper_Request";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::VacuumGripper_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::VacuumGripper_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pm_skills_interfaces::srv::VacuumGripper_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace pm_skills_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const VacuumGripper_Response & msg,
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
  const VacuumGripper_Response & msg,
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

inline std::string to_yaml(const VacuumGripper_Response & msg, bool use_flow_style = false)
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
  const pm_skills_interfaces::srv::VacuumGripper_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pm_skills_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pm_skills_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pm_skills_interfaces::srv::VacuumGripper_Response & msg)
{
  return pm_skills_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pm_skills_interfaces::srv::VacuumGripper_Response>()
{
  return "pm_skills_interfaces::srv::VacuumGripper_Response";
}

template<>
inline const char * name<pm_skills_interfaces::srv::VacuumGripper_Response>()
{
  return "pm_skills_interfaces/srv/VacuumGripper_Response";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::VacuumGripper_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::VacuumGripper_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pm_skills_interfaces::srv::VacuumGripper_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pm_skills_interfaces::srv::VacuumGripper>()
{
  return "pm_skills_interfaces::srv::VacuumGripper";
}

template<>
inline const char * name<pm_skills_interfaces::srv::VacuumGripper>()
{
  return "pm_skills_interfaces/srv/VacuumGripper";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::VacuumGripper>
  : std::integral_constant<
    bool,
    has_fixed_size<pm_skills_interfaces::srv::VacuumGripper_Request>::value &&
    has_fixed_size<pm_skills_interfaces::srv::VacuumGripper_Response>::value
  >
{
};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::VacuumGripper>
  : std::integral_constant<
    bool,
    has_bounded_size<pm_skills_interfaces::srv::VacuumGripper_Request>::value &&
    has_bounded_size<pm_skills_interfaces::srv::VacuumGripper_Response>::value
  >
{
};

template<>
struct is_service<pm_skills_interfaces::srv::VacuumGripper>
  : std::true_type
{
};

template<>
struct is_service_request<pm_skills_interfaces::srv::VacuumGripper_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pm_skills_interfaces::srv::VacuumGripper_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__VACUUM_GRIPPER__TRAITS_HPP_
