// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pm_skills_interfaces:srv/PlaceComponent.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__PLACE_COMPONENT__TRAITS_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__PLACE_COMPONENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pm_skills_interfaces/srv/detail/place_component__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pm_skills_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlaceComponent_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: align_orientation
  {
    out << "align_orientation: ";
    rosidl_generator_traits::value_to_yaml(msg.align_orientation, out);
    out << ", ";
  }

  // member: x_offset_um
  {
    out << "x_offset_um: ";
    rosidl_generator_traits::value_to_yaml(msg.x_offset_um, out);
    out << ", ";
  }

  // member: y_offset_um
  {
    out << "y_offset_um: ";
    rosidl_generator_traits::value_to_yaml(msg.y_offset_um, out);
    out << ", ";
  }

  // member: z_offset_um
  {
    out << "z_offset_um: ";
    rosidl_generator_traits::value_to_yaml(msg.z_offset_um, out);
    out << ", ";
  }

  // member: rx_offset_deg
  {
    out << "rx_offset_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.rx_offset_deg, out);
    out << ", ";
  }

  // member: ry_offset_deg
  {
    out << "ry_offset_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.ry_offset_deg, out);
    out << ", ";
  }

  // member: rz_offset_deg
  {
    out << "rz_offset_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.rz_offset_deg, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlaceComponent_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: align_orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "align_orientation: ";
    rosidl_generator_traits::value_to_yaml(msg.align_orientation, out);
    out << "\n";
  }

  // member: x_offset_um
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_offset_um: ";
    rosidl_generator_traits::value_to_yaml(msg.x_offset_um, out);
    out << "\n";
  }

  // member: y_offset_um
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_offset_um: ";
    rosidl_generator_traits::value_to_yaml(msg.y_offset_um, out);
    out << "\n";
  }

  // member: z_offset_um
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_offset_um: ";
    rosidl_generator_traits::value_to_yaml(msg.z_offset_um, out);
    out << "\n";
  }

  // member: rx_offset_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rx_offset_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.rx_offset_deg, out);
    out << "\n";
  }

  // member: ry_offset_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ry_offset_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.ry_offset_deg, out);
    out << "\n";
  }

  // member: rz_offset_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rz_offset_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.rz_offset_deg, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlaceComponent_Request & msg, bool use_flow_style = false)
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
  const pm_skills_interfaces::srv::PlaceComponent_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pm_skills_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pm_skills_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pm_skills_interfaces::srv::PlaceComponent_Request & msg)
{
  return pm_skills_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pm_skills_interfaces::srv::PlaceComponent_Request>()
{
  return "pm_skills_interfaces::srv::PlaceComponent_Request";
}

template<>
inline const char * name<pm_skills_interfaces::srv::PlaceComponent_Request>()
{
  return "pm_skills_interfaces/srv/PlaceComponent_Request";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::PlaceComponent_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::PlaceComponent_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pm_skills_interfaces::srv::PlaceComponent_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace pm_skills_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlaceComponent_Response & msg,
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
  const PlaceComponent_Response & msg,
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

inline std::string to_yaml(const PlaceComponent_Response & msg, bool use_flow_style = false)
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
  const pm_skills_interfaces::srv::PlaceComponent_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pm_skills_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pm_skills_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pm_skills_interfaces::srv::PlaceComponent_Response & msg)
{
  return pm_skills_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pm_skills_interfaces::srv::PlaceComponent_Response>()
{
  return "pm_skills_interfaces::srv::PlaceComponent_Response";
}

template<>
inline const char * name<pm_skills_interfaces::srv::PlaceComponent_Response>()
{
  return "pm_skills_interfaces/srv/PlaceComponent_Response";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::PlaceComponent_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::PlaceComponent_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pm_skills_interfaces::srv::PlaceComponent_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pm_skills_interfaces::srv::PlaceComponent>()
{
  return "pm_skills_interfaces::srv::PlaceComponent";
}

template<>
inline const char * name<pm_skills_interfaces::srv::PlaceComponent>()
{
  return "pm_skills_interfaces/srv/PlaceComponent";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::PlaceComponent>
  : std::integral_constant<
    bool,
    has_fixed_size<pm_skills_interfaces::srv::PlaceComponent_Request>::value &&
    has_fixed_size<pm_skills_interfaces::srv::PlaceComponent_Response>::value
  >
{
};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::PlaceComponent>
  : std::integral_constant<
    bool,
    has_bounded_size<pm_skills_interfaces::srv::PlaceComponent_Request>::value &&
    has_bounded_size<pm_skills_interfaces::srv::PlaceComponent_Response>::value
  >
{
};

template<>
struct is_service<pm_skills_interfaces::srv::PlaceComponent>
  : std::true_type
{
};

template<>
struct is_service_request<pm_skills_interfaces::srv::PlaceComponent_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pm_skills_interfaces::srv::PlaceComponent_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__PLACE_COMPONENT__TRAITS_HPP_
