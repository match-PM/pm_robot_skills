// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pm_skills_interfaces:srv/IterativeGonioAlign.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__ITERATIVE_GONIO_ALIGN__TRAITS_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__ITERATIVE_GONIO_ALIGN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pm_skills_interfaces/srv/detail/iterative_gonio_align__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pm_skills_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const IterativeGonioAlign_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: confocal_laser
  {
    out << "confocal_laser: ";
    rosidl_generator_traits::value_to_yaml(msg.confocal_laser, out);
    out << ", ";
  }

  // member: target_alignment_frame
  {
    out << "target_alignment_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.target_alignment_frame, out);
    out << ", ";
  }

  // member: gonio_endeffector_frame
  {
    out << "gonio_endeffector_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.gonio_endeffector_frame, out);
    out << ", ";
  }

  // member: num_iterations
  {
    out << "num_iterations: ";
    rosidl_generator_traits::value_to_yaml(msg.num_iterations, out);
    out << ", ";
  }

  // member: frames_to_measure
  {
    if (msg.frames_to_measure.size() == 0) {
      out << "frames_to_measure: []";
    } else {
      out << "frames_to_measure: [";
      size_t pending_items = msg.frames_to_measure.size();
      for (auto item : msg.frames_to_measure) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IterativeGonioAlign_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: confocal_laser
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confocal_laser: ";
    rosidl_generator_traits::value_to_yaml(msg.confocal_laser, out);
    out << "\n";
  }

  // member: target_alignment_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_alignment_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.target_alignment_frame, out);
    out << "\n";
  }

  // member: gonio_endeffector_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gonio_endeffector_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.gonio_endeffector_frame, out);
    out << "\n";
  }

  // member: num_iterations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_iterations: ";
    rosidl_generator_traits::value_to_yaml(msg.num_iterations, out);
    out << "\n";
  }

  // member: frames_to_measure
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.frames_to_measure.size() == 0) {
      out << "frames_to_measure: []\n";
    } else {
      out << "frames_to_measure:\n";
      for (auto item : msg.frames_to_measure) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IterativeGonioAlign_Request & msg, bool use_flow_style = false)
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
  const pm_skills_interfaces::srv::IterativeGonioAlign_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pm_skills_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pm_skills_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pm_skills_interfaces::srv::IterativeGonioAlign_Request & msg)
{
  return pm_skills_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pm_skills_interfaces::srv::IterativeGonioAlign_Request>()
{
  return "pm_skills_interfaces::srv::IterativeGonioAlign_Request";
}

template<>
inline const char * name<pm_skills_interfaces::srv::IterativeGonioAlign_Request>()
{
  return "pm_skills_interfaces/srv/IterativeGonioAlign_Request";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::IterativeGonioAlign_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::IterativeGonioAlign_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pm_skills_interfaces::srv::IterativeGonioAlign_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace pm_skills_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const IterativeGonioAlign_Response & msg,
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
  const IterativeGonioAlign_Response & msg,
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

inline std::string to_yaml(const IterativeGonioAlign_Response & msg, bool use_flow_style = false)
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
  const pm_skills_interfaces::srv::IterativeGonioAlign_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pm_skills_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pm_skills_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pm_skills_interfaces::srv::IterativeGonioAlign_Response & msg)
{
  return pm_skills_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pm_skills_interfaces::srv::IterativeGonioAlign_Response>()
{
  return "pm_skills_interfaces::srv::IterativeGonioAlign_Response";
}

template<>
inline const char * name<pm_skills_interfaces::srv::IterativeGonioAlign_Response>()
{
  return "pm_skills_interfaces/srv/IterativeGonioAlign_Response";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::IterativeGonioAlign_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::IterativeGonioAlign_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pm_skills_interfaces::srv::IterativeGonioAlign_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pm_skills_interfaces::srv::IterativeGonioAlign>()
{
  return "pm_skills_interfaces::srv::IterativeGonioAlign";
}

template<>
inline const char * name<pm_skills_interfaces::srv::IterativeGonioAlign>()
{
  return "pm_skills_interfaces/srv/IterativeGonioAlign";
}

template<>
struct has_fixed_size<pm_skills_interfaces::srv::IterativeGonioAlign>
  : std::integral_constant<
    bool,
    has_fixed_size<pm_skills_interfaces::srv::IterativeGonioAlign_Request>::value &&
    has_fixed_size<pm_skills_interfaces::srv::IterativeGonioAlign_Response>::value
  >
{
};

template<>
struct has_bounded_size<pm_skills_interfaces::srv::IterativeGonioAlign>
  : std::integral_constant<
    bool,
    has_bounded_size<pm_skills_interfaces::srv::IterativeGonioAlign_Request>::value &&
    has_bounded_size<pm_skills_interfaces::srv::IterativeGonioAlign_Response>::value
  >
{
};

template<>
struct is_service<pm_skills_interfaces::srv::IterativeGonioAlign>
  : std::true_type
{
};

template<>
struct is_service_request<pm_skills_interfaces::srv::IterativeGonioAlign_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pm_skills_interfaces::srv::IterativeGonioAlign_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__ITERATIVE_GONIO_ALIGN__TRAITS_HPP_
