// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pm_skills_interfaces:srv/PlaceComponent.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__PLACE_COMPONENT__BUILDER_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__PLACE_COMPONENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pm_skills_interfaces/srv/detail/place_component__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_PlaceComponent_Request_rz_offset_deg
{
public:
  explicit Init_PlaceComponent_Request_rz_offset_deg(::pm_skills_interfaces::srv::PlaceComponent_Request & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::PlaceComponent_Request rz_offset_deg(::pm_skills_interfaces::srv::PlaceComponent_Request::_rz_offset_deg_type arg)
  {
    msg_.rz_offset_deg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::PlaceComponent_Request msg_;
};

class Init_PlaceComponent_Request_ry_offset_deg
{
public:
  explicit Init_PlaceComponent_Request_ry_offset_deg(::pm_skills_interfaces::srv::PlaceComponent_Request & msg)
  : msg_(msg)
  {}
  Init_PlaceComponent_Request_rz_offset_deg ry_offset_deg(::pm_skills_interfaces::srv::PlaceComponent_Request::_ry_offset_deg_type arg)
  {
    msg_.ry_offset_deg = std::move(arg);
    return Init_PlaceComponent_Request_rz_offset_deg(msg_);
  }

private:
  ::pm_skills_interfaces::srv::PlaceComponent_Request msg_;
};

class Init_PlaceComponent_Request_rx_offset_deg
{
public:
  explicit Init_PlaceComponent_Request_rx_offset_deg(::pm_skills_interfaces::srv::PlaceComponent_Request & msg)
  : msg_(msg)
  {}
  Init_PlaceComponent_Request_ry_offset_deg rx_offset_deg(::pm_skills_interfaces::srv::PlaceComponent_Request::_rx_offset_deg_type arg)
  {
    msg_.rx_offset_deg = std::move(arg);
    return Init_PlaceComponent_Request_ry_offset_deg(msg_);
  }

private:
  ::pm_skills_interfaces::srv::PlaceComponent_Request msg_;
};

class Init_PlaceComponent_Request_z_offset_um
{
public:
  explicit Init_PlaceComponent_Request_z_offset_um(::pm_skills_interfaces::srv::PlaceComponent_Request & msg)
  : msg_(msg)
  {}
  Init_PlaceComponent_Request_rx_offset_deg z_offset_um(::pm_skills_interfaces::srv::PlaceComponent_Request::_z_offset_um_type arg)
  {
    msg_.z_offset_um = std::move(arg);
    return Init_PlaceComponent_Request_rx_offset_deg(msg_);
  }

private:
  ::pm_skills_interfaces::srv::PlaceComponent_Request msg_;
};

class Init_PlaceComponent_Request_y_offset_um
{
public:
  explicit Init_PlaceComponent_Request_y_offset_um(::pm_skills_interfaces::srv::PlaceComponent_Request & msg)
  : msg_(msg)
  {}
  Init_PlaceComponent_Request_z_offset_um y_offset_um(::pm_skills_interfaces::srv::PlaceComponent_Request::_y_offset_um_type arg)
  {
    msg_.y_offset_um = std::move(arg);
    return Init_PlaceComponent_Request_z_offset_um(msg_);
  }

private:
  ::pm_skills_interfaces::srv::PlaceComponent_Request msg_;
};

class Init_PlaceComponent_Request_x_offset_um
{
public:
  explicit Init_PlaceComponent_Request_x_offset_um(::pm_skills_interfaces::srv::PlaceComponent_Request & msg)
  : msg_(msg)
  {}
  Init_PlaceComponent_Request_y_offset_um x_offset_um(::pm_skills_interfaces::srv::PlaceComponent_Request::_x_offset_um_type arg)
  {
    msg_.x_offset_um = std::move(arg);
    return Init_PlaceComponent_Request_y_offset_um(msg_);
  }

private:
  ::pm_skills_interfaces::srv::PlaceComponent_Request msg_;
};

class Init_PlaceComponent_Request_align_orientation
{
public:
  Init_PlaceComponent_Request_align_orientation()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlaceComponent_Request_x_offset_um align_orientation(::pm_skills_interfaces::srv::PlaceComponent_Request::_align_orientation_type arg)
  {
    msg_.align_orientation = std::move(arg);
    return Init_PlaceComponent_Request_x_offset_um(msg_);
  }

private:
  ::pm_skills_interfaces::srv::PlaceComponent_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::PlaceComponent_Request>()
{
  return pm_skills_interfaces::srv::builder::Init_PlaceComponent_Request_align_orientation();
}

}  // namespace pm_skills_interfaces


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_PlaceComponent_Response_message
{
public:
  explicit Init_PlaceComponent_Response_message(::pm_skills_interfaces::srv::PlaceComponent_Response & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::PlaceComponent_Response message(::pm_skills_interfaces::srv::PlaceComponent_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::PlaceComponent_Response msg_;
};

class Init_PlaceComponent_Response_success
{
public:
  Init_PlaceComponent_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlaceComponent_Response_message success(::pm_skills_interfaces::srv::PlaceComponent_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PlaceComponent_Response_message(msg_);
  }

private:
  ::pm_skills_interfaces::srv::PlaceComponent_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::PlaceComponent_Response>()
{
  return pm_skills_interfaces::srv::builder::Init_PlaceComponent_Response_success();
}

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__PLACE_COMPONENT__BUILDER_HPP_
