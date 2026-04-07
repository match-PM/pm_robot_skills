// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pm_skills_interfaces:srv/GripComponent.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__GRIP_COMPONENT__BUILDER_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__GRIP_COMPONENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pm_skills_interfaces/srv/detail/grip_component__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_GripComponent_Request_align_orientation
{
public:
  explicit Init_GripComponent_Request_align_orientation(::pm_skills_interfaces::srv::GripComponent_Request & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::GripComponent_Request align_orientation(::pm_skills_interfaces::srv::GripComponent_Request::_align_orientation_type arg)
  {
    msg_.align_orientation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::GripComponent_Request msg_;
};

class Init_GripComponent_Request_component_name
{
public:
  Init_GripComponent_Request_component_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripComponent_Request_align_orientation component_name(::pm_skills_interfaces::srv::GripComponent_Request::_component_name_type arg)
  {
    msg_.component_name = std::move(arg);
    return Init_GripComponent_Request_align_orientation(msg_);
  }

private:
  ::pm_skills_interfaces::srv::GripComponent_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::GripComponent_Request>()
{
  return pm_skills_interfaces::srv::builder::Init_GripComponent_Request_component_name();
}

}  // namespace pm_skills_interfaces


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_GripComponent_Response_message
{
public:
  explicit Init_GripComponent_Response_message(::pm_skills_interfaces::srv::GripComponent_Response & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::GripComponent_Response message(::pm_skills_interfaces::srv::GripComponent_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::GripComponent_Response msg_;
};

class Init_GripComponent_Response_success
{
public:
  Init_GripComponent_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripComponent_Response_message success(::pm_skills_interfaces::srv::GripComponent_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GripComponent_Response_message(msg_);
  }

private:
  ::pm_skills_interfaces::srv::GripComponent_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::GripComponent_Response>()
{
  return pm_skills_interfaces::srv::builder::Init_GripComponent_Response_success();
}

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__GRIP_COMPONENT__BUILDER_HPP_
