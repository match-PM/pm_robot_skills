// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pm_skills_interfaces:srv/VacuumGripper.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__VACUUM_GRIPPER__BUILDER_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__VACUUM_GRIPPER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pm_skills_interfaces/srv/detail/vacuum_gripper__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_VacuumGripper_Request_activate_vacuum
{
public:
  Init_VacuumGripper_Request_activate_vacuum()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pm_skills_interfaces::srv::VacuumGripper_Request activate_vacuum(::pm_skills_interfaces::srv::VacuumGripper_Request::_activate_vacuum_type arg)
  {
    msg_.activate_vacuum = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::VacuumGripper_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::VacuumGripper_Request>()
{
  return pm_skills_interfaces::srv::builder::Init_VacuumGripper_Request_activate_vacuum();
}

}  // namespace pm_skills_interfaces


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_VacuumGripper_Response_message
{
public:
  explicit Init_VacuumGripper_Response_message(::pm_skills_interfaces::srv::VacuumGripper_Response & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::VacuumGripper_Response message(::pm_skills_interfaces::srv::VacuumGripper_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::VacuumGripper_Response msg_;
};

class Init_VacuumGripper_Response_success
{
public:
  Init_VacuumGripper_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VacuumGripper_Response_message success(::pm_skills_interfaces::srv::VacuumGripper_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_VacuumGripper_Response_message(msg_);
  }

private:
  ::pm_skills_interfaces::srv::VacuumGripper_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::VacuumGripper_Response>()
{
  return pm_skills_interfaces::srv::builder::Init_VacuumGripper_Response_success();
}

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__VACUUM_GRIPPER__BUILDER_HPP_
