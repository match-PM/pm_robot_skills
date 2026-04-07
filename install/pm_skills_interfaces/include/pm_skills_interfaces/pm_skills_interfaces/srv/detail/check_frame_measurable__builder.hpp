// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pm_skills_interfaces:srv/CheckFrameMeasurable.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__CHECK_FRAME_MEASURABLE__BUILDER_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__CHECK_FRAME_MEASURABLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pm_skills_interfaces/srv/detail/check_frame_measurable__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_CheckFrameMeasurable_Request_frame_name
{
public:
  Init_CheckFrameMeasurable_Request_frame_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pm_skills_interfaces::srv::CheckFrameMeasurable_Request frame_name(::pm_skills_interfaces::srv::CheckFrameMeasurable_Request::_frame_name_type arg)
  {
    msg_.frame_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::CheckFrameMeasurable_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::CheckFrameMeasurable_Request>()
{
  return pm_skills_interfaces::srv::builder::Init_CheckFrameMeasurable_Request_frame_name();
}

}  // namespace pm_skills_interfaces


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_CheckFrameMeasurable_Response_message
{
public:
  explicit Init_CheckFrameMeasurable_Response_message(::pm_skills_interfaces::srv::CheckFrameMeasurable_Response & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::CheckFrameMeasurable_Response message(::pm_skills_interfaces::srv::CheckFrameMeasurable_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::CheckFrameMeasurable_Response msg_;
};

class Init_CheckFrameMeasurable_Response_success
{
public:
  Init_CheckFrameMeasurable_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CheckFrameMeasurable_Response_message success(::pm_skills_interfaces::srv::CheckFrameMeasurable_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CheckFrameMeasurable_Response_message(msg_);
  }

private:
  ::pm_skills_interfaces::srv::CheckFrameMeasurable_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::CheckFrameMeasurable_Response>()
{
  return pm_skills_interfaces::srv::builder::Init_CheckFrameMeasurable_Response_success();
}

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__CHECK_FRAME_MEASURABLE__BUILDER_HPP_
