// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pm_skills_interfaces:srv/CorrectFrame.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME__BUILDER_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pm_skills_interfaces/srv/detail/correct_frame__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pm_skills_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::CorrectFrame_Request>()
{
  return ::pm_skills_interfaces::srv::CorrectFrame_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace pm_skills_interfaces


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_CorrectFrame_Response_message
{
public:
  explicit Init_CorrectFrame_Response_message(::pm_skills_interfaces::srv::CorrectFrame_Response & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::CorrectFrame_Response message(::pm_skills_interfaces::srv::CorrectFrame_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::CorrectFrame_Response msg_;
};

class Init_CorrectFrame_Response_correction_values
{
public:
  explicit Init_CorrectFrame_Response_correction_values(::pm_skills_interfaces::srv::CorrectFrame_Response & msg)
  : msg_(msg)
  {}
  Init_CorrectFrame_Response_message correction_values(::pm_skills_interfaces::srv::CorrectFrame_Response::_correction_values_type arg)
  {
    msg_.correction_values = std::move(arg);
    return Init_CorrectFrame_Response_message(msg_);
  }

private:
  ::pm_skills_interfaces::srv::CorrectFrame_Response msg_;
};

class Init_CorrectFrame_Response_success
{
public:
  Init_CorrectFrame_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CorrectFrame_Response_correction_values success(::pm_skills_interfaces::srv::CorrectFrame_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CorrectFrame_Response_correction_values(msg_);
  }

private:
  ::pm_skills_interfaces::srv::CorrectFrame_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::CorrectFrame_Response>()
{
  return pm_skills_interfaces::srv::builder::Init_CorrectFrame_Response_success();
}

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME__BUILDER_HPP_
