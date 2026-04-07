// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pm_skills_interfaces:srv/CorrectFrameVision.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME_VISION__BUILDER_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME_VISION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pm_skills_interfaces/srv/detail/correct_frame_vision__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_CorrectFrameVision_Request_remeasure_after_correction
{
public:
  explicit Init_CorrectFrameVision_Request_remeasure_after_correction(::pm_skills_interfaces::srv::CorrectFrameVision_Request & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::CorrectFrameVision_Request remeasure_after_correction(::pm_skills_interfaces::srv::CorrectFrameVision_Request::_remeasure_after_correction_type arg)
  {
    msg_.remeasure_after_correction = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::CorrectFrameVision_Request msg_;
};

class Init_CorrectFrameVision_Request_frame_name
{
public:
  Init_CorrectFrameVision_Request_frame_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CorrectFrameVision_Request_remeasure_after_correction frame_name(::pm_skills_interfaces::srv::CorrectFrameVision_Request::_frame_name_type arg)
  {
    msg_.frame_name = std::move(arg);
    return Init_CorrectFrameVision_Request_remeasure_after_correction(msg_);
  }

private:
  ::pm_skills_interfaces::srv::CorrectFrameVision_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::CorrectFrameVision_Request>()
{
  return pm_skills_interfaces::srv::builder::Init_CorrectFrameVision_Request_frame_name();
}

}  // namespace pm_skills_interfaces


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_CorrectFrameVision_Response_component_uuid
{
public:
  explicit Init_CorrectFrameVision_Response_component_uuid(::pm_skills_interfaces::srv::CorrectFrameVision_Response & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::CorrectFrameVision_Response component_uuid(::pm_skills_interfaces::srv::CorrectFrameVision_Response::_component_uuid_type arg)
  {
    msg_.component_uuid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::CorrectFrameVision_Response msg_;
};

class Init_CorrectFrameVision_Response_component_name
{
public:
  explicit Init_CorrectFrameVision_Response_component_name(::pm_skills_interfaces::srv::CorrectFrameVision_Response & msg)
  : msg_(msg)
  {}
  Init_CorrectFrameVision_Response_component_uuid component_name(::pm_skills_interfaces::srv::CorrectFrameVision_Response::_component_name_type arg)
  {
    msg_.component_name = std::move(arg);
    return Init_CorrectFrameVision_Response_component_uuid(msg_);
  }

private:
  ::pm_skills_interfaces::srv::CorrectFrameVision_Response msg_;
};

class Init_CorrectFrameVision_Response_vision_response
{
public:
  explicit Init_CorrectFrameVision_Response_vision_response(::pm_skills_interfaces::srv::CorrectFrameVision_Response & msg)
  : msg_(msg)
  {}
  Init_CorrectFrameVision_Response_component_name vision_response(::pm_skills_interfaces::srv::CorrectFrameVision_Response::_vision_response_type arg)
  {
    msg_.vision_response = std::move(arg);
    return Init_CorrectFrameVision_Response_component_name(msg_);
  }

private:
  ::pm_skills_interfaces::srv::CorrectFrameVision_Response msg_;
};

class Init_CorrectFrameVision_Response_message
{
public:
  explicit Init_CorrectFrameVision_Response_message(::pm_skills_interfaces::srv::CorrectFrameVision_Response & msg)
  : msg_(msg)
  {}
  Init_CorrectFrameVision_Response_vision_response message(::pm_skills_interfaces::srv::CorrectFrameVision_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_CorrectFrameVision_Response_vision_response(msg_);
  }

private:
  ::pm_skills_interfaces::srv::CorrectFrameVision_Response msg_;
};

class Init_CorrectFrameVision_Response_correction_values
{
public:
  explicit Init_CorrectFrameVision_Response_correction_values(::pm_skills_interfaces::srv::CorrectFrameVision_Response & msg)
  : msg_(msg)
  {}
  Init_CorrectFrameVision_Response_message correction_values(::pm_skills_interfaces::srv::CorrectFrameVision_Response::_correction_values_type arg)
  {
    msg_.correction_values = std::move(arg);
    return Init_CorrectFrameVision_Response_message(msg_);
  }

private:
  ::pm_skills_interfaces::srv::CorrectFrameVision_Response msg_;
};

class Init_CorrectFrameVision_Response_success
{
public:
  Init_CorrectFrameVision_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CorrectFrameVision_Response_correction_values success(::pm_skills_interfaces::srv::CorrectFrameVision_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CorrectFrameVision_Response_correction_values(msg_);
  }

private:
  ::pm_skills_interfaces::srv::CorrectFrameVision_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::CorrectFrameVision_Response>()
{
  return pm_skills_interfaces::srv::builder::Init_CorrectFrameVision_Response_success();
}

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME_VISION__BUILDER_HPP_
