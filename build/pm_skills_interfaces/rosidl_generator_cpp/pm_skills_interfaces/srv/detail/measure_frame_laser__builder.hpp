// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pm_skills_interfaces:srv/MeasureFrameLaser.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__MEASURE_FRAME_LASER__BUILDER_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__MEASURE_FRAME_LASER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pm_skills_interfaces/srv/detail/measure_frame_laser__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_MeasureFrameLaser_Request_frame_name
{
public:
  Init_MeasureFrameLaser_Request_frame_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pm_skills_interfaces::srv::MeasureFrameLaser_Request frame_name(::pm_skills_interfaces::srv::MeasureFrameLaser_Request::_frame_name_type arg)
  {
    msg_.frame_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::MeasureFrameLaser_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::MeasureFrameLaser_Request>()
{
  return pm_skills_interfaces::srv::builder::Init_MeasureFrameLaser_Request_frame_name();
}

}  // namespace pm_skills_interfaces


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_MeasureFrameLaser_Response_component_uuid
{
public:
  explicit Init_MeasureFrameLaser_Response_component_uuid(::pm_skills_interfaces::srv::MeasureFrameLaser_Response & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::MeasureFrameLaser_Response component_uuid(::pm_skills_interfaces::srv::MeasureFrameLaser_Response::_component_uuid_type arg)
  {
    msg_.component_uuid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::MeasureFrameLaser_Response msg_;
};

class Init_MeasureFrameLaser_Response_component_name
{
public:
  explicit Init_MeasureFrameLaser_Response_component_name(::pm_skills_interfaces::srv::MeasureFrameLaser_Response & msg)
  : msg_(msg)
  {}
  Init_MeasureFrameLaser_Response_component_uuid component_name(::pm_skills_interfaces::srv::MeasureFrameLaser_Response::_component_name_type arg)
  {
    msg_.component_name = std::move(arg);
    return Init_MeasureFrameLaser_Response_component_uuid(msg_);
  }

private:
  ::pm_skills_interfaces::srv::MeasureFrameLaser_Response msg_;
};

class Init_MeasureFrameLaser_Response_message
{
public:
  explicit Init_MeasureFrameLaser_Response_message(::pm_skills_interfaces::srv::MeasureFrameLaser_Response & msg)
  : msg_(msg)
  {}
  Init_MeasureFrameLaser_Response_component_name message(::pm_skills_interfaces::srv::MeasureFrameLaser_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_MeasureFrameLaser_Response_component_name(msg_);
  }

private:
  ::pm_skills_interfaces::srv::MeasureFrameLaser_Response msg_;
};

class Init_MeasureFrameLaser_Response_result_vector
{
public:
  explicit Init_MeasureFrameLaser_Response_result_vector(::pm_skills_interfaces::srv::MeasureFrameLaser_Response & msg)
  : msg_(msg)
  {}
  Init_MeasureFrameLaser_Response_message result_vector(::pm_skills_interfaces::srv::MeasureFrameLaser_Response::_result_vector_type arg)
  {
    msg_.result_vector = std::move(arg);
    return Init_MeasureFrameLaser_Response_message(msg_);
  }

private:
  ::pm_skills_interfaces::srv::MeasureFrameLaser_Response msg_;
};

class Init_MeasureFrameLaser_Response_success
{
public:
  Init_MeasureFrameLaser_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeasureFrameLaser_Response_result_vector success(::pm_skills_interfaces::srv::MeasureFrameLaser_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MeasureFrameLaser_Response_result_vector(msg_);
  }

private:
  ::pm_skills_interfaces::srv::MeasureFrameLaser_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::MeasureFrameLaser_Response>()
{
  return pm_skills_interfaces::srv::builder::Init_MeasureFrameLaser_Response_success();
}

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__MEASURE_FRAME_LASER__BUILDER_HPP_
