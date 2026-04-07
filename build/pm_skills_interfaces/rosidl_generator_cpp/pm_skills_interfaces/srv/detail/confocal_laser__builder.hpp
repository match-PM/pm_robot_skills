// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pm_skills_interfaces:srv/ConfocalLaser.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__CONFOCAL_LASER__BUILDER_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__CONFOCAL_LASER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pm_skills_interfaces/srv/detail/confocal_laser__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_ConfocalLaser_Request_laser
{
public:
  Init_ConfocalLaser_Request_laser()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pm_skills_interfaces::srv::ConfocalLaser_Request laser(::pm_skills_interfaces::srv::ConfocalLaser_Request::_laser_type arg)
  {
    msg_.laser = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::ConfocalLaser_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::ConfocalLaser_Request>()
{
  return pm_skills_interfaces::srv::builder::Init_ConfocalLaser_Request_laser();
}

}  // namespace pm_skills_interfaces


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_ConfocalLaser_Response_result
{
public:
  explicit Init_ConfocalLaser_Response_result(::pm_skills_interfaces::srv::ConfocalLaser_Response & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::ConfocalLaser_Response result(::pm_skills_interfaces::srv::ConfocalLaser_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::ConfocalLaser_Response msg_;
};

class Init_ConfocalLaser_Response_success
{
public:
  Init_ConfocalLaser_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConfocalLaser_Response_result success(::pm_skills_interfaces::srv::ConfocalLaser_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ConfocalLaser_Response_result(msg_);
  }

private:
  ::pm_skills_interfaces::srv::ConfocalLaser_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::ConfocalLaser_Response>()
{
  return pm_skills_interfaces::srv::builder::Init_ConfocalLaser_Response_success();
}

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__CONFOCAL_LASER__BUILDER_HPP_
