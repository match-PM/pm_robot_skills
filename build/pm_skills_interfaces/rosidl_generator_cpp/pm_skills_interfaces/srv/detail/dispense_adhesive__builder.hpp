// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pm_skills_interfaces:srv/DispenseAdhesive.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__DISPENSE_ADHESIVE__BUILDER_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__DISPENSE_ADHESIVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pm_skills_interfaces/srv/detail/dispense_adhesive__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_DispenseAdhesive_Request_dispense_time
{
public:
  explicit Init_DispenseAdhesive_Request_dispense_time(::pm_skills_interfaces::srv::DispenseAdhesive_Request & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::DispenseAdhesive_Request dispense_time(::pm_skills_interfaces::srv::DispenseAdhesive_Request::_dispense_time_type arg)
  {
    msg_.dispense_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::DispenseAdhesive_Request msg_;
};

class Init_DispenseAdhesive_Request_dispenser
{
public:
  Init_DispenseAdhesive_Request_dispenser()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DispenseAdhesive_Request_dispense_time dispenser(::pm_skills_interfaces::srv::DispenseAdhesive_Request::_dispenser_type arg)
  {
    msg_.dispenser = std::move(arg);
    return Init_DispenseAdhesive_Request_dispense_time(msg_);
  }

private:
  ::pm_skills_interfaces::srv::DispenseAdhesive_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::DispenseAdhesive_Request>()
{
  return pm_skills_interfaces::srv::builder::Init_DispenseAdhesive_Request_dispenser();
}

}  // namespace pm_skills_interfaces


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_DispenseAdhesive_Response_message
{
public:
  explicit Init_DispenseAdhesive_Response_message(::pm_skills_interfaces::srv::DispenseAdhesive_Response & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::DispenseAdhesive_Response message(::pm_skills_interfaces::srv::DispenseAdhesive_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::DispenseAdhesive_Response msg_;
};

class Init_DispenseAdhesive_Response_success
{
public:
  Init_DispenseAdhesive_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DispenseAdhesive_Response_message success(::pm_skills_interfaces::srv::DispenseAdhesive_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_DispenseAdhesive_Response_message(msg_);
  }

private:
  ::pm_skills_interfaces::srv::DispenseAdhesive_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::DispenseAdhesive_Response>()
{
  return pm_skills_interfaces::srv::builder::Init_DispenseAdhesive_Response_success();
}

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__DISPENSE_ADHESIVE__BUILDER_HPP_
