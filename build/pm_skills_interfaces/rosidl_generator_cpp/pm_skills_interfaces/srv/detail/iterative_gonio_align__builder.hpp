// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pm_skills_interfaces:srv/IterativeGonioAlign.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__ITERATIVE_GONIO_ALIGN__BUILDER_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__ITERATIVE_GONIO_ALIGN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pm_skills_interfaces/srv/detail/iterative_gonio_align__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_IterativeGonioAlign_Request_frames_to_measure
{
public:
  explicit Init_IterativeGonioAlign_Request_frames_to_measure(::pm_skills_interfaces::srv::IterativeGonioAlign_Request & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::IterativeGonioAlign_Request frames_to_measure(::pm_skills_interfaces::srv::IterativeGonioAlign_Request::_frames_to_measure_type arg)
  {
    msg_.frames_to_measure = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::IterativeGonioAlign_Request msg_;
};

class Init_IterativeGonioAlign_Request_num_iterations
{
public:
  explicit Init_IterativeGonioAlign_Request_num_iterations(::pm_skills_interfaces::srv::IterativeGonioAlign_Request & msg)
  : msg_(msg)
  {}
  Init_IterativeGonioAlign_Request_frames_to_measure num_iterations(::pm_skills_interfaces::srv::IterativeGonioAlign_Request::_num_iterations_type arg)
  {
    msg_.num_iterations = std::move(arg);
    return Init_IterativeGonioAlign_Request_frames_to_measure(msg_);
  }

private:
  ::pm_skills_interfaces::srv::IterativeGonioAlign_Request msg_;
};

class Init_IterativeGonioAlign_Request_gonio_endeffector_frame
{
public:
  explicit Init_IterativeGonioAlign_Request_gonio_endeffector_frame(::pm_skills_interfaces::srv::IterativeGonioAlign_Request & msg)
  : msg_(msg)
  {}
  Init_IterativeGonioAlign_Request_num_iterations gonio_endeffector_frame(::pm_skills_interfaces::srv::IterativeGonioAlign_Request::_gonio_endeffector_frame_type arg)
  {
    msg_.gonio_endeffector_frame = std::move(arg);
    return Init_IterativeGonioAlign_Request_num_iterations(msg_);
  }

private:
  ::pm_skills_interfaces::srv::IterativeGonioAlign_Request msg_;
};

class Init_IterativeGonioAlign_Request_target_alignment_frame
{
public:
  explicit Init_IterativeGonioAlign_Request_target_alignment_frame(::pm_skills_interfaces::srv::IterativeGonioAlign_Request & msg)
  : msg_(msg)
  {}
  Init_IterativeGonioAlign_Request_gonio_endeffector_frame target_alignment_frame(::pm_skills_interfaces::srv::IterativeGonioAlign_Request::_target_alignment_frame_type arg)
  {
    msg_.target_alignment_frame = std::move(arg);
    return Init_IterativeGonioAlign_Request_gonio_endeffector_frame(msg_);
  }

private:
  ::pm_skills_interfaces::srv::IterativeGonioAlign_Request msg_;
};

class Init_IterativeGonioAlign_Request_confocal_laser
{
public:
  Init_IterativeGonioAlign_Request_confocal_laser()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IterativeGonioAlign_Request_target_alignment_frame confocal_laser(::pm_skills_interfaces::srv::IterativeGonioAlign_Request::_confocal_laser_type arg)
  {
    msg_.confocal_laser = std::move(arg);
    return Init_IterativeGonioAlign_Request_target_alignment_frame(msg_);
  }

private:
  ::pm_skills_interfaces::srv::IterativeGonioAlign_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::IterativeGonioAlign_Request>()
{
  return pm_skills_interfaces::srv::builder::Init_IterativeGonioAlign_Request_confocal_laser();
}

}  // namespace pm_skills_interfaces


namespace pm_skills_interfaces
{

namespace srv
{

namespace builder
{

class Init_IterativeGonioAlign_Response_message
{
public:
  explicit Init_IterativeGonioAlign_Response_message(::pm_skills_interfaces::srv::IterativeGonioAlign_Response & msg)
  : msg_(msg)
  {}
  ::pm_skills_interfaces::srv::IterativeGonioAlign_Response message(::pm_skills_interfaces::srv::IterativeGonioAlign_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pm_skills_interfaces::srv::IterativeGonioAlign_Response msg_;
};

class Init_IterativeGonioAlign_Response_success
{
public:
  Init_IterativeGonioAlign_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IterativeGonioAlign_Response_message success(::pm_skills_interfaces::srv::IterativeGonioAlign_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_IterativeGonioAlign_Response_message(msg_);
  }

private:
  ::pm_skills_interfaces::srv::IterativeGonioAlign_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pm_skills_interfaces::srv::IterativeGonioAlign_Response>()
{
  return pm_skills_interfaces::srv::builder::Init_IterativeGonioAlign_Response_success();
}

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__ITERATIVE_GONIO_ALIGN__BUILDER_HPP_
