// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pm_skills_interfaces:srv/CorrectFrameVision.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME_VISION__STRUCT_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME_VISION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pm_skills_interfaces__srv__CorrectFrameVision_Request __attribute__((deprecated))
#else
# define DEPRECATED__pm_skills_interfaces__srv__CorrectFrameVision_Request __declspec(deprecated)
#endif

namespace pm_skills_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CorrectFrameVision_Request_
{
  using Type = CorrectFrameVision_Request_<ContainerAllocator>;

  explicit CorrectFrameVision_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->remeasure_after_correction = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->frame_name = "";
      this->remeasure_after_correction = false;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_name = "";
    }
  }

  explicit CorrectFrameVision_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : frame_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->remeasure_after_correction = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->frame_name = "";
      this->remeasure_after_correction = false;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_name = "";
    }
  }

  // field types and members
  using _frame_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _frame_name_type frame_name;
  using _remeasure_after_correction_type =
    bool;
  _remeasure_after_correction_type remeasure_after_correction;

  // setters for named parameter idiom
  Type & set__frame_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->frame_name = _arg;
    return *this;
  }
  Type & set__remeasure_after_correction(
    const bool & _arg)
  {
    this->remeasure_after_correction = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pm_skills_interfaces::srv::CorrectFrameVision_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pm_skills_interfaces::srv::CorrectFrameVision_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::CorrectFrameVision_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::CorrectFrameVision_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pm_skills_interfaces__srv__CorrectFrameVision_Request
    std::shared_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pm_skills_interfaces__srv__CorrectFrameVision_Request
    std::shared_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CorrectFrameVision_Request_ & other) const
  {
    if (this->frame_name != other.frame_name) {
      return false;
    }
    if (this->remeasure_after_correction != other.remeasure_after_correction) {
      return false;
    }
    return true;
  }
  bool operator!=(const CorrectFrameVision_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CorrectFrameVision_Request_

// alias to use template instance with default allocator
using CorrectFrameVision_Request =
  pm_skills_interfaces::srv::CorrectFrameVision_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pm_skills_interfaces


// Include directives for member types
// Member 'correction_values'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'vision_response'
#include "pm_vision_interfaces/msg/detail/vision_response__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pm_skills_interfaces__srv__CorrectFrameVision_Response __attribute__((deprecated))
#else
# define DEPRECATED__pm_skills_interfaces__srv__CorrectFrameVision_Response __declspec(deprecated)
#endif

namespace pm_skills_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CorrectFrameVision_Response_
{
  using Type = CorrectFrameVision_Response_<ContainerAllocator>;

  explicit CorrectFrameVision_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : correction_values(_init),
    vision_response(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->component_name = "";
      this->component_uuid = "";
    }
  }

  explicit CorrectFrameVision_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : correction_values(_alloc, _init),
    message(_alloc),
    vision_response(_alloc, _init),
    component_name(_alloc),
    component_uuid(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->component_name = "";
      this->component_uuid = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _correction_values_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _correction_values_type correction_values;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _vision_response_type =
    pm_vision_interfaces::msg::VisionResponse_<ContainerAllocator>;
  _vision_response_type vision_response;
  using _component_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _component_name_type component_name;
  using _component_uuid_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _component_uuid_type component_uuid;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__correction_values(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->correction_values = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__vision_response(
    const pm_vision_interfaces::msg::VisionResponse_<ContainerAllocator> & _arg)
  {
    this->vision_response = _arg;
    return *this;
  }
  Type & set__component_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->component_name = _arg;
    return *this;
  }
  Type & set__component_uuid(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->component_uuid = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pm_skills_interfaces::srv::CorrectFrameVision_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pm_skills_interfaces::srv::CorrectFrameVision_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::CorrectFrameVision_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::CorrectFrameVision_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pm_skills_interfaces__srv__CorrectFrameVision_Response
    std::shared_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pm_skills_interfaces__srv__CorrectFrameVision_Response
    std::shared_ptr<pm_skills_interfaces::srv::CorrectFrameVision_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CorrectFrameVision_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->correction_values != other.correction_values) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->vision_response != other.vision_response) {
      return false;
    }
    if (this->component_name != other.component_name) {
      return false;
    }
    if (this->component_uuid != other.component_uuid) {
      return false;
    }
    return true;
  }
  bool operator!=(const CorrectFrameVision_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CorrectFrameVision_Response_

// alias to use template instance with default allocator
using CorrectFrameVision_Response =
  pm_skills_interfaces::srv::CorrectFrameVision_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pm_skills_interfaces

namespace pm_skills_interfaces
{

namespace srv
{

struct CorrectFrameVision
{
  using Request = pm_skills_interfaces::srv::CorrectFrameVision_Request;
  using Response = pm_skills_interfaces::srv::CorrectFrameVision_Response;
};

}  // namespace srv

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__CORRECT_FRAME_VISION__STRUCT_HPP_
