// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pm_skills_interfaces:srv/GripComponent.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__GRIP_COMPONENT__STRUCT_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__GRIP_COMPONENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pm_skills_interfaces__srv__GripComponent_Request __attribute__((deprecated))
#else
# define DEPRECATED__pm_skills_interfaces__srv__GripComponent_Request __declspec(deprecated)
#endif

namespace pm_skills_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GripComponent_Request_
{
  using Type = GripComponent_Request_<ContainerAllocator>;

  explicit GripComponent_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->component_name = "";
      this->align_orientation = false;
    }
  }

  explicit GripComponent_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : component_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->component_name = "";
      this->align_orientation = false;
    }
  }

  // field types and members
  using _component_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _component_name_type component_name;
  using _align_orientation_type =
    bool;
  _align_orientation_type align_orientation;

  // setters for named parameter idiom
  Type & set__component_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->component_name = _arg;
    return *this;
  }
  Type & set__align_orientation(
    const bool & _arg)
  {
    this->align_orientation = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pm_skills_interfaces::srv::GripComponent_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pm_skills_interfaces::srv::GripComponent_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::GripComponent_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::GripComponent_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::GripComponent_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::GripComponent_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::GripComponent_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::GripComponent_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::GripComponent_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::GripComponent_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pm_skills_interfaces__srv__GripComponent_Request
    std::shared_ptr<pm_skills_interfaces::srv::GripComponent_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pm_skills_interfaces__srv__GripComponent_Request
    std::shared_ptr<pm_skills_interfaces::srv::GripComponent_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GripComponent_Request_ & other) const
  {
    if (this->component_name != other.component_name) {
      return false;
    }
    if (this->align_orientation != other.align_orientation) {
      return false;
    }
    return true;
  }
  bool operator!=(const GripComponent_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GripComponent_Request_

// alias to use template instance with default allocator
using GripComponent_Request =
  pm_skills_interfaces::srv::GripComponent_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pm_skills_interfaces


#ifndef _WIN32
# define DEPRECATED__pm_skills_interfaces__srv__GripComponent_Response __attribute__((deprecated))
#else
# define DEPRECATED__pm_skills_interfaces__srv__GripComponent_Response __declspec(deprecated)
#endif

namespace pm_skills_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GripComponent_Response_
{
  using Type = GripComponent_Response_<ContainerAllocator>;

  explicit GripComponent_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit GripComponent_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pm_skills_interfaces::srv::GripComponent_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pm_skills_interfaces::srv::GripComponent_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::GripComponent_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::GripComponent_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::GripComponent_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::GripComponent_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::GripComponent_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::GripComponent_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::GripComponent_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::GripComponent_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pm_skills_interfaces__srv__GripComponent_Response
    std::shared_ptr<pm_skills_interfaces::srv::GripComponent_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pm_skills_interfaces__srv__GripComponent_Response
    std::shared_ptr<pm_skills_interfaces::srv::GripComponent_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GripComponent_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const GripComponent_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GripComponent_Response_

// alias to use template instance with default allocator
using GripComponent_Response =
  pm_skills_interfaces::srv::GripComponent_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pm_skills_interfaces

namespace pm_skills_interfaces
{

namespace srv
{

struct GripComponent
{
  using Request = pm_skills_interfaces::srv::GripComponent_Request;
  using Response = pm_skills_interfaces::srv::GripComponent_Response;
};

}  // namespace srv

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__GRIP_COMPONENT__STRUCT_HPP_
