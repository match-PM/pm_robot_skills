// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pm_skills_interfaces:srv/VacuumGripper.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__VACUUM_GRIPPER__STRUCT_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__VACUUM_GRIPPER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pm_skills_interfaces__srv__VacuumGripper_Request __attribute__((deprecated))
#else
# define DEPRECATED__pm_skills_interfaces__srv__VacuumGripper_Request __declspec(deprecated)
#endif

namespace pm_skills_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct VacuumGripper_Request_
{
  using Type = VacuumGripper_Request_<ContainerAllocator>;

  explicit VacuumGripper_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->activate_vacuum = false;
    }
  }

  explicit VacuumGripper_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->activate_vacuum = false;
    }
  }

  // field types and members
  using _activate_vacuum_type =
    bool;
  _activate_vacuum_type activate_vacuum;

  // setters for named parameter idiom
  Type & set__activate_vacuum(
    const bool & _arg)
  {
    this->activate_vacuum = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pm_skills_interfaces::srv::VacuumGripper_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pm_skills_interfaces::srv::VacuumGripper_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::VacuumGripper_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::VacuumGripper_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::VacuumGripper_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::VacuumGripper_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::VacuumGripper_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::VacuumGripper_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::VacuumGripper_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::VacuumGripper_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pm_skills_interfaces__srv__VacuumGripper_Request
    std::shared_ptr<pm_skills_interfaces::srv::VacuumGripper_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pm_skills_interfaces__srv__VacuumGripper_Request
    std::shared_ptr<pm_skills_interfaces::srv::VacuumGripper_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VacuumGripper_Request_ & other) const
  {
    if (this->activate_vacuum != other.activate_vacuum) {
      return false;
    }
    return true;
  }
  bool operator!=(const VacuumGripper_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VacuumGripper_Request_

// alias to use template instance with default allocator
using VacuumGripper_Request =
  pm_skills_interfaces::srv::VacuumGripper_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pm_skills_interfaces


#ifndef _WIN32
# define DEPRECATED__pm_skills_interfaces__srv__VacuumGripper_Response __attribute__((deprecated))
#else
# define DEPRECATED__pm_skills_interfaces__srv__VacuumGripper_Response __declspec(deprecated)
#endif

namespace pm_skills_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct VacuumGripper_Response_
{
  using Type = VacuumGripper_Response_<ContainerAllocator>;

  explicit VacuumGripper_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit VacuumGripper_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    pm_skills_interfaces::srv::VacuumGripper_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pm_skills_interfaces::srv::VacuumGripper_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::VacuumGripper_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::VacuumGripper_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::VacuumGripper_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::VacuumGripper_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::VacuumGripper_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::VacuumGripper_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::VacuumGripper_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::VacuumGripper_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pm_skills_interfaces__srv__VacuumGripper_Response
    std::shared_ptr<pm_skills_interfaces::srv::VacuumGripper_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pm_skills_interfaces__srv__VacuumGripper_Response
    std::shared_ptr<pm_skills_interfaces::srv::VacuumGripper_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VacuumGripper_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const VacuumGripper_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VacuumGripper_Response_

// alias to use template instance with default allocator
using VacuumGripper_Response =
  pm_skills_interfaces::srv::VacuumGripper_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pm_skills_interfaces

namespace pm_skills_interfaces
{

namespace srv
{

struct VacuumGripper
{
  using Request = pm_skills_interfaces::srv::VacuumGripper_Request;
  using Response = pm_skills_interfaces::srv::VacuumGripper_Response;
};

}  // namespace srv

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__VACUUM_GRIPPER__STRUCT_HPP_
