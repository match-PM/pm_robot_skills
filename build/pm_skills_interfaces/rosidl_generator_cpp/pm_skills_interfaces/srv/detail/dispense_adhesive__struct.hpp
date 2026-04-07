// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pm_skills_interfaces:srv/DispenseAdhesive.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__DISPENSE_ADHESIVE__STRUCT_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__DISPENSE_ADHESIVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pm_skills_interfaces__srv__DispenseAdhesive_Request __attribute__((deprecated))
#else
# define DEPRECATED__pm_skills_interfaces__srv__DispenseAdhesive_Request __declspec(deprecated)
#endif

namespace pm_skills_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DispenseAdhesive_Request_
{
  using Type = DispenseAdhesive_Request_<ContainerAllocator>;

  explicit DispenseAdhesive_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dispenser = "";
      this->dispense_time = 0.0f;
    }
  }

  explicit DispenseAdhesive_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : dispenser(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dispenser = "";
      this->dispense_time = 0.0f;
    }
  }

  // field types and members
  using _dispenser_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _dispenser_type dispenser;
  using _dispense_time_type =
    float;
  _dispense_time_type dispense_time;

  // setters for named parameter idiom
  Type & set__dispenser(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->dispenser = _arg;
    return *this;
  }
  Type & set__dispense_time(
    const float & _arg)
  {
    this->dispense_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pm_skills_interfaces::srv::DispenseAdhesive_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pm_skills_interfaces::srv::DispenseAdhesive_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::DispenseAdhesive_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::DispenseAdhesive_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pm_skills_interfaces__srv__DispenseAdhesive_Request
    std::shared_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pm_skills_interfaces__srv__DispenseAdhesive_Request
    std::shared_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DispenseAdhesive_Request_ & other) const
  {
    if (this->dispenser != other.dispenser) {
      return false;
    }
    if (this->dispense_time != other.dispense_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const DispenseAdhesive_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DispenseAdhesive_Request_

// alias to use template instance with default allocator
using DispenseAdhesive_Request =
  pm_skills_interfaces::srv::DispenseAdhesive_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pm_skills_interfaces


#ifndef _WIN32
# define DEPRECATED__pm_skills_interfaces__srv__DispenseAdhesive_Response __attribute__((deprecated))
#else
# define DEPRECATED__pm_skills_interfaces__srv__DispenseAdhesive_Response __declspec(deprecated)
#endif

namespace pm_skills_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DispenseAdhesive_Response_
{
  using Type = DispenseAdhesive_Response_<ContainerAllocator>;

  explicit DispenseAdhesive_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit DispenseAdhesive_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    pm_skills_interfaces::srv::DispenseAdhesive_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pm_skills_interfaces::srv::DispenseAdhesive_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::DispenseAdhesive_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::DispenseAdhesive_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pm_skills_interfaces__srv__DispenseAdhesive_Response
    std::shared_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pm_skills_interfaces__srv__DispenseAdhesive_Response
    std::shared_ptr<pm_skills_interfaces::srv::DispenseAdhesive_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DispenseAdhesive_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const DispenseAdhesive_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DispenseAdhesive_Response_

// alias to use template instance with default allocator
using DispenseAdhesive_Response =
  pm_skills_interfaces::srv::DispenseAdhesive_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pm_skills_interfaces

namespace pm_skills_interfaces
{

namespace srv
{

struct DispenseAdhesive
{
  using Request = pm_skills_interfaces::srv::DispenseAdhesive_Request;
  using Response = pm_skills_interfaces::srv::DispenseAdhesive_Response;
};

}  // namespace srv

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__DISPENSE_ADHESIVE__STRUCT_HPP_
