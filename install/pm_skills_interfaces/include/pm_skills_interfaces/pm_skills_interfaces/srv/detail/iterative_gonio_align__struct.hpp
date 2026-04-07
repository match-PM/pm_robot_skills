// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pm_skills_interfaces:srv/IterativeGonioAlign.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__ITERATIVE_GONIO_ALIGN__STRUCT_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__ITERATIVE_GONIO_ALIGN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pm_skills_interfaces__srv__IterativeGonioAlign_Request __attribute__((deprecated))
#else
# define DEPRECATED__pm_skills_interfaces__srv__IterativeGonioAlign_Request __declspec(deprecated)
#endif

namespace pm_skills_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct IterativeGonioAlign_Request_
{
  using Type = IterativeGonioAlign_Request_<ContainerAllocator>;

  explicit IterativeGonioAlign_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confocal_laser = false;
      this->target_alignment_frame = "";
      this->gonio_endeffector_frame = "";
      this->num_iterations = 0l;
    }
  }

  explicit IterativeGonioAlign_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_alignment_frame(_alloc),
    gonio_endeffector_frame(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confocal_laser = false;
      this->target_alignment_frame = "";
      this->gonio_endeffector_frame = "";
      this->num_iterations = 0l;
    }
  }

  // field types and members
  using _confocal_laser_type =
    bool;
  _confocal_laser_type confocal_laser;
  using _target_alignment_frame_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _target_alignment_frame_type target_alignment_frame;
  using _gonio_endeffector_frame_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _gonio_endeffector_frame_type gonio_endeffector_frame;
  using _num_iterations_type =
    int32_t;
  _num_iterations_type num_iterations;
  using _frames_to_measure_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _frames_to_measure_type frames_to_measure;

  // setters for named parameter idiom
  Type & set__confocal_laser(
    const bool & _arg)
  {
    this->confocal_laser = _arg;
    return *this;
  }
  Type & set__target_alignment_frame(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->target_alignment_frame = _arg;
    return *this;
  }
  Type & set__gonio_endeffector_frame(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->gonio_endeffector_frame = _arg;
    return *this;
  }
  Type & set__num_iterations(
    const int32_t & _arg)
  {
    this->num_iterations = _arg;
    return *this;
  }
  Type & set__frames_to_measure(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->frames_to_measure = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pm_skills_interfaces::srv::IterativeGonioAlign_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pm_skills_interfaces::srv::IterativeGonioAlign_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::IterativeGonioAlign_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::IterativeGonioAlign_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pm_skills_interfaces__srv__IterativeGonioAlign_Request
    std::shared_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pm_skills_interfaces__srv__IterativeGonioAlign_Request
    std::shared_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IterativeGonioAlign_Request_ & other) const
  {
    if (this->confocal_laser != other.confocal_laser) {
      return false;
    }
    if (this->target_alignment_frame != other.target_alignment_frame) {
      return false;
    }
    if (this->gonio_endeffector_frame != other.gonio_endeffector_frame) {
      return false;
    }
    if (this->num_iterations != other.num_iterations) {
      return false;
    }
    if (this->frames_to_measure != other.frames_to_measure) {
      return false;
    }
    return true;
  }
  bool operator!=(const IterativeGonioAlign_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IterativeGonioAlign_Request_

// alias to use template instance with default allocator
using IterativeGonioAlign_Request =
  pm_skills_interfaces::srv::IterativeGonioAlign_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pm_skills_interfaces


#ifndef _WIN32
# define DEPRECATED__pm_skills_interfaces__srv__IterativeGonioAlign_Response __attribute__((deprecated))
#else
# define DEPRECATED__pm_skills_interfaces__srv__IterativeGonioAlign_Response __declspec(deprecated)
#endif

namespace pm_skills_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct IterativeGonioAlign_Response_
{
  using Type = IterativeGonioAlign_Response_<ContainerAllocator>;

  explicit IterativeGonioAlign_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit IterativeGonioAlign_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    pm_skills_interfaces::srv::IterativeGonioAlign_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pm_skills_interfaces::srv::IterativeGonioAlign_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::IterativeGonioAlign_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::IterativeGonioAlign_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pm_skills_interfaces__srv__IterativeGonioAlign_Response
    std::shared_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pm_skills_interfaces__srv__IterativeGonioAlign_Response
    std::shared_ptr<pm_skills_interfaces::srv::IterativeGonioAlign_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IterativeGonioAlign_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const IterativeGonioAlign_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IterativeGonioAlign_Response_

// alias to use template instance with default allocator
using IterativeGonioAlign_Response =
  pm_skills_interfaces::srv::IterativeGonioAlign_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pm_skills_interfaces

namespace pm_skills_interfaces
{

namespace srv
{

struct IterativeGonioAlign
{
  using Request = pm_skills_interfaces::srv::IterativeGonioAlign_Request;
  using Response = pm_skills_interfaces::srv::IterativeGonioAlign_Response;
};

}  // namespace srv

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__ITERATIVE_GONIO_ALIGN__STRUCT_HPP_
