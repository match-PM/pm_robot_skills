// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pm_skills_interfaces:srv/PlaceComponent.idl
// generated code does not contain a copyright notice

#ifndef PM_SKILLS_INTERFACES__SRV__DETAIL__PLACE_COMPONENT__STRUCT_HPP_
#define PM_SKILLS_INTERFACES__SRV__DETAIL__PLACE_COMPONENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pm_skills_interfaces__srv__PlaceComponent_Request __attribute__((deprecated))
#else
# define DEPRECATED__pm_skills_interfaces__srv__PlaceComponent_Request __declspec(deprecated)
#endif

namespace pm_skills_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlaceComponent_Request_
{
  using Type = PlaceComponent_Request_<ContainerAllocator>;

  explicit PlaceComponent_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->align_orientation = false;
      this->x_offset_um = 0.0f;
      this->y_offset_um = 0.0f;
      this->z_offset_um = 0.0f;
      this->rx_offset_deg = 0.0f;
      this->ry_offset_deg = 0.0f;
      this->rz_offset_deg = 0.0f;
    }
  }

  explicit PlaceComponent_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->align_orientation = false;
      this->x_offset_um = 0.0f;
      this->y_offset_um = 0.0f;
      this->z_offset_um = 0.0f;
      this->rx_offset_deg = 0.0f;
      this->ry_offset_deg = 0.0f;
      this->rz_offset_deg = 0.0f;
    }
  }

  // field types and members
  using _align_orientation_type =
    bool;
  _align_orientation_type align_orientation;
  using _x_offset_um_type =
    float;
  _x_offset_um_type x_offset_um;
  using _y_offset_um_type =
    float;
  _y_offset_um_type y_offset_um;
  using _z_offset_um_type =
    float;
  _z_offset_um_type z_offset_um;
  using _rx_offset_deg_type =
    float;
  _rx_offset_deg_type rx_offset_deg;
  using _ry_offset_deg_type =
    float;
  _ry_offset_deg_type ry_offset_deg;
  using _rz_offset_deg_type =
    float;
  _rz_offset_deg_type rz_offset_deg;

  // setters for named parameter idiom
  Type & set__align_orientation(
    const bool & _arg)
  {
    this->align_orientation = _arg;
    return *this;
  }
  Type & set__x_offset_um(
    const float & _arg)
  {
    this->x_offset_um = _arg;
    return *this;
  }
  Type & set__y_offset_um(
    const float & _arg)
  {
    this->y_offset_um = _arg;
    return *this;
  }
  Type & set__z_offset_um(
    const float & _arg)
  {
    this->z_offset_um = _arg;
    return *this;
  }
  Type & set__rx_offset_deg(
    const float & _arg)
  {
    this->rx_offset_deg = _arg;
    return *this;
  }
  Type & set__ry_offset_deg(
    const float & _arg)
  {
    this->ry_offset_deg = _arg;
    return *this;
  }
  Type & set__rz_offset_deg(
    const float & _arg)
  {
    this->rz_offset_deg = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pm_skills_interfaces::srv::PlaceComponent_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pm_skills_interfaces::srv::PlaceComponent_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::PlaceComponent_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::PlaceComponent_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::PlaceComponent_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::PlaceComponent_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::PlaceComponent_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::PlaceComponent_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::PlaceComponent_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::PlaceComponent_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pm_skills_interfaces__srv__PlaceComponent_Request
    std::shared_ptr<pm_skills_interfaces::srv::PlaceComponent_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pm_skills_interfaces__srv__PlaceComponent_Request
    std::shared_ptr<pm_skills_interfaces::srv::PlaceComponent_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlaceComponent_Request_ & other) const
  {
    if (this->align_orientation != other.align_orientation) {
      return false;
    }
    if (this->x_offset_um != other.x_offset_um) {
      return false;
    }
    if (this->y_offset_um != other.y_offset_um) {
      return false;
    }
    if (this->z_offset_um != other.z_offset_um) {
      return false;
    }
    if (this->rx_offset_deg != other.rx_offset_deg) {
      return false;
    }
    if (this->ry_offset_deg != other.ry_offset_deg) {
      return false;
    }
    if (this->rz_offset_deg != other.rz_offset_deg) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlaceComponent_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlaceComponent_Request_

// alias to use template instance with default allocator
using PlaceComponent_Request =
  pm_skills_interfaces::srv::PlaceComponent_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pm_skills_interfaces


#ifndef _WIN32
# define DEPRECATED__pm_skills_interfaces__srv__PlaceComponent_Response __attribute__((deprecated))
#else
# define DEPRECATED__pm_skills_interfaces__srv__PlaceComponent_Response __declspec(deprecated)
#endif

namespace pm_skills_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlaceComponent_Response_
{
  using Type = PlaceComponent_Response_<ContainerAllocator>;

  explicit PlaceComponent_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit PlaceComponent_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    pm_skills_interfaces::srv::PlaceComponent_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pm_skills_interfaces::srv::PlaceComponent_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::PlaceComponent_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pm_skills_interfaces::srv::PlaceComponent_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::PlaceComponent_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::PlaceComponent_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pm_skills_interfaces::srv::PlaceComponent_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pm_skills_interfaces::srv::PlaceComponent_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::PlaceComponent_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pm_skills_interfaces::srv::PlaceComponent_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pm_skills_interfaces__srv__PlaceComponent_Response
    std::shared_ptr<pm_skills_interfaces::srv::PlaceComponent_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pm_skills_interfaces__srv__PlaceComponent_Response
    std::shared_ptr<pm_skills_interfaces::srv::PlaceComponent_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlaceComponent_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlaceComponent_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlaceComponent_Response_

// alias to use template instance with default allocator
using PlaceComponent_Response =
  pm_skills_interfaces::srv::PlaceComponent_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pm_skills_interfaces

namespace pm_skills_interfaces
{

namespace srv
{

struct PlaceComponent
{
  using Request = pm_skills_interfaces::srv::PlaceComponent_Request;
  using Response = pm_skills_interfaces::srv::PlaceComponent_Response;
};

}  // namespace srv

}  // namespace pm_skills_interfaces

#endif  // PM_SKILLS_INTERFACES__SRV__DETAIL__PLACE_COMPONENT__STRUCT_HPP_
