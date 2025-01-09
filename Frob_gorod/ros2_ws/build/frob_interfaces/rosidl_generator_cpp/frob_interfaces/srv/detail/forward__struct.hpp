// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from frob_interfaces:srv/Forward.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "frob_interfaces/srv/forward.hpp"


#ifndef FROB_INTERFACES__SRV__DETAIL__FORWARD__STRUCT_HPP_
#define FROB_INTERFACES__SRV__DETAIL__FORWARD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__frob_interfaces__srv__Forward_Request __attribute__((deprecated))
#else
# define DEPRECATED__frob_interfaces__srv__Forward_Request __declspec(deprecated)
#endif

namespace frob_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Forward_Request_
{
  using Type = Forward_Request_<ContainerAllocator>;

  explicit Forward_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dist = 0l;
      this->speed = 0l;
    }
  }

  explicit Forward_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dist = 0l;
      this->speed = 0l;
    }
  }

  // field types and members
  using _dist_type =
    int32_t;
  _dist_type dist;
  using _speed_type =
    int32_t;
  _speed_type speed;

  // setters for named parameter idiom
  Type & set__dist(
    const int32_t & _arg)
  {
    this->dist = _arg;
    return *this;
  }
  Type & set__speed(
    const int32_t & _arg)
  {
    this->speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    frob_interfaces::srv::Forward_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const frob_interfaces::srv::Forward_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<frob_interfaces::srv::Forward_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<frob_interfaces::srv::Forward_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      frob_interfaces::srv::Forward_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<frob_interfaces::srv::Forward_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      frob_interfaces::srv::Forward_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<frob_interfaces::srv::Forward_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<frob_interfaces::srv::Forward_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<frob_interfaces::srv::Forward_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__frob_interfaces__srv__Forward_Request
    std::shared_ptr<frob_interfaces::srv::Forward_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__frob_interfaces__srv__Forward_Request
    std::shared_ptr<frob_interfaces::srv::Forward_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Forward_Request_ & other) const
  {
    if (this->dist != other.dist) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const Forward_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Forward_Request_

// alias to use template instance with default allocator
using Forward_Request =
  frob_interfaces::srv::Forward_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace frob_interfaces


#ifndef _WIN32
# define DEPRECATED__frob_interfaces__srv__Forward_Response __attribute__((deprecated))
#else
# define DEPRECATED__frob_interfaces__srv__Forward_Response __declspec(deprecated)
#endif

namespace frob_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Forward_Response_
{
  using Type = Forward_Response_<ContainerAllocator>;

  explicit Forward_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit Forward_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    frob_interfaces::srv::Forward_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const frob_interfaces::srv::Forward_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<frob_interfaces::srv::Forward_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<frob_interfaces::srv::Forward_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      frob_interfaces::srv::Forward_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<frob_interfaces::srv::Forward_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      frob_interfaces::srv::Forward_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<frob_interfaces::srv::Forward_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<frob_interfaces::srv::Forward_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<frob_interfaces::srv::Forward_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__frob_interfaces__srv__Forward_Response
    std::shared_ptr<frob_interfaces::srv::Forward_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__frob_interfaces__srv__Forward_Response
    std::shared_ptr<frob_interfaces::srv::Forward_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Forward_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const Forward_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Forward_Response_

// alias to use template instance with default allocator
using Forward_Response =
  frob_interfaces::srv::Forward_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace frob_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__frob_interfaces__srv__Forward_Event __attribute__((deprecated))
#else
# define DEPRECATED__frob_interfaces__srv__Forward_Event __declspec(deprecated)
#endif

namespace frob_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Forward_Event_
{
  using Type = Forward_Event_<ContainerAllocator>;

  explicit Forward_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit Forward_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<frob_interfaces::srv::Forward_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<frob_interfaces::srv::Forward_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<frob_interfaces::srv::Forward_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<frob_interfaces::srv::Forward_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<frob_interfaces::srv::Forward_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<frob_interfaces::srv::Forward_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<frob_interfaces::srv::Forward_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<frob_interfaces::srv::Forward_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    frob_interfaces::srv::Forward_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const frob_interfaces::srv::Forward_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<frob_interfaces::srv::Forward_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<frob_interfaces::srv::Forward_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      frob_interfaces::srv::Forward_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<frob_interfaces::srv::Forward_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      frob_interfaces::srv::Forward_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<frob_interfaces::srv::Forward_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<frob_interfaces::srv::Forward_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<frob_interfaces::srv::Forward_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__frob_interfaces__srv__Forward_Event
    std::shared_ptr<frob_interfaces::srv::Forward_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__frob_interfaces__srv__Forward_Event
    std::shared_ptr<frob_interfaces::srv::Forward_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Forward_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const Forward_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Forward_Event_

// alias to use template instance with default allocator
using Forward_Event =
  frob_interfaces::srv::Forward_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace frob_interfaces

namespace frob_interfaces
{

namespace srv
{

struct Forward
{
  using Request = frob_interfaces::srv::Forward_Request;
  using Response = frob_interfaces::srv::Forward_Response;
  using Event = frob_interfaces::srv::Forward_Event;
};

}  // namespace srv

}  // namespace frob_interfaces

#endif  // FROB_INTERFACES__SRV__DETAIL__FORWARD__STRUCT_HPP_
