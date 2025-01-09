// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from frob_interfaces:srv/Forward.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "frob_interfaces/srv/forward.hpp"


#ifndef FROB_INTERFACES__SRV__DETAIL__FORWARD__BUILDER_HPP_
#define FROB_INTERFACES__SRV__DETAIL__FORWARD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "frob_interfaces/srv/detail/forward__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace frob_interfaces
{

namespace srv
{

namespace builder
{

class Init_Forward_Request_speed
{
public:
  explicit Init_Forward_Request_speed(::frob_interfaces::srv::Forward_Request & msg)
  : msg_(msg)
  {}
  ::frob_interfaces::srv::Forward_Request speed(::frob_interfaces::srv::Forward_Request::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::frob_interfaces::srv::Forward_Request msg_;
};

class Init_Forward_Request_dist
{
public:
  Init_Forward_Request_dist()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Forward_Request_speed dist(::frob_interfaces::srv::Forward_Request::_dist_type arg)
  {
    msg_.dist = std::move(arg);
    return Init_Forward_Request_speed(msg_);
  }

private:
  ::frob_interfaces::srv::Forward_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::frob_interfaces::srv::Forward_Request>()
{
  return frob_interfaces::srv::builder::Init_Forward_Request_dist();
}

}  // namespace frob_interfaces


namespace frob_interfaces
{

namespace srv
{

namespace builder
{

class Init_Forward_Response_success
{
public:
  Init_Forward_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::frob_interfaces::srv::Forward_Response success(::frob_interfaces::srv::Forward_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::frob_interfaces::srv::Forward_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::frob_interfaces::srv::Forward_Response>()
{
  return frob_interfaces::srv::builder::Init_Forward_Response_success();
}

}  // namespace frob_interfaces


namespace frob_interfaces
{

namespace srv
{

namespace builder
{

class Init_Forward_Event_response
{
public:
  explicit Init_Forward_Event_response(::frob_interfaces::srv::Forward_Event & msg)
  : msg_(msg)
  {}
  ::frob_interfaces::srv::Forward_Event response(::frob_interfaces::srv::Forward_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::frob_interfaces::srv::Forward_Event msg_;
};

class Init_Forward_Event_request
{
public:
  explicit Init_Forward_Event_request(::frob_interfaces::srv::Forward_Event & msg)
  : msg_(msg)
  {}
  Init_Forward_Event_response request(::frob_interfaces::srv::Forward_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Forward_Event_response(msg_);
  }

private:
  ::frob_interfaces::srv::Forward_Event msg_;
};

class Init_Forward_Event_info
{
public:
  Init_Forward_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Forward_Event_request info(::frob_interfaces::srv::Forward_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Forward_Event_request(msg_);
  }

private:
  ::frob_interfaces::srv::Forward_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::frob_interfaces::srv::Forward_Event>()
{
  return frob_interfaces::srv::builder::Init_Forward_Event_info();
}

}  // namespace frob_interfaces

#endif  // FROB_INTERFACES__SRV__DETAIL__FORWARD__BUILDER_HPP_