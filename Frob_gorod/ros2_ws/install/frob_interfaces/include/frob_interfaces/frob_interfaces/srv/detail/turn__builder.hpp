// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from frob_interfaces:srv/Turn.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "frob_interfaces/srv/turn.hpp"


#ifndef FROB_INTERFACES__SRV__DETAIL__TURN__BUILDER_HPP_
#define FROB_INTERFACES__SRV__DETAIL__TURN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "frob_interfaces/srv/detail/turn__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace frob_interfaces
{

namespace srv
{

namespace builder
{

class Init_Turn_Request_speed
{
public:
  explicit Init_Turn_Request_speed(::frob_interfaces::srv::Turn_Request & msg)
  : msg_(msg)
  {}
  ::frob_interfaces::srv::Turn_Request speed(::frob_interfaces::srv::Turn_Request::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::frob_interfaces::srv::Turn_Request msg_;
};

class Init_Turn_Request_angle
{
public:
  Init_Turn_Request_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Turn_Request_speed angle(::frob_interfaces::srv::Turn_Request::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_Turn_Request_speed(msg_);
  }

private:
  ::frob_interfaces::srv::Turn_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::frob_interfaces::srv::Turn_Request>()
{
  return frob_interfaces::srv::builder::Init_Turn_Request_angle();
}

}  // namespace frob_interfaces


namespace frob_interfaces
{

namespace srv
{

namespace builder
{

class Init_Turn_Response_success
{
public:
  Init_Turn_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::frob_interfaces::srv::Turn_Response success(::frob_interfaces::srv::Turn_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::frob_interfaces::srv::Turn_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::frob_interfaces::srv::Turn_Response>()
{
  return frob_interfaces::srv::builder::Init_Turn_Response_success();
}

}  // namespace frob_interfaces


namespace frob_interfaces
{

namespace srv
{

namespace builder
{

class Init_Turn_Event_response
{
public:
  explicit Init_Turn_Event_response(::frob_interfaces::srv::Turn_Event & msg)
  : msg_(msg)
  {}
  ::frob_interfaces::srv::Turn_Event response(::frob_interfaces::srv::Turn_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::frob_interfaces::srv::Turn_Event msg_;
};

class Init_Turn_Event_request
{
public:
  explicit Init_Turn_Event_request(::frob_interfaces::srv::Turn_Event & msg)
  : msg_(msg)
  {}
  Init_Turn_Event_response request(::frob_interfaces::srv::Turn_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Turn_Event_response(msg_);
  }

private:
  ::frob_interfaces::srv::Turn_Event msg_;
};

class Init_Turn_Event_info
{
public:
  Init_Turn_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Turn_Event_request info(::frob_interfaces::srv::Turn_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Turn_Event_request(msg_);
  }

private:
  ::frob_interfaces::srv::Turn_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::frob_interfaces::srv::Turn_Event>()
{
  return frob_interfaces::srv::builder::Init_Turn_Event_info();
}

}  // namespace frob_interfaces

#endif  // FROB_INTERFACES__SRV__DETAIL__TURN__BUILDER_HPP_
