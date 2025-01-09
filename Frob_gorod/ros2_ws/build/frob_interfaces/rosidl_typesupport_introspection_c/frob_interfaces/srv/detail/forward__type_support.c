// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from frob_interfaces:srv/Forward.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "frob_interfaces/srv/detail/forward__rosidl_typesupport_introspection_c.h"
#include "frob_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "frob_interfaces/srv/detail/forward__functions.h"
#include "frob_interfaces/srv/detail/forward__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void frob_interfaces__srv__Forward_Request__rosidl_typesupport_introspection_c__Forward_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  frob_interfaces__srv__Forward_Request__init(message_memory);
}

void frob_interfaces__srv__Forward_Request__rosidl_typesupport_introspection_c__Forward_Request_fini_function(void * message_memory)
{
  frob_interfaces__srv__Forward_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember frob_interfaces__srv__Forward_Request__rosidl_typesupport_introspection_c__Forward_Request_message_member_array[2] = {
  {
    "dist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(frob_interfaces__srv__Forward_Request, dist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(frob_interfaces__srv__Forward_Request, speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers frob_interfaces__srv__Forward_Request__rosidl_typesupport_introspection_c__Forward_Request_message_members = {
  "frob_interfaces__srv",  // message namespace
  "Forward_Request",  // message name
  2,  // number of fields
  sizeof(frob_interfaces__srv__Forward_Request),
  false,  // has_any_key_member_
  frob_interfaces__srv__Forward_Request__rosidl_typesupport_introspection_c__Forward_Request_message_member_array,  // message members
  frob_interfaces__srv__Forward_Request__rosidl_typesupport_introspection_c__Forward_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  frob_interfaces__srv__Forward_Request__rosidl_typesupport_introspection_c__Forward_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t frob_interfaces__srv__Forward_Request__rosidl_typesupport_introspection_c__Forward_Request_message_type_support_handle = {
  0,
  &frob_interfaces__srv__Forward_Request__rosidl_typesupport_introspection_c__Forward_Request_message_members,
  get_message_typesupport_handle_function,
  &frob_interfaces__srv__Forward_Request__get_type_hash,
  &frob_interfaces__srv__Forward_Request__get_type_description,
  &frob_interfaces__srv__Forward_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_frob_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Forward_Request)() {
  if (!frob_interfaces__srv__Forward_Request__rosidl_typesupport_introspection_c__Forward_Request_message_type_support_handle.typesupport_identifier) {
    frob_interfaces__srv__Forward_Request__rosidl_typesupport_introspection_c__Forward_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &frob_interfaces__srv__Forward_Request__rosidl_typesupport_introspection_c__Forward_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "frob_interfaces/srv/detail/forward__rosidl_typesupport_introspection_c.h"
// already included above
// #include "frob_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "frob_interfaces/srv/detail/forward__functions.h"
// already included above
// #include "frob_interfaces/srv/detail/forward__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void frob_interfaces__srv__Forward_Response__rosidl_typesupport_introspection_c__Forward_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  frob_interfaces__srv__Forward_Response__init(message_memory);
}

void frob_interfaces__srv__Forward_Response__rosidl_typesupport_introspection_c__Forward_Response_fini_function(void * message_memory)
{
  frob_interfaces__srv__Forward_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember frob_interfaces__srv__Forward_Response__rosidl_typesupport_introspection_c__Forward_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(frob_interfaces__srv__Forward_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers frob_interfaces__srv__Forward_Response__rosidl_typesupport_introspection_c__Forward_Response_message_members = {
  "frob_interfaces__srv",  // message namespace
  "Forward_Response",  // message name
  1,  // number of fields
  sizeof(frob_interfaces__srv__Forward_Response),
  false,  // has_any_key_member_
  frob_interfaces__srv__Forward_Response__rosidl_typesupport_introspection_c__Forward_Response_message_member_array,  // message members
  frob_interfaces__srv__Forward_Response__rosidl_typesupport_introspection_c__Forward_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  frob_interfaces__srv__Forward_Response__rosidl_typesupport_introspection_c__Forward_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t frob_interfaces__srv__Forward_Response__rosidl_typesupport_introspection_c__Forward_Response_message_type_support_handle = {
  0,
  &frob_interfaces__srv__Forward_Response__rosidl_typesupport_introspection_c__Forward_Response_message_members,
  get_message_typesupport_handle_function,
  &frob_interfaces__srv__Forward_Response__get_type_hash,
  &frob_interfaces__srv__Forward_Response__get_type_description,
  &frob_interfaces__srv__Forward_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_frob_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Forward_Response)() {
  if (!frob_interfaces__srv__Forward_Response__rosidl_typesupport_introspection_c__Forward_Response_message_type_support_handle.typesupport_identifier) {
    frob_interfaces__srv__Forward_Response__rosidl_typesupport_introspection_c__Forward_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &frob_interfaces__srv__Forward_Response__rosidl_typesupport_introspection_c__Forward_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "frob_interfaces/srv/detail/forward__rosidl_typesupport_introspection_c.h"
// already included above
// #include "frob_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "frob_interfaces/srv/detail/forward__functions.h"
// already included above
// #include "frob_interfaces/srv/detail/forward__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "frob_interfaces/srv/forward.h"
// Member `request`
// Member `response`
// already included above
// #include "frob_interfaces/srv/detail/forward__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  frob_interfaces__srv__Forward_Event__init(message_memory);
}

void frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_fini_function(void * message_memory)
{
  frob_interfaces__srv__Forward_Event__fini(message_memory);
}

size_t frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__size_function__Forward_Event__request(
  const void * untyped_member)
{
  const frob_interfaces__srv__Forward_Request__Sequence * member =
    (const frob_interfaces__srv__Forward_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__get_const_function__Forward_Event__request(
  const void * untyped_member, size_t index)
{
  const frob_interfaces__srv__Forward_Request__Sequence * member =
    (const frob_interfaces__srv__Forward_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__get_function__Forward_Event__request(
  void * untyped_member, size_t index)
{
  frob_interfaces__srv__Forward_Request__Sequence * member =
    (frob_interfaces__srv__Forward_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__fetch_function__Forward_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const frob_interfaces__srv__Forward_Request * item =
    ((const frob_interfaces__srv__Forward_Request *)
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__get_const_function__Forward_Event__request(untyped_member, index));
  frob_interfaces__srv__Forward_Request * value =
    (frob_interfaces__srv__Forward_Request *)(untyped_value);
  *value = *item;
}

void frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__assign_function__Forward_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  frob_interfaces__srv__Forward_Request * item =
    ((frob_interfaces__srv__Forward_Request *)
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__get_function__Forward_Event__request(untyped_member, index));
  const frob_interfaces__srv__Forward_Request * value =
    (const frob_interfaces__srv__Forward_Request *)(untyped_value);
  *item = *value;
}

bool frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__resize_function__Forward_Event__request(
  void * untyped_member, size_t size)
{
  frob_interfaces__srv__Forward_Request__Sequence * member =
    (frob_interfaces__srv__Forward_Request__Sequence *)(untyped_member);
  frob_interfaces__srv__Forward_Request__Sequence__fini(member);
  return frob_interfaces__srv__Forward_Request__Sequence__init(member, size);
}

size_t frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__size_function__Forward_Event__response(
  const void * untyped_member)
{
  const frob_interfaces__srv__Forward_Response__Sequence * member =
    (const frob_interfaces__srv__Forward_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__get_const_function__Forward_Event__response(
  const void * untyped_member, size_t index)
{
  const frob_interfaces__srv__Forward_Response__Sequence * member =
    (const frob_interfaces__srv__Forward_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__get_function__Forward_Event__response(
  void * untyped_member, size_t index)
{
  frob_interfaces__srv__Forward_Response__Sequence * member =
    (frob_interfaces__srv__Forward_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__fetch_function__Forward_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const frob_interfaces__srv__Forward_Response * item =
    ((const frob_interfaces__srv__Forward_Response *)
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__get_const_function__Forward_Event__response(untyped_member, index));
  frob_interfaces__srv__Forward_Response * value =
    (frob_interfaces__srv__Forward_Response *)(untyped_value);
  *value = *item;
}

void frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__assign_function__Forward_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  frob_interfaces__srv__Forward_Response * item =
    ((frob_interfaces__srv__Forward_Response *)
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__get_function__Forward_Event__response(untyped_member, index));
  const frob_interfaces__srv__Forward_Response * value =
    (const frob_interfaces__srv__Forward_Response *)(untyped_value);
  *item = *value;
}

bool frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__resize_function__Forward_Event__response(
  void * untyped_member, size_t size)
{
  frob_interfaces__srv__Forward_Response__Sequence * member =
    (frob_interfaces__srv__Forward_Response__Sequence *)(untyped_member);
  frob_interfaces__srv__Forward_Response__Sequence__fini(member);
  return frob_interfaces__srv__Forward_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(frob_interfaces__srv__Forward_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(frob_interfaces__srv__Forward_Event, request),  // bytes offset in struct
    NULL,  // default value
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__size_function__Forward_Event__request,  // size() function pointer
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__get_const_function__Forward_Event__request,  // get_const(index) function pointer
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__get_function__Forward_Event__request,  // get(index) function pointer
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__fetch_function__Forward_Event__request,  // fetch(index, &value) function pointer
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__assign_function__Forward_Event__request,  // assign(index, value) function pointer
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__resize_function__Forward_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(frob_interfaces__srv__Forward_Event, response),  // bytes offset in struct
    NULL,  // default value
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__size_function__Forward_Event__response,  // size() function pointer
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__get_const_function__Forward_Event__response,  // get_const(index) function pointer
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__get_function__Forward_Event__response,  // get(index) function pointer
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__fetch_function__Forward_Event__response,  // fetch(index, &value) function pointer
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__assign_function__Forward_Event__response,  // assign(index, value) function pointer
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__resize_function__Forward_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_message_members = {
  "frob_interfaces__srv",  // message namespace
  "Forward_Event",  // message name
  3,  // number of fields
  sizeof(frob_interfaces__srv__Forward_Event),
  false,  // has_any_key_member_
  frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_message_member_array,  // message members
  frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_message_type_support_handle = {
  0,
  &frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_message_members,
  get_message_typesupport_handle_function,
  &frob_interfaces__srv__Forward_Event__get_type_hash,
  &frob_interfaces__srv__Forward_Event__get_type_description,
  &frob_interfaces__srv__Forward_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_frob_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Forward_Event)() {
  frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Forward_Request)();
  frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Forward_Response)();
  if (!frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_message_type_support_handle.typesupport_identifier) {
    frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "frob_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "frob_interfaces/srv/detail/forward__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers frob_interfaces__srv__detail__forward__rosidl_typesupport_introspection_c__Forward_service_members = {
  "frob_interfaces__srv",  // service namespace
  "Forward",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // frob_interfaces__srv__detail__forward__rosidl_typesupport_introspection_c__Forward_Request_message_type_support_handle,
  NULL,  // response message
  // frob_interfaces__srv__detail__forward__rosidl_typesupport_introspection_c__Forward_Response_message_type_support_handle
  NULL  // event_message
  // frob_interfaces__srv__detail__forward__rosidl_typesupport_introspection_c__Forward_Response_message_type_support_handle
};


static rosidl_service_type_support_t frob_interfaces__srv__detail__forward__rosidl_typesupport_introspection_c__Forward_service_type_support_handle = {
  0,
  &frob_interfaces__srv__detail__forward__rosidl_typesupport_introspection_c__Forward_service_members,
  get_service_typesupport_handle_function,
  &frob_interfaces__srv__Forward_Request__rosidl_typesupport_introspection_c__Forward_Request_message_type_support_handle,
  &frob_interfaces__srv__Forward_Response__rosidl_typesupport_introspection_c__Forward_Response_message_type_support_handle,
  &frob_interfaces__srv__Forward_Event__rosidl_typesupport_introspection_c__Forward_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    frob_interfaces,
    srv,
    Forward
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    frob_interfaces,
    srv,
    Forward
  ),
  &frob_interfaces__srv__Forward__get_type_hash,
  &frob_interfaces__srv__Forward__get_type_description,
  &frob_interfaces__srv__Forward__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Forward_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Forward_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Forward_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_frob_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Forward)(void) {
  if (!frob_interfaces__srv__detail__forward__rosidl_typesupport_introspection_c__Forward_service_type_support_handle.typesupport_identifier) {
    frob_interfaces__srv__detail__forward__rosidl_typesupport_introspection_c__Forward_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)frob_interfaces__srv__detail__forward__rosidl_typesupport_introspection_c__Forward_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Forward_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Forward_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Forward_Event)()->data;
  }

  return &frob_interfaces__srv__detail__forward__rosidl_typesupport_introspection_c__Forward_service_type_support_handle;
}
