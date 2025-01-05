// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from frob_interfaces:srv/Forward.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "frob_interfaces/srv/forward.h"


#ifndef FROB_INTERFACES__SRV__DETAIL__FORWARD__STRUCT_H_
#define FROB_INTERFACES__SRV__DETAIL__FORWARD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Forward in the package frob_interfaces.
typedef struct frob_interfaces__srv__Forward_Request
{
  int32_t dist;
  int32_t speed;
} frob_interfaces__srv__Forward_Request;

// Struct for a sequence of frob_interfaces__srv__Forward_Request.
typedef struct frob_interfaces__srv__Forward_Request__Sequence
{
  frob_interfaces__srv__Forward_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} frob_interfaces__srv__Forward_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/Forward in the package frob_interfaces.
typedef struct frob_interfaces__srv__Forward_Response
{
  bool success;
} frob_interfaces__srv__Forward_Response;

// Struct for a sequence of frob_interfaces__srv__Forward_Response.
typedef struct frob_interfaces__srv__Forward_Response__Sequence
{
  frob_interfaces__srv__Forward_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} frob_interfaces__srv__Forward_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  frob_interfaces__srv__Forward_Event__request__MAX_SIZE = 1
};
// response
enum
{
  frob_interfaces__srv__Forward_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Forward in the package frob_interfaces.
typedef struct frob_interfaces__srv__Forward_Event
{
  service_msgs__msg__ServiceEventInfo info;
  frob_interfaces__srv__Forward_Request__Sequence request;
  frob_interfaces__srv__Forward_Response__Sequence response;
} frob_interfaces__srv__Forward_Event;

// Struct for a sequence of frob_interfaces__srv__Forward_Event.
typedef struct frob_interfaces__srv__Forward_Event__Sequence
{
  frob_interfaces__srv__Forward_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} frob_interfaces__srv__Forward_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FROB_INTERFACES__SRV__DETAIL__FORWARD__STRUCT_H_
