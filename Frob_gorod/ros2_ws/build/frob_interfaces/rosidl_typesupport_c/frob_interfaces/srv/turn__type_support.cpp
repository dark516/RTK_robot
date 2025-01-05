// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from frob_interfaces:srv/Turn.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "frob_interfaces/srv/detail/turn__struct.h"
#include "frob_interfaces/srv/detail/turn__type_support.h"
#include "frob_interfaces/srv/detail/turn__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace frob_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _Turn_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_Request_type_support_ids_t;

static const _Turn_Request_type_support_ids_t _Turn_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_Request_type_support_symbol_names_t _Turn_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, frob_interfaces, srv, Turn_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Turn_Request)),
  }
};

typedef struct _Turn_Request_type_support_data_t
{
  void * data[2];
} _Turn_Request_type_support_data_t;

static _Turn_Request_type_support_data_t _Turn_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_Request_message_typesupport_map = {
  2,
  "frob_interfaces",
  &_Turn_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Turn_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Turn_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Turn_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &frob_interfaces__srv__Turn_Request__get_type_hash,
  &frob_interfaces__srv__Turn_Request__get_type_description,
  &frob_interfaces__srv__Turn_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace frob_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, frob_interfaces, srv, Turn_Request)() {
  return &::frob_interfaces::srv::rosidl_typesupport_c::Turn_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "frob_interfaces/srv/detail/turn__struct.h"
// already included above
// #include "frob_interfaces/srv/detail/turn__type_support.h"
// already included above
// #include "frob_interfaces/srv/detail/turn__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace frob_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _Turn_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_Response_type_support_ids_t;

static const _Turn_Response_type_support_ids_t _Turn_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_Response_type_support_symbol_names_t _Turn_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, frob_interfaces, srv, Turn_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Turn_Response)),
  }
};

typedef struct _Turn_Response_type_support_data_t
{
  void * data[2];
} _Turn_Response_type_support_data_t;

static _Turn_Response_type_support_data_t _Turn_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_Response_message_typesupport_map = {
  2,
  "frob_interfaces",
  &_Turn_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Turn_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Turn_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Turn_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &frob_interfaces__srv__Turn_Response__get_type_hash,
  &frob_interfaces__srv__Turn_Response__get_type_description,
  &frob_interfaces__srv__Turn_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace frob_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, frob_interfaces, srv, Turn_Response)() {
  return &::frob_interfaces::srv::rosidl_typesupport_c::Turn_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "frob_interfaces/srv/detail/turn__struct.h"
// already included above
// #include "frob_interfaces/srv/detail/turn__type_support.h"
// already included above
// #include "frob_interfaces/srv/detail/turn__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace frob_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _Turn_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_Event_type_support_ids_t;

static const _Turn_Event_type_support_ids_t _Turn_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_Event_type_support_symbol_names_t _Turn_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, frob_interfaces, srv, Turn_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Turn_Event)),
  }
};

typedef struct _Turn_Event_type_support_data_t
{
  void * data[2];
} _Turn_Event_type_support_data_t;

static _Turn_Event_type_support_data_t _Turn_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_Event_message_typesupport_map = {
  2,
  "frob_interfaces",
  &_Turn_Event_message_typesupport_ids.typesupport_identifier[0],
  &_Turn_Event_message_typesupport_symbol_names.symbol_name[0],
  &_Turn_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Turn_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &frob_interfaces__srv__Turn_Event__get_type_hash,
  &frob_interfaces__srv__Turn_Event__get_type_description,
  &frob_interfaces__srv__Turn_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace frob_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, frob_interfaces, srv, Turn_Event)() {
  return &::frob_interfaces::srv::rosidl_typesupport_c::Turn_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "frob_interfaces/srv/detail/turn__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace frob_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _Turn_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_type_support_ids_t;

static const _Turn_type_support_ids_t _Turn_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_type_support_symbol_names_t _Turn_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, frob_interfaces, srv, Turn)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, frob_interfaces, srv, Turn)),
  }
};

typedef struct _Turn_type_support_data_t
{
  void * data[2];
} _Turn_type_support_data_t;

static _Turn_type_support_data_t _Turn_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_service_typesupport_map = {
  2,
  "frob_interfaces",
  &_Turn_service_typesupport_ids.typesupport_identifier[0],
  &_Turn_service_typesupport_symbol_names.symbol_name[0],
  &_Turn_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Turn_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &Turn_Request_message_type_support_handle,
  &Turn_Response_message_type_support_handle,
  &Turn_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    frob_interfaces,
    srv,
    Turn
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    frob_interfaces,
    srv,
    Turn
  ),
  &frob_interfaces__srv__Turn__get_type_hash,
  &frob_interfaces__srv__Turn__get_type_description,
  &frob_interfaces__srv__Turn__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace frob_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, frob_interfaces, srv, Turn)() {
  return &::frob_interfaces::srv::rosidl_typesupport_c::Turn_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
