// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from frob_interfaces:srv/Forward.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "frob_interfaces/srv/forward.h"


#ifndef FROB_INTERFACES__SRV__DETAIL__FORWARD__FUNCTIONS_H_
#define FROB_INTERFACES__SRV__DETAIL__FORWARD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "frob_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "frob_interfaces/srv/detail/forward__struct.h"

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_type_hash_t *
frob_interfaces__srv__Forward__get_type_hash(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
frob_interfaces__srv__Forward__get_type_description(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_runtime_c__type_description__TypeSource *
frob_interfaces__srv__Forward__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
frob_interfaces__srv__Forward__get_type_description_sources(
  const rosidl_service_type_support_t * type_support);

/// Initialize srv/Forward message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * frob_interfaces__srv__Forward_Request
 * )) before or use
 * frob_interfaces__srv__Forward_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Request__init(frob_interfaces__srv__Forward_Request * msg);

/// Finalize srv/Forward message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
void
frob_interfaces__srv__Forward_Request__fini(frob_interfaces__srv__Forward_Request * msg);

/// Create srv/Forward message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * frob_interfaces__srv__Forward_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
frob_interfaces__srv__Forward_Request *
frob_interfaces__srv__Forward_Request__create(void);

/// Destroy srv/Forward message.
/**
 * It calls
 * frob_interfaces__srv__Forward_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
void
frob_interfaces__srv__Forward_Request__destroy(frob_interfaces__srv__Forward_Request * msg);

/// Check for srv/Forward message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Request__are_equal(const frob_interfaces__srv__Forward_Request * lhs, const frob_interfaces__srv__Forward_Request * rhs);

/// Copy a srv/Forward message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Request__copy(
  const frob_interfaces__srv__Forward_Request * input,
  frob_interfaces__srv__Forward_Request * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_type_hash_t *
frob_interfaces__srv__Forward_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
frob_interfaces__srv__Forward_Request__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_runtime_c__type_description__TypeSource *
frob_interfaces__srv__Forward_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
frob_interfaces__srv__Forward_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/Forward messages.
/**
 * It allocates the memory for the number of elements and calls
 * frob_interfaces__srv__Forward_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Request__Sequence__init(frob_interfaces__srv__Forward_Request__Sequence * array, size_t size);

/// Finalize array of srv/Forward messages.
/**
 * It calls
 * frob_interfaces__srv__Forward_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
void
frob_interfaces__srv__Forward_Request__Sequence__fini(frob_interfaces__srv__Forward_Request__Sequence * array);

/// Create array of srv/Forward messages.
/**
 * It allocates the memory for the array and calls
 * frob_interfaces__srv__Forward_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
frob_interfaces__srv__Forward_Request__Sequence *
frob_interfaces__srv__Forward_Request__Sequence__create(size_t size);

/// Destroy array of srv/Forward messages.
/**
 * It calls
 * frob_interfaces__srv__Forward_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
void
frob_interfaces__srv__Forward_Request__Sequence__destroy(frob_interfaces__srv__Forward_Request__Sequence * array);

/// Check for srv/Forward message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Request__Sequence__are_equal(const frob_interfaces__srv__Forward_Request__Sequence * lhs, const frob_interfaces__srv__Forward_Request__Sequence * rhs);

/// Copy an array of srv/Forward messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Request__Sequence__copy(
  const frob_interfaces__srv__Forward_Request__Sequence * input,
  frob_interfaces__srv__Forward_Request__Sequence * output);

/// Initialize srv/Forward message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * frob_interfaces__srv__Forward_Response
 * )) before or use
 * frob_interfaces__srv__Forward_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Response__init(frob_interfaces__srv__Forward_Response * msg);

/// Finalize srv/Forward message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
void
frob_interfaces__srv__Forward_Response__fini(frob_interfaces__srv__Forward_Response * msg);

/// Create srv/Forward message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * frob_interfaces__srv__Forward_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
frob_interfaces__srv__Forward_Response *
frob_interfaces__srv__Forward_Response__create(void);

/// Destroy srv/Forward message.
/**
 * It calls
 * frob_interfaces__srv__Forward_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
void
frob_interfaces__srv__Forward_Response__destroy(frob_interfaces__srv__Forward_Response * msg);

/// Check for srv/Forward message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Response__are_equal(const frob_interfaces__srv__Forward_Response * lhs, const frob_interfaces__srv__Forward_Response * rhs);

/// Copy a srv/Forward message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Response__copy(
  const frob_interfaces__srv__Forward_Response * input,
  frob_interfaces__srv__Forward_Response * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_type_hash_t *
frob_interfaces__srv__Forward_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
frob_interfaces__srv__Forward_Response__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_runtime_c__type_description__TypeSource *
frob_interfaces__srv__Forward_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
frob_interfaces__srv__Forward_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/Forward messages.
/**
 * It allocates the memory for the number of elements and calls
 * frob_interfaces__srv__Forward_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Response__Sequence__init(frob_interfaces__srv__Forward_Response__Sequence * array, size_t size);

/// Finalize array of srv/Forward messages.
/**
 * It calls
 * frob_interfaces__srv__Forward_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
void
frob_interfaces__srv__Forward_Response__Sequence__fini(frob_interfaces__srv__Forward_Response__Sequence * array);

/// Create array of srv/Forward messages.
/**
 * It allocates the memory for the array and calls
 * frob_interfaces__srv__Forward_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
frob_interfaces__srv__Forward_Response__Sequence *
frob_interfaces__srv__Forward_Response__Sequence__create(size_t size);

/// Destroy array of srv/Forward messages.
/**
 * It calls
 * frob_interfaces__srv__Forward_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
void
frob_interfaces__srv__Forward_Response__Sequence__destroy(frob_interfaces__srv__Forward_Response__Sequence * array);

/// Check for srv/Forward message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Response__Sequence__are_equal(const frob_interfaces__srv__Forward_Response__Sequence * lhs, const frob_interfaces__srv__Forward_Response__Sequence * rhs);

/// Copy an array of srv/Forward messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Response__Sequence__copy(
  const frob_interfaces__srv__Forward_Response__Sequence * input,
  frob_interfaces__srv__Forward_Response__Sequence * output);

/// Initialize srv/Forward message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * frob_interfaces__srv__Forward_Event
 * )) before or use
 * frob_interfaces__srv__Forward_Event__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Event__init(frob_interfaces__srv__Forward_Event * msg);

/// Finalize srv/Forward message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
void
frob_interfaces__srv__Forward_Event__fini(frob_interfaces__srv__Forward_Event * msg);

/// Create srv/Forward message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * frob_interfaces__srv__Forward_Event__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
frob_interfaces__srv__Forward_Event *
frob_interfaces__srv__Forward_Event__create(void);

/// Destroy srv/Forward message.
/**
 * It calls
 * frob_interfaces__srv__Forward_Event__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
void
frob_interfaces__srv__Forward_Event__destroy(frob_interfaces__srv__Forward_Event * msg);

/// Check for srv/Forward message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Event__are_equal(const frob_interfaces__srv__Forward_Event * lhs, const frob_interfaces__srv__Forward_Event * rhs);

/// Copy a srv/Forward message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Event__copy(
  const frob_interfaces__srv__Forward_Event * input,
  frob_interfaces__srv__Forward_Event * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_type_hash_t *
frob_interfaces__srv__Forward_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
frob_interfaces__srv__Forward_Event__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_runtime_c__type_description__TypeSource *
frob_interfaces__srv__Forward_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
frob_interfaces__srv__Forward_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/Forward messages.
/**
 * It allocates the memory for the number of elements and calls
 * frob_interfaces__srv__Forward_Event__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Event__Sequence__init(frob_interfaces__srv__Forward_Event__Sequence * array, size_t size);

/// Finalize array of srv/Forward messages.
/**
 * It calls
 * frob_interfaces__srv__Forward_Event__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
void
frob_interfaces__srv__Forward_Event__Sequence__fini(frob_interfaces__srv__Forward_Event__Sequence * array);

/// Create array of srv/Forward messages.
/**
 * It allocates the memory for the array and calls
 * frob_interfaces__srv__Forward_Event__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
frob_interfaces__srv__Forward_Event__Sequence *
frob_interfaces__srv__Forward_Event__Sequence__create(size_t size);

/// Destroy array of srv/Forward messages.
/**
 * It calls
 * frob_interfaces__srv__Forward_Event__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
void
frob_interfaces__srv__Forward_Event__Sequence__destroy(frob_interfaces__srv__Forward_Event__Sequence * array);

/// Check for srv/Forward message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Event__Sequence__are_equal(const frob_interfaces__srv__Forward_Event__Sequence * lhs, const frob_interfaces__srv__Forward_Event__Sequence * rhs);

/// Copy an array of srv/Forward messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_frob_interfaces
bool
frob_interfaces__srv__Forward_Event__Sequence__copy(
  const frob_interfaces__srv__Forward_Event__Sequence * input,
  frob_interfaces__srv__Forward_Event__Sequence * output);
#ifdef __cplusplus
}
#endif

#endif  // FROB_INTERFACES__SRV__DETAIL__FORWARD__FUNCTIONS_H_
