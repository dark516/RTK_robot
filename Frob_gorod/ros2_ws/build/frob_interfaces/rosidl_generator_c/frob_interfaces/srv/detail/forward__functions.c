// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from frob_interfaces:srv/Forward.idl
// generated code does not contain a copyright notice
#include "frob_interfaces/srv/detail/forward__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
frob_interfaces__srv__Forward_Request__init(frob_interfaces__srv__Forward_Request * msg)
{
  if (!msg) {
    return false;
  }
  // dist
  // speed
  return true;
}

void
frob_interfaces__srv__Forward_Request__fini(frob_interfaces__srv__Forward_Request * msg)
{
  if (!msg) {
    return;
  }
  // dist
  // speed
}

bool
frob_interfaces__srv__Forward_Request__are_equal(const frob_interfaces__srv__Forward_Request * lhs, const frob_interfaces__srv__Forward_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // dist
  if (lhs->dist != rhs->dist) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  return true;
}

bool
frob_interfaces__srv__Forward_Request__copy(
  const frob_interfaces__srv__Forward_Request * input,
  frob_interfaces__srv__Forward_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // dist
  output->dist = input->dist;
  // speed
  output->speed = input->speed;
  return true;
}

frob_interfaces__srv__Forward_Request *
frob_interfaces__srv__Forward_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  frob_interfaces__srv__Forward_Request * msg = (frob_interfaces__srv__Forward_Request *)allocator.allocate(sizeof(frob_interfaces__srv__Forward_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(frob_interfaces__srv__Forward_Request));
  bool success = frob_interfaces__srv__Forward_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
frob_interfaces__srv__Forward_Request__destroy(frob_interfaces__srv__Forward_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    frob_interfaces__srv__Forward_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
frob_interfaces__srv__Forward_Request__Sequence__init(frob_interfaces__srv__Forward_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  frob_interfaces__srv__Forward_Request * data = NULL;

  if (size) {
    data = (frob_interfaces__srv__Forward_Request *)allocator.zero_allocate(size, sizeof(frob_interfaces__srv__Forward_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = frob_interfaces__srv__Forward_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        frob_interfaces__srv__Forward_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
frob_interfaces__srv__Forward_Request__Sequence__fini(frob_interfaces__srv__Forward_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      frob_interfaces__srv__Forward_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

frob_interfaces__srv__Forward_Request__Sequence *
frob_interfaces__srv__Forward_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  frob_interfaces__srv__Forward_Request__Sequence * array = (frob_interfaces__srv__Forward_Request__Sequence *)allocator.allocate(sizeof(frob_interfaces__srv__Forward_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = frob_interfaces__srv__Forward_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
frob_interfaces__srv__Forward_Request__Sequence__destroy(frob_interfaces__srv__Forward_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    frob_interfaces__srv__Forward_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
frob_interfaces__srv__Forward_Request__Sequence__are_equal(const frob_interfaces__srv__Forward_Request__Sequence * lhs, const frob_interfaces__srv__Forward_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!frob_interfaces__srv__Forward_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
frob_interfaces__srv__Forward_Request__Sequence__copy(
  const frob_interfaces__srv__Forward_Request__Sequence * input,
  frob_interfaces__srv__Forward_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(frob_interfaces__srv__Forward_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    frob_interfaces__srv__Forward_Request * data =
      (frob_interfaces__srv__Forward_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!frob_interfaces__srv__Forward_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          frob_interfaces__srv__Forward_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!frob_interfaces__srv__Forward_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
frob_interfaces__srv__Forward_Response__init(frob_interfaces__srv__Forward_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
frob_interfaces__srv__Forward_Response__fini(frob_interfaces__srv__Forward_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
frob_interfaces__srv__Forward_Response__are_equal(const frob_interfaces__srv__Forward_Response * lhs, const frob_interfaces__srv__Forward_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
frob_interfaces__srv__Forward_Response__copy(
  const frob_interfaces__srv__Forward_Response * input,
  frob_interfaces__srv__Forward_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

frob_interfaces__srv__Forward_Response *
frob_interfaces__srv__Forward_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  frob_interfaces__srv__Forward_Response * msg = (frob_interfaces__srv__Forward_Response *)allocator.allocate(sizeof(frob_interfaces__srv__Forward_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(frob_interfaces__srv__Forward_Response));
  bool success = frob_interfaces__srv__Forward_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
frob_interfaces__srv__Forward_Response__destroy(frob_interfaces__srv__Forward_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    frob_interfaces__srv__Forward_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
frob_interfaces__srv__Forward_Response__Sequence__init(frob_interfaces__srv__Forward_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  frob_interfaces__srv__Forward_Response * data = NULL;

  if (size) {
    data = (frob_interfaces__srv__Forward_Response *)allocator.zero_allocate(size, sizeof(frob_interfaces__srv__Forward_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = frob_interfaces__srv__Forward_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        frob_interfaces__srv__Forward_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
frob_interfaces__srv__Forward_Response__Sequence__fini(frob_interfaces__srv__Forward_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      frob_interfaces__srv__Forward_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

frob_interfaces__srv__Forward_Response__Sequence *
frob_interfaces__srv__Forward_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  frob_interfaces__srv__Forward_Response__Sequence * array = (frob_interfaces__srv__Forward_Response__Sequence *)allocator.allocate(sizeof(frob_interfaces__srv__Forward_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = frob_interfaces__srv__Forward_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
frob_interfaces__srv__Forward_Response__Sequence__destroy(frob_interfaces__srv__Forward_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    frob_interfaces__srv__Forward_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
frob_interfaces__srv__Forward_Response__Sequence__are_equal(const frob_interfaces__srv__Forward_Response__Sequence * lhs, const frob_interfaces__srv__Forward_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!frob_interfaces__srv__Forward_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
frob_interfaces__srv__Forward_Response__Sequence__copy(
  const frob_interfaces__srv__Forward_Response__Sequence * input,
  frob_interfaces__srv__Forward_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(frob_interfaces__srv__Forward_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    frob_interfaces__srv__Forward_Response * data =
      (frob_interfaces__srv__Forward_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!frob_interfaces__srv__Forward_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          frob_interfaces__srv__Forward_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!frob_interfaces__srv__Forward_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "frob_interfaces/srv/detail/forward__functions.h"

bool
frob_interfaces__srv__Forward_Event__init(frob_interfaces__srv__Forward_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    frob_interfaces__srv__Forward_Event__fini(msg);
    return false;
  }
  // request
  if (!frob_interfaces__srv__Forward_Request__Sequence__init(&msg->request, 0)) {
    frob_interfaces__srv__Forward_Event__fini(msg);
    return false;
  }
  // response
  if (!frob_interfaces__srv__Forward_Response__Sequence__init(&msg->response, 0)) {
    frob_interfaces__srv__Forward_Event__fini(msg);
    return false;
  }
  return true;
}

void
frob_interfaces__srv__Forward_Event__fini(frob_interfaces__srv__Forward_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  frob_interfaces__srv__Forward_Request__Sequence__fini(&msg->request);
  // response
  frob_interfaces__srv__Forward_Response__Sequence__fini(&msg->response);
}

bool
frob_interfaces__srv__Forward_Event__are_equal(const frob_interfaces__srv__Forward_Event * lhs, const frob_interfaces__srv__Forward_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!frob_interfaces__srv__Forward_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!frob_interfaces__srv__Forward_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
frob_interfaces__srv__Forward_Event__copy(
  const frob_interfaces__srv__Forward_Event * input,
  frob_interfaces__srv__Forward_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!frob_interfaces__srv__Forward_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!frob_interfaces__srv__Forward_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

frob_interfaces__srv__Forward_Event *
frob_interfaces__srv__Forward_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  frob_interfaces__srv__Forward_Event * msg = (frob_interfaces__srv__Forward_Event *)allocator.allocate(sizeof(frob_interfaces__srv__Forward_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(frob_interfaces__srv__Forward_Event));
  bool success = frob_interfaces__srv__Forward_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
frob_interfaces__srv__Forward_Event__destroy(frob_interfaces__srv__Forward_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    frob_interfaces__srv__Forward_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
frob_interfaces__srv__Forward_Event__Sequence__init(frob_interfaces__srv__Forward_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  frob_interfaces__srv__Forward_Event * data = NULL;

  if (size) {
    data = (frob_interfaces__srv__Forward_Event *)allocator.zero_allocate(size, sizeof(frob_interfaces__srv__Forward_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = frob_interfaces__srv__Forward_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        frob_interfaces__srv__Forward_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
frob_interfaces__srv__Forward_Event__Sequence__fini(frob_interfaces__srv__Forward_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      frob_interfaces__srv__Forward_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

frob_interfaces__srv__Forward_Event__Sequence *
frob_interfaces__srv__Forward_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  frob_interfaces__srv__Forward_Event__Sequence * array = (frob_interfaces__srv__Forward_Event__Sequence *)allocator.allocate(sizeof(frob_interfaces__srv__Forward_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = frob_interfaces__srv__Forward_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
frob_interfaces__srv__Forward_Event__Sequence__destroy(frob_interfaces__srv__Forward_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    frob_interfaces__srv__Forward_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
frob_interfaces__srv__Forward_Event__Sequence__are_equal(const frob_interfaces__srv__Forward_Event__Sequence * lhs, const frob_interfaces__srv__Forward_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!frob_interfaces__srv__Forward_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
frob_interfaces__srv__Forward_Event__Sequence__copy(
  const frob_interfaces__srv__Forward_Event__Sequence * input,
  frob_interfaces__srv__Forward_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(frob_interfaces__srv__Forward_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    frob_interfaces__srv__Forward_Event * data =
      (frob_interfaces__srv__Forward_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!frob_interfaces__srv__Forward_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          frob_interfaces__srv__Forward_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!frob_interfaces__srv__Forward_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
