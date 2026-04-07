// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pm_skills_interfaces:srv/MeasureFrameVision.idl
// generated code does not contain a copyright notice
#include "pm_skills_interfaces/srv/detail/measure_frame_vision__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `frame_name`
// Member `vision_process_file_name`
#include "rosidl_runtime_c/string_functions.h"

bool
pm_skills_interfaces__srv__MeasureFrameVision_Request__init(pm_skills_interfaces__srv__MeasureFrameVision_Request * msg)
{
  if (!msg) {
    return false;
  }
  // frame_name
  if (!rosidl_runtime_c__String__init(&msg->frame_name)) {
    pm_skills_interfaces__srv__MeasureFrameVision_Request__fini(msg);
    return false;
  }
  // vision_process_file_name
  if (!rosidl_runtime_c__String__init(&msg->vision_process_file_name)) {
    pm_skills_interfaces__srv__MeasureFrameVision_Request__fini(msg);
    return false;
  }
  return true;
}

void
pm_skills_interfaces__srv__MeasureFrameVision_Request__fini(pm_skills_interfaces__srv__MeasureFrameVision_Request * msg)
{
  if (!msg) {
    return;
  }
  // frame_name
  rosidl_runtime_c__String__fini(&msg->frame_name);
  // vision_process_file_name
  rosidl_runtime_c__String__fini(&msg->vision_process_file_name);
}

bool
pm_skills_interfaces__srv__MeasureFrameVision_Request__are_equal(const pm_skills_interfaces__srv__MeasureFrameVision_Request * lhs, const pm_skills_interfaces__srv__MeasureFrameVision_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // frame_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->frame_name), &(rhs->frame_name)))
  {
    return false;
  }
  // vision_process_file_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->vision_process_file_name), &(rhs->vision_process_file_name)))
  {
    return false;
  }
  return true;
}

bool
pm_skills_interfaces__srv__MeasureFrameVision_Request__copy(
  const pm_skills_interfaces__srv__MeasureFrameVision_Request * input,
  pm_skills_interfaces__srv__MeasureFrameVision_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // frame_name
  if (!rosidl_runtime_c__String__copy(
      &(input->frame_name), &(output->frame_name)))
  {
    return false;
  }
  // vision_process_file_name
  if (!rosidl_runtime_c__String__copy(
      &(input->vision_process_file_name), &(output->vision_process_file_name)))
  {
    return false;
  }
  return true;
}

pm_skills_interfaces__srv__MeasureFrameVision_Request *
pm_skills_interfaces__srv__MeasureFrameVision_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pm_skills_interfaces__srv__MeasureFrameVision_Request * msg = (pm_skills_interfaces__srv__MeasureFrameVision_Request *)allocator.allocate(sizeof(pm_skills_interfaces__srv__MeasureFrameVision_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pm_skills_interfaces__srv__MeasureFrameVision_Request));
  bool success = pm_skills_interfaces__srv__MeasureFrameVision_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pm_skills_interfaces__srv__MeasureFrameVision_Request__destroy(pm_skills_interfaces__srv__MeasureFrameVision_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pm_skills_interfaces__srv__MeasureFrameVision_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence__init(pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pm_skills_interfaces__srv__MeasureFrameVision_Request * data = NULL;

  if (size) {
    data = (pm_skills_interfaces__srv__MeasureFrameVision_Request *)allocator.zero_allocate(size, sizeof(pm_skills_interfaces__srv__MeasureFrameVision_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pm_skills_interfaces__srv__MeasureFrameVision_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pm_skills_interfaces__srv__MeasureFrameVision_Request__fini(&data[i - 1]);
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
pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence__fini(pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence * array)
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
      pm_skills_interfaces__srv__MeasureFrameVision_Request__fini(&array->data[i]);
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

pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence *
pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence * array = (pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence *)allocator.allocate(sizeof(pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence__destroy(pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence__are_equal(const pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence * lhs, const pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pm_skills_interfaces__srv__MeasureFrameVision_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence__copy(
  const pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence * input,
  pm_skills_interfaces__srv__MeasureFrameVision_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pm_skills_interfaces__srv__MeasureFrameVision_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pm_skills_interfaces__srv__MeasureFrameVision_Request * data =
      (pm_skills_interfaces__srv__MeasureFrameVision_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pm_skills_interfaces__srv__MeasureFrameVision_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pm_skills_interfaces__srv__MeasureFrameVision_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pm_skills_interfaces__srv__MeasureFrameVision_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result_vector`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `message`
// Member `component_name`
// Member `component_uuid`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `vision_response`
#include "pm_vision_interfaces/msg/detail/vision_response__functions.h"

bool
pm_skills_interfaces__srv__MeasureFrameVision_Response__init(pm_skills_interfaces__srv__MeasureFrameVision_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // result_vector
  if (!geometry_msgs__msg__Vector3__init(&msg->result_vector)) {
    pm_skills_interfaces__srv__MeasureFrameVision_Response__fini(msg);
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    pm_skills_interfaces__srv__MeasureFrameVision_Response__fini(msg);
    return false;
  }
  // vision_response
  if (!pm_vision_interfaces__msg__VisionResponse__init(&msg->vision_response)) {
    pm_skills_interfaces__srv__MeasureFrameVision_Response__fini(msg);
    return false;
  }
  // component_name
  if (!rosidl_runtime_c__String__init(&msg->component_name)) {
    pm_skills_interfaces__srv__MeasureFrameVision_Response__fini(msg);
    return false;
  }
  // component_uuid
  if (!rosidl_runtime_c__String__init(&msg->component_uuid)) {
    pm_skills_interfaces__srv__MeasureFrameVision_Response__fini(msg);
    return false;
  }
  return true;
}

void
pm_skills_interfaces__srv__MeasureFrameVision_Response__fini(pm_skills_interfaces__srv__MeasureFrameVision_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // result_vector
  geometry_msgs__msg__Vector3__fini(&msg->result_vector);
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // vision_response
  pm_vision_interfaces__msg__VisionResponse__fini(&msg->vision_response);
  // component_name
  rosidl_runtime_c__String__fini(&msg->component_name);
  // component_uuid
  rosidl_runtime_c__String__fini(&msg->component_uuid);
}

bool
pm_skills_interfaces__srv__MeasureFrameVision_Response__are_equal(const pm_skills_interfaces__srv__MeasureFrameVision_Response * lhs, const pm_skills_interfaces__srv__MeasureFrameVision_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // result_vector
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->result_vector), &(rhs->result_vector)))
  {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // vision_response
  if (!pm_vision_interfaces__msg__VisionResponse__are_equal(
      &(lhs->vision_response), &(rhs->vision_response)))
  {
    return false;
  }
  // component_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->component_name), &(rhs->component_name)))
  {
    return false;
  }
  // component_uuid
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->component_uuid), &(rhs->component_uuid)))
  {
    return false;
  }
  return true;
}

bool
pm_skills_interfaces__srv__MeasureFrameVision_Response__copy(
  const pm_skills_interfaces__srv__MeasureFrameVision_Response * input,
  pm_skills_interfaces__srv__MeasureFrameVision_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // result_vector
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->result_vector), &(output->result_vector)))
  {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // vision_response
  if (!pm_vision_interfaces__msg__VisionResponse__copy(
      &(input->vision_response), &(output->vision_response)))
  {
    return false;
  }
  // component_name
  if (!rosidl_runtime_c__String__copy(
      &(input->component_name), &(output->component_name)))
  {
    return false;
  }
  // component_uuid
  if (!rosidl_runtime_c__String__copy(
      &(input->component_uuid), &(output->component_uuid)))
  {
    return false;
  }
  return true;
}

pm_skills_interfaces__srv__MeasureFrameVision_Response *
pm_skills_interfaces__srv__MeasureFrameVision_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pm_skills_interfaces__srv__MeasureFrameVision_Response * msg = (pm_skills_interfaces__srv__MeasureFrameVision_Response *)allocator.allocate(sizeof(pm_skills_interfaces__srv__MeasureFrameVision_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pm_skills_interfaces__srv__MeasureFrameVision_Response));
  bool success = pm_skills_interfaces__srv__MeasureFrameVision_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pm_skills_interfaces__srv__MeasureFrameVision_Response__destroy(pm_skills_interfaces__srv__MeasureFrameVision_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pm_skills_interfaces__srv__MeasureFrameVision_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence__init(pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pm_skills_interfaces__srv__MeasureFrameVision_Response * data = NULL;

  if (size) {
    data = (pm_skills_interfaces__srv__MeasureFrameVision_Response *)allocator.zero_allocate(size, sizeof(pm_skills_interfaces__srv__MeasureFrameVision_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pm_skills_interfaces__srv__MeasureFrameVision_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pm_skills_interfaces__srv__MeasureFrameVision_Response__fini(&data[i - 1]);
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
pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence__fini(pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence * array)
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
      pm_skills_interfaces__srv__MeasureFrameVision_Response__fini(&array->data[i]);
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

pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence *
pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence * array = (pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence *)allocator.allocate(sizeof(pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence__destroy(pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence__are_equal(const pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence * lhs, const pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pm_skills_interfaces__srv__MeasureFrameVision_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence__copy(
  const pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence * input,
  pm_skills_interfaces__srv__MeasureFrameVision_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pm_skills_interfaces__srv__MeasureFrameVision_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pm_skills_interfaces__srv__MeasureFrameVision_Response * data =
      (pm_skills_interfaces__srv__MeasureFrameVision_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pm_skills_interfaces__srv__MeasureFrameVision_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pm_skills_interfaces__srv__MeasureFrameVision_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pm_skills_interfaces__srv__MeasureFrameVision_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
