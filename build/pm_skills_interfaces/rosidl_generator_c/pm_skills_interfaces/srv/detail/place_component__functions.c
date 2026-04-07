// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pm_skills_interfaces:srv/PlaceComponent.idl
// generated code does not contain a copyright notice
#include "pm_skills_interfaces/srv/detail/place_component__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
pm_skills_interfaces__srv__PlaceComponent_Request__init(pm_skills_interfaces__srv__PlaceComponent_Request * msg)
{
  if (!msg) {
    return false;
  }
  // align_orientation
  // x_offset_um
  // y_offset_um
  // z_offset_um
  // rx_offset_deg
  // ry_offset_deg
  // rz_offset_deg
  return true;
}

void
pm_skills_interfaces__srv__PlaceComponent_Request__fini(pm_skills_interfaces__srv__PlaceComponent_Request * msg)
{
  if (!msg) {
    return;
  }
  // align_orientation
  // x_offset_um
  // y_offset_um
  // z_offset_um
  // rx_offset_deg
  // ry_offset_deg
  // rz_offset_deg
}

bool
pm_skills_interfaces__srv__PlaceComponent_Request__are_equal(const pm_skills_interfaces__srv__PlaceComponent_Request * lhs, const pm_skills_interfaces__srv__PlaceComponent_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // align_orientation
  if (lhs->align_orientation != rhs->align_orientation) {
    return false;
  }
  // x_offset_um
  if (lhs->x_offset_um != rhs->x_offset_um) {
    return false;
  }
  // y_offset_um
  if (lhs->y_offset_um != rhs->y_offset_um) {
    return false;
  }
  // z_offset_um
  if (lhs->z_offset_um != rhs->z_offset_um) {
    return false;
  }
  // rx_offset_deg
  if (lhs->rx_offset_deg != rhs->rx_offset_deg) {
    return false;
  }
  // ry_offset_deg
  if (lhs->ry_offset_deg != rhs->ry_offset_deg) {
    return false;
  }
  // rz_offset_deg
  if (lhs->rz_offset_deg != rhs->rz_offset_deg) {
    return false;
  }
  return true;
}

bool
pm_skills_interfaces__srv__PlaceComponent_Request__copy(
  const pm_skills_interfaces__srv__PlaceComponent_Request * input,
  pm_skills_interfaces__srv__PlaceComponent_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // align_orientation
  output->align_orientation = input->align_orientation;
  // x_offset_um
  output->x_offset_um = input->x_offset_um;
  // y_offset_um
  output->y_offset_um = input->y_offset_um;
  // z_offset_um
  output->z_offset_um = input->z_offset_um;
  // rx_offset_deg
  output->rx_offset_deg = input->rx_offset_deg;
  // ry_offset_deg
  output->ry_offset_deg = input->ry_offset_deg;
  // rz_offset_deg
  output->rz_offset_deg = input->rz_offset_deg;
  return true;
}

pm_skills_interfaces__srv__PlaceComponent_Request *
pm_skills_interfaces__srv__PlaceComponent_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pm_skills_interfaces__srv__PlaceComponent_Request * msg = (pm_skills_interfaces__srv__PlaceComponent_Request *)allocator.allocate(sizeof(pm_skills_interfaces__srv__PlaceComponent_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pm_skills_interfaces__srv__PlaceComponent_Request));
  bool success = pm_skills_interfaces__srv__PlaceComponent_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pm_skills_interfaces__srv__PlaceComponent_Request__destroy(pm_skills_interfaces__srv__PlaceComponent_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pm_skills_interfaces__srv__PlaceComponent_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pm_skills_interfaces__srv__PlaceComponent_Request__Sequence__init(pm_skills_interfaces__srv__PlaceComponent_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pm_skills_interfaces__srv__PlaceComponent_Request * data = NULL;

  if (size) {
    data = (pm_skills_interfaces__srv__PlaceComponent_Request *)allocator.zero_allocate(size, sizeof(pm_skills_interfaces__srv__PlaceComponent_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pm_skills_interfaces__srv__PlaceComponent_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pm_skills_interfaces__srv__PlaceComponent_Request__fini(&data[i - 1]);
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
pm_skills_interfaces__srv__PlaceComponent_Request__Sequence__fini(pm_skills_interfaces__srv__PlaceComponent_Request__Sequence * array)
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
      pm_skills_interfaces__srv__PlaceComponent_Request__fini(&array->data[i]);
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

pm_skills_interfaces__srv__PlaceComponent_Request__Sequence *
pm_skills_interfaces__srv__PlaceComponent_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pm_skills_interfaces__srv__PlaceComponent_Request__Sequence * array = (pm_skills_interfaces__srv__PlaceComponent_Request__Sequence *)allocator.allocate(sizeof(pm_skills_interfaces__srv__PlaceComponent_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pm_skills_interfaces__srv__PlaceComponent_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pm_skills_interfaces__srv__PlaceComponent_Request__Sequence__destroy(pm_skills_interfaces__srv__PlaceComponent_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pm_skills_interfaces__srv__PlaceComponent_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pm_skills_interfaces__srv__PlaceComponent_Request__Sequence__are_equal(const pm_skills_interfaces__srv__PlaceComponent_Request__Sequence * lhs, const pm_skills_interfaces__srv__PlaceComponent_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pm_skills_interfaces__srv__PlaceComponent_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pm_skills_interfaces__srv__PlaceComponent_Request__Sequence__copy(
  const pm_skills_interfaces__srv__PlaceComponent_Request__Sequence * input,
  pm_skills_interfaces__srv__PlaceComponent_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pm_skills_interfaces__srv__PlaceComponent_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pm_skills_interfaces__srv__PlaceComponent_Request * data =
      (pm_skills_interfaces__srv__PlaceComponent_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pm_skills_interfaces__srv__PlaceComponent_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pm_skills_interfaces__srv__PlaceComponent_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pm_skills_interfaces__srv__PlaceComponent_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
pm_skills_interfaces__srv__PlaceComponent_Response__init(pm_skills_interfaces__srv__PlaceComponent_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    pm_skills_interfaces__srv__PlaceComponent_Response__fini(msg);
    return false;
  }
  return true;
}

void
pm_skills_interfaces__srv__PlaceComponent_Response__fini(pm_skills_interfaces__srv__PlaceComponent_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
pm_skills_interfaces__srv__PlaceComponent_Response__are_equal(const pm_skills_interfaces__srv__PlaceComponent_Response * lhs, const pm_skills_interfaces__srv__PlaceComponent_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
pm_skills_interfaces__srv__PlaceComponent_Response__copy(
  const pm_skills_interfaces__srv__PlaceComponent_Response * input,
  pm_skills_interfaces__srv__PlaceComponent_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

pm_skills_interfaces__srv__PlaceComponent_Response *
pm_skills_interfaces__srv__PlaceComponent_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pm_skills_interfaces__srv__PlaceComponent_Response * msg = (pm_skills_interfaces__srv__PlaceComponent_Response *)allocator.allocate(sizeof(pm_skills_interfaces__srv__PlaceComponent_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pm_skills_interfaces__srv__PlaceComponent_Response));
  bool success = pm_skills_interfaces__srv__PlaceComponent_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pm_skills_interfaces__srv__PlaceComponent_Response__destroy(pm_skills_interfaces__srv__PlaceComponent_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pm_skills_interfaces__srv__PlaceComponent_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pm_skills_interfaces__srv__PlaceComponent_Response__Sequence__init(pm_skills_interfaces__srv__PlaceComponent_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pm_skills_interfaces__srv__PlaceComponent_Response * data = NULL;

  if (size) {
    data = (pm_skills_interfaces__srv__PlaceComponent_Response *)allocator.zero_allocate(size, sizeof(pm_skills_interfaces__srv__PlaceComponent_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pm_skills_interfaces__srv__PlaceComponent_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pm_skills_interfaces__srv__PlaceComponent_Response__fini(&data[i - 1]);
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
pm_skills_interfaces__srv__PlaceComponent_Response__Sequence__fini(pm_skills_interfaces__srv__PlaceComponent_Response__Sequence * array)
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
      pm_skills_interfaces__srv__PlaceComponent_Response__fini(&array->data[i]);
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

pm_skills_interfaces__srv__PlaceComponent_Response__Sequence *
pm_skills_interfaces__srv__PlaceComponent_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pm_skills_interfaces__srv__PlaceComponent_Response__Sequence * array = (pm_skills_interfaces__srv__PlaceComponent_Response__Sequence *)allocator.allocate(sizeof(pm_skills_interfaces__srv__PlaceComponent_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pm_skills_interfaces__srv__PlaceComponent_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pm_skills_interfaces__srv__PlaceComponent_Response__Sequence__destroy(pm_skills_interfaces__srv__PlaceComponent_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pm_skills_interfaces__srv__PlaceComponent_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pm_skills_interfaces__srv__PlaceComponent_Response__Sequence__are_equal(const pm_skills_interfaces__srv__PlaceComponent_Response__Sequence * lhs, const pm_skills_interfaces__srv__PlaceComponent_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pm_skills_interfaces__srv__PlaceComponent_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pm_skills_interfaces__srv__PlaceComponent_Response__Sequence__copy(
  const pm_skills_interfaces__srv__PlaceComponent_Response__Sequence * input,
  pm_skills_interfaces__srv__PlaceComponent_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pm_skills_interfaces__srv__PlaceComponent_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pm_skills_interfaces__srv__PlaceComponent_Response * data =
      (pm_skills_interfaces__srv__PlaceComponent_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pm_skills_interfaces__srv__PlaceComponent_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pm_skills_interfaces__srv__PlaceComponent_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pm_skills_interfaces__srv__PlaceComponent_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
