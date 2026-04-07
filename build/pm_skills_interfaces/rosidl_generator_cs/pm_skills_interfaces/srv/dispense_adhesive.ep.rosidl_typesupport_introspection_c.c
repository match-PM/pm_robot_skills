// generated from rosidl_generator_cs/resource/idl_typesupport.c.em
// with input from pm_skills_interfaces:srv/DispenseAdhesive.idl
// generated code does not contain a copyright notice







#include <stdbool.h>
#include <stdint.h>
#include <rosidl_runtime_c/visibility_control.h>

#include <pm_skills_interfaces/srv/dispense_adhesive.h>

ROSIDL_GENERATOR_C_EXPORT
void * pm_skills_interfaces__srv__DispenseAdhesive_Request_native_get_type_support()
{
    return (void *)ROSIDL_GET_SRV_TYPE_SUPPORT(pm_skills_interfaces, srv, DispenseAdhesive);
}

ROSIDL_GENERATOR_C_EXPORT
void *pm_skills_interfaces__srv__DispenseAdhesive_Request_native_create_native_message()
{
   pm_skills_interfaces__srv__DispenseAdhesive_Request *ros_message = pm_skills_interfaces__srv__DispenseAdhesive_Request__create();
   return ros_message;
}

ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__DispenseAdhesive_Request_native_destroy_native_message(void *raw_ros_message) {
  pm_skills_interfaces__srv__DispenseAdhesive_Request *ros_message = (pm_skills_interfaces__srv__DispenseAdhesive_Request *)raw_ros_message;
  pm_skills_interfaces__srv__DispenseAdhesive_Request__destroy(ros_message);
}







ROSIDL_GENERATOR_C_EXPORT
void * pm_skills_interfaces__srv__DispenseAdhesive_Response_native_get_type_support()
{
    return (void *)ROSIDL_GET_SRV_TYPE_SUPPORT(pm_skills_interfaces, srv, DispenseAdhesive);
}

ROSIDL_GENERATOR_C_EXPORT
void *pm_skills_interfaces__srv__DispenseAdhesive_Response_native_create_native_message()
{
   pm_skills_interfaces__srv__DispenseAdhesive_Response *ros_message = pm_skills_interfaces__srv__DispenseAdhesive_Response__create();
   return ros_message;
}

ROSIDL_GENERATOR_C_EXPORT
void pm_skills_interfaces__srv__DispenseAdhesive_Response_native_destroy_native_message(void *raw_ros_message) {
  pm_skills_interfaces__srv__DispenseAdhesive_Response *ros_message = (pm_skills_interfaces__srv__DispenseAdhesive_Response *)raw_ros_message;
  pm_skills_interfaces__srv__DispenseAdhesive_Response__destroy(ros_message);
}

