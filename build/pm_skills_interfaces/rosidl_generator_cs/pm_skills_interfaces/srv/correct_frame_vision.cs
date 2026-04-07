// generated from rosidl_generator_cs/resource/idl.cs.em
// with input from pm_skills_interfaces:srv/CorrectFrameVision.idl
// generated code does not contain a copyright notice

//TODO (adamdbrw): include depending on what is needed
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using ROS2;
using ROS2.Internal;





namespace pm_skills_interfaces
{
namespace srv
{
// message class
public class CorrectFrameVision_Request : MessageInternals, Message
{
  private IntPtr _handle;
  private static readonly DllLoadUtils dllLoadUtils;

  public bool IsDisposed { get { return disposed; } }
  private bool disposed;

  // constant declarations

  // members
  public System.String Frame_name { get; set; }
  public bool Remeasure_after_correction { get; set; }

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeGetTypeSupportType();
  private static NativeGetTypeSupportType native_get_typesupport = null;

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeCreateNativeMessageType();
  private static NativeCreateNativeMessageType native_create_native_message = null;

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeDestroyNativeMessageType(IntPtr messageHandle);
  private static NativeDestroyNativeMessageType native_destroy_native_message = null;

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeReadFieldFrame_nameType(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldFrame_nameType(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);


  private static NativeReadFieldFrame_nameType native_read_field_frame_name = null;
  private static NativeWriteFieldFrame_nameType native_write_field_frame_name = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate bool NativeReadFieldRemeasure_after_correctionType(
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldRemeasure_after_correctionType(
    IntPtr messageHandle, bool value);


  private static NativeReadFieldRemeasure_after_correctionType native_read_field_remeasure_after_correction = null;
  private static NativeWriteFieldRemeasure_after_correctionType native_write_field_remeasure_after_correction = null;

  // This is done to preload before ros2 rmw_implementation attempts to find custom message library (and fails without absolute path)
  static private void MessageTypeSupportPreload()
  {
    if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
    { //only affects Linux since on Windows PATH can be set effectively, dynamically
        const string rmw_fastrtps = "rmw_fastrtps_cpp";
        var rmw_implementation = Environment.GetEnvironmentVariable("RMW_IMPLEMENTATION");
        if (rmw_implementation == null)
        {
          var ros_distro = Environment.GetEnvironmentVariable("ROS_DISTRO");
          if (ros_distro == "galactic")
          { // no preloads for CycloneDDS, default for galactic
            return;
          }
          rmw_implementation = rmw_fastrtps; // default for all other distros
        }

        // TODO - generalize to Connext and other implementations
        if (rmw_implementation == rmw_fastrtps)
        { // TODO - get rcl level constants, e.g. rosidl_typesupport_fastrtps_c__identifier
          // Load typesupport for fastrtps (_c depends on _cpp)
          var loadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
          IntPtr messageLibraryTypesupportFastRTPS_CPP = loadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_fastrtps_cpp");
          IntPtr messageLibraryTypesupportFastRTPS_C = loadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_fastrtps_c");
      }
    }
  }

  static CorrectFrameVision_Request()
  {
    Ros2csLogger logger = Ros2csLogger.GetInstance();

    dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
    IntPtr messageLibraryTypesupport = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_c");
    IntPtr messageLibraryGenerator = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_generator_c");
    IntPtr messageLibraryIntro = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_introspection_c");
    MessageTypeSupportPreload();

    IntPtr nativelibrary = dllLoadUtils.LoadLibrary("pm_skills_interfaces_srv_correct_frame_vision__rosidl_typesupport_c");
    IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Request_native_get_type_support");
    CorrectFrameVision_Request.native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
      native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));

    IntPtr native_create_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Request_native_create_native_message");
    CorrectFrameVision_Request.native_create_native_message = (NativeCreateNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_create_native_message_ptr, typeof(NativeCreateNativeMessageType));

    IntPtr native_destroy_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Request_native_destroy_native_message");
    CorrectFrameVision_Request.native_destroy_native_message = (NativeDestroyNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_destroy_native_message_ptr, typeof(NativeDestroyNativeMessageType));

    IntPtr native_read_field_frame_name_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Request_native_read_field_frame_name");
    CorrectFrameVision_Request.native_read_field_frame_name =
      (NativeReadFieldFrame_nameType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_frame_name_ptr, typeof(NativeReadFieldFrame_nameType));

    IntPtr native_write_field_frame_name_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Request_native_write_field_frame_name");
    CorrectFrameVision_Request.native_write_field_frame_name =
      (NativeWriteFieldFrame_nameType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_frame_name_ptr, typeof(NativeWriteFieldFrame_nameType));
    IntPtr native_read_field_remeasure_after_correction_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Request_native_read_field_remeasure_after_correction");
    CorrectFrameVision_Request.native_read_field_remeasure_after_correction =
      (NativeReadFieldRemeasure_after_correctionType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_remeasure_after_correction_ptr, typeof(NativeReadFieldRemeasure_after_correctionType));

    IntPtr native_write_field_remeasure_after_correction_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Request_native_write_field_remeasure_after_correction");
    CorrectFrameVision_Request.native_write_field_remeasure_after_correction =
      (NativeWriteFieldRemeasure_after_correctionType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_remeasure_after_correction_ptr, typeof(NativeWriteFieldRemeasure_after_correctionType));
  }

  public IntPtr TypeSupportHandle
  {
    get
    {
      return native_get_typesupport();
    }
  }

  // Handle. Create on first use. Can be set for nested classes. TODO -- access...
  public IntPtr Handle
  {
    get
    {
      if (_handle == IntPtr.Zero)
        _handle = native_create_native_message();
      return _handle;
    }
  }

  public CorrectFrameVision_Request()
  {
    Frame_name = "";
  }

  public void ReadNativeMessage()
  {
    ReadNativeMessage(Handle);
  }

  public void ReadNativeMessage(IntPtr handle)
  {
    if (handle == IntPtr.Zero)
      throw new System.InvalidOperationException("Invalid handle for reading");
    {
      IntPtr pStr = native_read_field_frame_name(handle);
      Frame_name = Marshal.PtrToStringAnsi(pStr);
    }
    Remeasure_after_correction = native_read_field_remeasure_after_correction(handle);
  }

  public void WriteNativeMessage()
  {
    if (_handle == IntPtr.Zero)
    { // message object reused for subsequent publishing.
      // This could be problematic if sequences sizes changed, but me handle that by checking for it in the c implementation
      _handle = native_create_native_message();
    }

    WriteNativeMessage(Handle);
  }

  // Write from CS to native handle
  public void WriteNativeMessage(IntPtr handle)
  {
    if (handle == IntPtr.Zero)
      throw new System.InvalidOperationException("Invalid handle for writing");
    native_write_field_frame_name(handle, Frame_name);
    native_write_field_remeasure_after_correction(handle, Remeasure_after_correction);
  }

  public void Dispose()
  {
    if (!disposed)
    {
      if (_handle != IntPtr.Zero)
      {
        native_destroy_native_message(_handle);
        _handle = IntPtr.Zero;
        disposed = true;
      }
    }
  }

  ~CorrectFrameVision_Request()
  {
    Dispose();
  }

};  // class CorrectFrameVision_Request
}  // namespace srv
}  // namespace pm_skills_interfaces




namespace pm_skills_interfaces
{
namespace srv
{
// message class
public class CorrectFrameVision_Response : MessageInternals, Message
{
  private IntPtr _handle;
  private static readonly DllLoadUtils dllLoadUtils;

  public bool IsDisposed { get { return disposed; } }
  private bool disposed;

  // constant declarations

  // members
  public bool Success { get; set; }
  public geometry_msgs.msg.Vector3 Correction_values { get; set; }
  public System.String Message { get; set; }
  public pm_vision_interfaces.msg.VisionResponse Vision_response { get; set; }
  public System.String Component_name { get; set; }
  public System.String Component_uuid { get; set; }

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeGetTypeSupportType();
  private static NativeGetTypeSupportType native_get_typesupport = null;

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeCreateNativeMessageType();
  private static NativeCreateNativeMessageType native_create_native_message = null;

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeDestroyNativeMessageType(IntPtr messageHandle);
  private static NativeDestroyNativeMessageType native_destroy_native_message = null;

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate bool NativeReadFieldSuccessType(
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldSuccessType(
    IntPtr messageHandle, bool value);


  private static NativeReadFieldSuccessType native_read_field_success = null;
  private static NativeWriteFieldSuccessType native_write_field_success = null;

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeGetNestedHandleCorrection_valuesType(
    IntPtr messageHandle);
  private static NativeGetNestedHandleCorrection_valuesType native_get_nested_message_handle_correction_values = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeReadFieldMessageType(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldMessageType(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);


  private static NativeReadFieldMessageType native_read_field_message = null;
  private static NativeWriteFieldMessageType native_write_field_message = null;

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeGetNestedHandleVision_responseType(
    IntPtr messageHandle);
  private static NativeGetNestedHandleVision_responseType native_get_nested_message_handle_vision_response = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeReadFieldComponent_nameType(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldComponent_nameType(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);


  private static NativeReadFieldComponent_nameType native_read_field_component_name = null;
  private static NativeWriteFieldComponent_nameType native_write_field_component_name = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeReadFieldComponent_uuidType(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldComponent_uuidType(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);


  private static NativeReadFieldComponent_uuidType native_read_field_component_uuid = null;
  private static NativeWriteFieldComponent_uuidType native_write_field_component_uuid = null;

  // This is done to preload before ros2 rmw_implementation attempts to find custom message library (and fails without absolute path)
  static private void MessageTypeSupportPreload()
  {
    if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
    { //only affects Linux since on Windows PATH can be set effectively, dynamically
        const string rmw_fastrtps = "rmw_fastrtps_cpp";
        var rmw_implementation = Environment.GetEnvironmentVariable("RMW_IMPLEMENTATION");
        if (rmw_implementation == null)
        {
          var ros_distro = Environment.GetEnvironmentVariable("ROS_DISTRO");
          if (ros_distro == "galactic")
          { // no preloads for CycloneDDS, default for galactic
            return;
          }
          rmw_implementation = rmw_fastrtps; // default for all other distros
        }

        // TODO - generalize to Connext and other implementations
        if (rmw_implementation == rmw_fastrtps)
        { // TODO - get rcl level constants, e.g. rosidl_typesupport_fastrtps_c__identifier
          // Load typesupport for fastrtps (_c depends on _cpp)
          var loadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
          IntPtr messageLibraryTypesupportFastRTPS_CPP = loadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_fastrtps_cpp");
          IntPtr messageLibraryTypesupportFastRTPS_C = loadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_fastrtps_c");
      }
    }
  }

  static CorrectFrameVision_Response()
  {
    Ros2csLogger logger = Ros2csLogger.GetInstance();

    dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
    IntPtr messageLibraryTypesupport = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_c");
    IntPtr messageLibraryGenerator = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_generator_c");
    IntPtr messageLibraryIntro = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_introspection_c");
    MessageTypeSupportPreload();

    IntPtr nativelibrary = dllLoadUtils.LoadLibrary("pm_skills_interfaces_srv_correct_frame_vision__rosidl_typesupport_c");
    IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Response_native_get_type_support");
    CorrectFrameVision_Response.native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
      native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));

    IntPtr native_create_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Response_native_create_native_message");
    CorrectFrameVision_Response.native_create_native_message = (NativeCreateNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_create_native_message_ptr, typeof(NativeCreateNativeMessageType));

    IntPtr native_destroy_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Response_native_destroy_native_message");
    CorrectFrameVision_Response.native_destroy_native_message = (NativeDestroyNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_destroy_native_message_ptr, typeof(NativeDestroyNativeMessageType));

    IntPtr native_read_field_success_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Response_native_read_field_success");
    CorrectFrameVision_Response.native_read_field_success =
      (NativeReadFieldSuccessType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_success_ptr, typeof(NativeReadFieldSuccessType));

    IntPtr native_write_field_success_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Response_native_write_field_success");
    CorrectFrameVision_Response.native_write_field_success =
      (NativeWriteFieldSuccessType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_success_ptr, typeof(NativeWriteFieldSuccessType));
    IntPtr native_get_nested_message_handle_correction_values_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Response_native_get_nested_message_handle_correction_values");
    CorrectFrameVision_Response.native_get_nested_message_handle_correction_values =
      (NativeGetNestedHandleCorrection_valuesType)Marshal.GetDelegateForFunctionPointer(
      native_get_nested_message_handle_correction_values_ptr, typeof(NativeGetNestedHandleCorrection_valuesType));
    IntPtr native_read_field_message_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Response_native_read_field_message");
    CorrectFrameVision_Response.native_read_field_message =
      (NativeReadFieldMessageType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_message_ptr, typeof(NativeReadFieldMessageType));

    IntPtr native_write_field_message_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Response_native_write_field_message");
    CorrectFrameVision_Response.native_write_field_message =
      (NativeWriteFieldMessageType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_message_ptr, typeof(NativeWriteFieldMessageType));
    IntPtr native_get_nested_message_handle_vision_response_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Response_native_get_nested_message_handle_vision_response");
    CorrectFrameVision_Response.native_get_nested_message_handle_vision_response =
      (NativeGetNestedHandleVision_responseType)Marshal.GetDelegateForFunctionPointer(
      native_get_nested_message_handle_vision_response_ptr, typeof(NativeGetNestedHandleVision_responseType));
    IntPtr native_read_field_component_name_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Response_native_read_field_component_name");
    CorrectFrameVision_Response.native_read_field_component_name =
      (NativeReadFieldComponent_nameType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_component_name_ptr, typeof(NativeReadFieldComponent_nameType));

    IntPtr native_write_field_component_name_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Response_native_write_field_component_name");
    CorrectFrameVision_Response.native_write_field_component_name =
      (NativeWriteFieldComponent_nameType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_component_name_ptr, typeof(NativeWriteFieldComponent_nameType));
    IntPtr native_read_field_component_uuid_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Response_native_read_field_component_uuid");
    CorrectFrameVision_Response.native_read_field_component_uuid =
      (NativeReadFieldComponent_uuidType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_component_uuid_ptr, typeof(NativeReadFieldComponent_uuidType));

    IntPtr native_write_field_component_uuid_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__CorrectFrameVision_Response_native_write_field_component_uuid");
    CorrectFrameVision_Response.native_write_field_component_uuid =
      (NativeWriteFieldComponent_uuidType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_component_uuid_ptr, typeof(NativeWriteFieldComponent_uuidType));
  }

  public IntPtr TypeSupportHandle
  {
    get
    {
      return native_get_typesupport();
    }
  }

  // Handle. Create on first use. Can be set for nested classes. TODO -- access...
  public IntPtr Handle
  {
    get
    {
      if (_handle == IntPtr.Zero)
        _handle = native_create_native_message();
      return _handle;
    }
  }

  public CorrectFrameVision_Response()
  {
    Correction_values = new geometry_msgs.msg.Vector3();
    Message = "";
    Vision_response = new pm_vision_interfaces.msg.VisionResponse();
    Component_name = "";
    Component_uuid = "";
  }

  public void ReadNativeMessage()
  {
    ReadNativeMessage(Handle);
  }

  public void ReadNativeMessage(IntPtr handle)
  {
    if (handle == IntPtr.Zero)
      throw new System.InvalidOperationException("Invalid handle for reading");
    Success = native_read_field_success(handle);
    Correction_values.ReadNativeMessage(native_get_nested_message_handle_correction_values(handle));
    {
      IntPtr pStr = native_read_field_message(handle);
      Message = Marshal.PtrToStringAnsi(pStr);
    }
    Vision_response.ReadNativeMessage(native_get_nested_message_handle_vision_response(handle));
    {
      IntPtr pStr = native_read_field_component_name(handle);
      Component_name = Marshal.PtrToStringAnsi(pStr);
    }
    {
      IntPtr pStr = native_read_field_component_uuid(handle);
      Component_uuid = Marshal.PtrToStringAnsi(pStr);
    }
  }

  public void WriteNativeMessage()
  {
    if (_handle == IntPtr.Zero)
    { // message object reused for subsequent publishing.
      // This could be problematic if sequences sizes changed, but me handle that by checking for it in the c implementation
      _handle = native_create_native_message();
    }

    WriteNativeMessage(Handle);
  }

  // Write from CS to native handle
  public void WriteNativeMessage(IntPtr handle)
  {
    if (handle == IntPtr.Zero)
      throw new System.InvalidOperationException("Invalid handle for writing");
    native_write_field_success(handle, Success);
    Correction_values.WriteNativeMessage(native_get_nested_message_handle_correction_values(handle));
    native_write_field_message(handle, Message);
    Vision_response.WriteNativeMessage(native_get_nested_message_handle_vision_response(handle));
    native_write_field_component_name(handle, Component_name);
    native_write_field_component_uuid(handle, Component_uuid);
  }

  public void Dispose()
  {
    if (!disposed)
    {
      if (_handle != IntPtr.Zero)
      {
        native_destroy_native_message(_handle);
        _handle = IntPtr.Zero;
        disposed = true;
      }
    }
  }

  ~CorrectFrameVision_Response()
  {
    Dispose();
  }

};  // class CorrectFrameVision_Response
}  // namespace srv
}  // namespace pm_skills_interfaces

