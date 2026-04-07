// generated from rosidl_generator_cs/resource/idl.cs.em
// with input from pm_skills_interfaces:srv/MeasureFrameLaser.idl
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
public class MeasureFrameLaser_Request : MessageInternals, Message
{
  private IntPtr _handle;
  private static readonly DllLoadUtils dllLoadUtils;

  public bool IsDisposed { get { return disposed; } }
  private bool disposed;

  // constant declarations

  // members
  public System.String Frame_name { get; set; }

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

  static MeasureFrameLaser_Request()
  {
    Ros2csLogger logger = Ros2csLogger.GetInstance();

    dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
    IntPtr messageLibraryTypesupport = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_c");
    IntPtr messageLibraryGenerator = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_generator_c");
    IntPtr messageLibraryIntro = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_introspection_c");
    MessageTypeSupportPreload();

    IntPtr nativelibrary = dllLoadUtils.LoadLibrary("pm_skills_interfaces_srv_measure_frame_laser__rosidl_typesupport_c");
    IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Request_native_get_type_support");
    MeasureFrameLaser_Request.native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
      native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));

    IntPtr native_create_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Request_native_create_native_message");
    MeasureFrameLaser_Request.native_create_native_message = (NativeCreateNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_create_native_message_ptr, typeof(NativeCreateNativeMessageType));

    IntPtr native_destroy_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Request_native_destroy_native_message");
    MeasureFrameLaser_Request.native_destroy_native_message = (NativeDestroyNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_destroy_native_message_ptr, typeof(NativeDestroyNativeMessageType));

    IntPtr native_read_field_frame_name_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Request_native_read_field_frame_name");
    MeasureFrameLaser_Request.native_read_field_frame_name =
      (NativeReadFieldFrame_nameType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_frame_name_ptr, typeof(NativeReadFieldFrame_nameType));

    IntPtr native_write_field_frame_name_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Request_native_write_field_frame_name");
    MeasureFrameLaser_Request.native_write_field_frame_name =
      (NativeWriteFieldFrame_nameType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_frame_name_ptr, typeof(NativeWriteFieldFrame_nameType));
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

  public MeasureFrameLaser_Request()
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

  ~MeasureFrameLaser_Request()
  {
    Dispose();
  }

};  // class MeasureFrameLaser_Request
}  // namespace srv
}  // namespace pm_skills_interfaces




namespace pm_skills_interfaces
{
namespace srv
{
// message class
public class MeasureFrameLaser_Response : MessageInternals, Message
{
  private IntPtr _handle;
  private static readonly DllLoadUtils dllLoadUtils;

  public bool IsDisposed { get { return disposed; } }
  private bool disposed;

  // constant declarations

  // members
  public bool Success { get; set; }
  public geometry_msgs.msg.Vector3 Result_vector { get; set; }
  public System.String Message { get; set; }
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
  private delegate IntPtr NativeGetNestedHandleResult_vectorType(
    IntPtr messageHandle);
  private static NativeGetNestedHandleResult_vectorType native_get_nested_message_handle_result_vector = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeReadFieldMessageType(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldMessageType(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);


  private static NativeReadFieldMessageType native_read_field_message = null;
  private static NativeWriteFieldMessageType native_write_field_message = null;
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

  static MeasureFrameLaser_Response()
  {
    Ros2csLogger logger = Ros2csLogger.GetInstance();

    dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
    IntPtr messageLibraryTypesupport = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_c");
    IntPtr messageLibraryGenerator = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_generator_c");
    IntPtr messageLibraryIntro = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_introspection_c");
    MessageTypeSupportPreload();

    IntPtr nativelibrary = dllLoadUtils.LoadLibrary("pm_skills_interfaces_srv_measure_frame_laser__rosidl_typesupport_c");
    IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Response_native_get_type_support");
    MeasureFrameLaser_Response.native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
      native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));

    IntPtr native_create_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Response_native_create_native_message");
    MeasureFrameLaser_Response.native_create_native_message = (NativeCreateNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_create_native_message_ptr, typeof(NativeCreateNativeMessageType));

    IntPtr native_destroy_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Response_native_destroy_native_message");
    MeasureFrameLaser_Response.native_destroy_native_message = (NativeDestroyNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_destroy_native_message_ptr, typeof(NativeDestroyNativeMessageType));

    IntPtr native_read_field_success_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Response_native_read_field_success");
    MeasureFrameLaser_Response.native_read_field_success =
      (NativeReadFieldSuccessType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_success_ptr, typeof(NativeReadFieldSuccessType));

    IntPtr native_write_field_success_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Response_native_write_field_success");
    MeasureFrameLaser_Response.native_write_field_success =
      (NativeWriteFieldSuccessType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_success_ptr, typeof(NativeWriteFieldSuccessType));
    IntPtr native_get_nested_message_handle_result_vector_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Response_native_get_nested_message_handle_result_vector");
    MeasureFrameLaser_Response.native_get_nested_message_handle_result_vector =
      (NativeGetNestedHandleResult_vectorType)Marshal.GetDelegateForFunctionPointer(
      native_get_nested_message_handle_result_vector_ptr, typeof(NativeGetNestedHandleResult_vectorType));
    IntPtr native_read_field_message_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Response_native_read_field_message");
    MeasureFrameLaser_Response.native_read_field_message =
      (NativeReadFieldMessageType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_message_ptr, typeof(NativeReadFieldMessageType));

    IntPtr native_write_field_message_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Response_native_write_field_message");
    MeasureFrameLaser_Response.native_write_field_message =
      (NativeWriteFieldMessageType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_message_ptr, typeof(NativeWriteFieldMessageType));
    IntPtr native_read_field_component_name_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Response_native_read_field_component_name");
    MeasureFrameLaser_Response.native_read_field_component_name =
      (NativeReadFieldComponent_nameType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_component_name_ptr, typeof(NativeReadFieldComponent_nameType));

    IntPtr native_write_field_component_name_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Response_native_write_field_component_name");
    MeasureFrameLaser_Response.native_write_field_component_name =
      (NativeWriteFieldComponent_nameType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_component_name_ptr, typeof(NativeWriteFieldComponent_nameType));
    IntPtr native_read_field_component_uuid_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Response_native_read_field_component_uuid");
    MeasureFrameLaser_Response.native_read_field_component_uuid =
      (NativeReadFieldComponent_uuidType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_component_uuid_ptr, typeof(NativeReadFieldComponent_uuidType));

    IntPtr native_write_field_component_uuid_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__MeasureFrameLaser_Response_native_write_field_component_uuid");
    MeasureFrameLaser_Response.native_write_field_component_uuid =
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

  public MeasureFrameLaser_Response()
  {
    Result_vector = new geometry_msgs.msg.Vector3();
    Message = "";
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
    Result_vector.ReadNativeMessage(native_get_nested_message_handle_result_vector(handle));
    {
      IntPtr pStr = native_read_field_message(handle);
      Message = Marshal.PtrToStringAnsi(pStr);
    }
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
    Result_vector.WriteNativeMessage(native_get_nested_message_handle_result_vector(handle));
    native_write_field_message(handle, Message);
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

  ~MeasureFrameLaser_Response()
  {
    Dispose();
  }

};  // class MeasureFrameLaser_Response
}  // namespace srv
}  // namespace pm_skills_interfaces

