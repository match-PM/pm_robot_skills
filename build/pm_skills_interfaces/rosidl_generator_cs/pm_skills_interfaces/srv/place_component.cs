// generated from rosidl_generator_cs/resource/idl.cs.em
// with input from pm_skills_interfaces:srv/PlaceComponent.idl
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
public class PlaceComponent_Request : MessageInternals, Message
{
  private IntPtr _handle;
  private static readonly DllLoadUtils dllLoadUtils;

  public bool IsDisposed { get { return disposed; } }
  private bool disposed;

  // constant declarations

  // members
  public bool Align_orientation { get; set; }
  public float X_offset_um { get; set; }
  public float Y_offset_um { get; set; }
  public float Z_offset_um { get; set; }
  public float Rx_offset_deg { get; set; }
  public float Ry_offset_deg { get; set; }
  public float Rz_offset_deg { get; set; }

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
  private delegate bool NativeReadFieldAlign_orientationType(
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldAlign_orientationType(
    IntPtr messageHandle, bool value);


  private static NativeReadFieldAlign_orientationType native_read_field_align_orientation = null;
  private static NativeWriteFieldAlign_orientationType native_write_field_align_orientation = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate float NativeReadFieldX_offset_umType(
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldX_offset_umType(
    IntPtr messageHandle, float value);


  private static NativeReadFieldX_offset_umType native_read_field_x_offset_um = null;
  private static NativeWriteFieldX_offset_umType native_write_field_x_offset_um = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate float NativeReadFieldY_offset_umType(
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldY_offset_umType(
    IntPtr messageHandle, float value);


  private static NativeReadFieldY_offset_umType native_read_field_y_offset_um = null;
  private static NativeWriteFieldY_offset_umType native_write_field_y_offset_um = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate float NativeReadFieldZ_offset_umType(
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldZ_offset_umType(
    IntPtr messageHandle, float value);


  private static NativeReadFieldZ_offset_umType native_read_field_z_offset_um = null;
  private static NativeWriteFieldZ_offset_umType native_write_field_z_offset_um = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate float NativeReadFieldRx_offset_degType(
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldRx_offset_degType(
    IntPtr messageHandle, float value);


  private static NativeReadFieldRx_offset_degType native_read_field_rx_offset_deg = null;
  private static NativeWriteFieldRx_offset_degType native_write_field_rx_offset_deg = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate float NativeReadFieldRy_offset_degType(
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldRy_offset_degType(
    IntPtr messageHandle, float value);


  private static NativeReadFieldRy_offset_degType native_read_field_ry_offset_deg = null;
  private static NativeWriteFieldRy_offset_degType native_write_field_ry_offset_deg = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate float NativeReadFieldRz_offset_degType(
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldRz_offset_degType(
    IntPtr messageHandle, float value);


  private static NativeReadFieldRz_offset_degType native_read_field_rz_offset_deg = null;
  private static NativeWriteFieldRz_offset_degType native_write_field_rz_offset_deg = null;

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

  static PlaceComponent_Request()
  {
    Ros2csLogger logger = Ros2csLogger.GetInstance();

    dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
    IntPtr messageLibraryTypesupport = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_c");
    IntPtr messageLibraryGenerator = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_generator_c");
    IntPtr messageLibraryIntro = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_introspection_c");
    MessageTypeSupportPreload();

    IntPtr nativelibrary = dllLoadUtils.LoadLibrary("pm_skills_interfaces_srv_place_component__rosidl_typesupport_c");
    IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_get_type_support");
    PlaceComponent_Request.native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
      native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));

    IntPtr native_create_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_create_native_message");
    PlaceComponent_Request.native_create_native_message = (NativeCreateNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_create_native_message_ptr, typeof(NativeCreateNativeMessageType));

    IntPtr native_destroy_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_destroy_native_message");
    PlaceComponent_Request.native_destroy_native_message = (NativeDestroyNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_destroy_native_message_ptr, typeof(NativeDestroyNativeMessageType));

    IntPtr native_read_field_align_orientation_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_read_field_align_orientation");
    PlaceComponent_Request.native_read_field_align_orientation =
      (NativeReadFieldAlign_orientationType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_align_orientation_ptr, typeof(NativeReadFieldAlign_orientationType));

    IntPtr native_write_field_align_orientation_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_write_field_align_orientation");
    PlaceComponent_Request.native_write_field_align_orientation =
      (NativeWriteFieldAlign_orientationType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_align_orientation_ptr, typeof(NativeWriteFieldAlign_orientationType));
    IntPtr native_read_field_x_offset_um_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_read_field_x_offset_um");
    PlaceComponent_Request.native_read_field_x_offset_um =
      (NativeReadFieldX_offset_umType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_x_offset_um_ptr, typeof(NativeReadFieldX_offset_umType));

    IntPtr native_write_field_x_offset_um_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_write_field_x_offset_um");
    PlaceComponent_Request.native_write_field_x_offset_um =
      (NativeWriteFieldX_offset_umType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_x_offset_um_ptr, typeof(NativeWriteFieldX_offset_umType));
    IntPtr native_read_field_y_offset_um_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_read_field_y_offset_um");
    PlaceComponent_Request.native_read_field_y_offset_um =
      (NativeReadFieldY_offset_umType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_y_offset_um_ptr, typeof(NativeReadFieldY_offset_umType));

    IntPtr native_write_field_y_offset_um_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_write_field_y_offset_um");
    PlaceComponent_Request.native_write_field_y_offset_um =
      (NativeWriteFieldY_offset_umType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_y_offset_um_ptr, typeof(NativeWriteFieldY_offset_umType));
    IntPtr native_read_field_z_offset_um_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_read_field_z_offset_um");
    PlaceComponent_Request.native_read_field_z_offset_um =
      (NativeReadFieldZ_offset_umType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_z_offset_um_ptr, typeof(NativeReadFieldZ_offset_umType));

    IntPtr native_write_field_z_offset_um_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_write_field_z_offset_um");
    PlaceComponent_Request.native_write_field_z_offset_um =
      (NativeWriteFieldZ_offset_umType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_z_offset_um_ptr, typeof(NativeWriteFieldZ_offset_umType));
    IntPtr native_read_field_rx_offset_deg_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_read_field_rx_offset_deg");
    PlaceComponent_Request.native_read_field_rx_offset_deg =
      (NativeReadFieldRx_offset_degType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_rx_offset_deg_ptr, typeof(NativeReadFieldRx_offset_degType));

    IntPtr native_write_field_rx_offset_deg_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_write_field_rx_offset_deg");
    PlaceComponent_Request.native_write_field_rx_offset_deg =
      (NativeWriteFieldRx_offset_degType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_rx_offset_deg_ptr, typeof(NativeWriteFieldRx_offset_degType));
    IntPtr native_read_field_ry_offset_deg_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_read_field_ry_offset_deg");
    PlaceComponent_Request.native_read_field_ry_offset_deg =
      (NativeReadFieldRy_offset_degType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_ry_offset_deg_ptr, typeof(NativeReadFieldRy_offset_degType));

    IntPtr native_write_field_ry_offset_deg_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_write_field_ry_offset_deg");
    PlaceComponent_Request.native_write_field_ry_offset_deg =
      (NativeWriteFieldRy_offset_degType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_ry_offset_deg_ptr, typeof(NativeWriteFieldRy_offset_degType));
    IntPtr native_read_field_rz_offset_deg_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_read_field_rz_offset_deg");
    PlaceComponent_Request.native_read_field_rz_offset_deg =
      (NativeReadFieldRz_offset_degType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_rz_offset_deg_ptr, typeof(NativeReadFieldRz_offset_degType));

    IntPtr native_write_field_rz_offset_deg_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Request_native_write_field_rz_offset_deg");
    PlaceComponent_Request.native_write_field_rz_offset_deg =
      (NativeWriteFieldRz_offset_degType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_rz_offset_deg_ptr, typeof(NativeWriteFieldRz_offset_degType));
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

  public PlaceComponent_Request()
  {
  }

  public void ReadNativeMessage()
  {
    ReadNativeMessage(Handle);
  }

  public void ReadNativeMessage(IntPtr handle)
  {
    if (handle == IntPtr.Zero)
      throw new System.InvalidOperationException("Invalid handle for reading");
    Align_orientation = native_read_field_align_orientation(handle);
    X_offset_um = native_read_field_x_offset_um(handle);
    Y_offset_um = native_read_field_y_offset_um(handle);
    Z_offset_um = native_read_field_z_offset_um(handle);
    Rx_offset_deg = native_read_field_rx_offset_deg(handle);
    Ry_offset_deg = native_read_field_ry_offset_deg(handle);
    Rz_offset_deg = native_read_field_rz_offset_deg(handle);
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
    native_write_field_align_orientation(handle, Align_orientation);
    native_write_field_x_offset_um(handle, X_offset_um);
    native_write_field_y_offset_um(handle, Y_offset_um);
    native_write_field_z_offset_um(handle, Z_offset_um);
    native_write_field_rx_offset_deg(handle, Rx_offset_deg);
    native_write_field_ry_offset_deg(handle, Ry_offset_deg);
    native_write_field_rz_offset_deg(handle, Rz_offset_deg);
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

  ~PlaceComponent_Request()
  {
    Dispose();
  }

};  // class PlaceComponent_Request
}  // namespace srv
}  // namespace pm_skills_interfaces




namespace pm_skills_interfaces
{
namespace srv
{
// message class
public class PlaceComponent_Response : MessageInternals, Message
{
  private IntPtr _handle;
  private static readonly DllLoadUtils dllLoadUtils;

  public bool IsDisposed { get { return disposed; } }
  private bool disposed;

  // constant declarations

  // members
  public bool Success { get; set; }
  public System.String Message { get; set; }

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
  private delegate IntPtr NativeReadFieldMessageType(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldMessageType(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);


  private static NativeReadFieldMessageType native_read_field_message = null;
  private static NativeWriteFieldMessageType native_write_field_message = null;

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

  static PlaceComponent_Response()
  {
    Ros2csLogger logger = Ros2csLogger.GetInstance();

    dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
    IntPtr messageLibraryTypesupport = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_c");
    IntPtr messageLibraryGenerator = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_generator_c");
    IntPtr messageLibraryIntro = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_introspection_c");
    MessageTypeSupportPreload();

    IntPtr nativelibrary = dllLoadUtils.LoadLibrary("pm_skills_interfaces_srv_place_component__rosidl_typesupport_c");
    IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Response_native_get_type_support");
    PlaceComponent_Response.native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
      native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));

    IntPtr native_create_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Response_native_create_native_message");
    PlaceComponent_Response.native_create_native_message = (NativeCreateNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_create_native_message_ptr, typeof(NativeCreateNativeMessageType));

    IntPtr native_destroy_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Response_native_destroy_native_message");
    PlaceComponent_Response.native_destroy_native_message = (NativeDestroyNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_destroy_native_message_ptr, typeof(NativeDestroyNativeMessageType));

    IntPtr native_read_field_success_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Response_native_read_field_success");
    PlaceComponent_Response.native_read_field_success =
      (NativeReadFieldSuccessType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_success_ptr, typeof(NativeReadFieldSuccessType));

    IntPtr native_write_field_success_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Response_native_write_field_success");
    PlaceComponent_Response.native_write_field_success =
      (NativeWriteFieldSuccessType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_success_ptr, typeof(NativeWriteFieldSuccessType));
    IntPtr native_read_field_message_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Response_native_read_field_message");
    PlaceComponent_Response.native_read_field_message =
      (NativeReadFieldMessageType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_message_ptr, typeof(NativeReadFieldMessageType));

    IntPtr native_write_field_message_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__PlaceComponent_Response_native_write_field_message");
    PlaceComponent_Response.native_write_field_message =
      (NativeWriteFieldMessageType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_message_ptr, typeof(NativeWriteFieldMessageType));
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

  public PlaceComponent_Response()
  {
    Message = "";
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
    {
      IntPtr pStr = native_read_field_message(handle);
      Message = Marshal.PtrToStringAnsi(pStr);
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
    native_write_field_message(handle, Message);
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

  ~PlaceComponent_Response()
  {
    Dispose();
  }

};  // class PlaceComponent_Response
}  // namespace srv
}  // namespace pm_skills_interfaces

