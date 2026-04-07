// generated from rosidl_generator_cs/resource/idl.cs.em
// with input from pm_skills_interfaces:srv/ExecuteVision.idl
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
public class ExecuteVision_Request : MessageInternals, Message
{
  private IntPtr _handle;
  private static readonly DllLoadUtils dllLoadUtils;

  public bool IsDisposed { get { return disposed; } }
  private bool disposed;

  // constant declarations

  // members
  public System.String Process_filename { get; set; }
  public System.String Camera_config_filename { get; set; }
  public System.String Process_uid { get; set; }
  public float Image_display_time { get; set; }
  public bool Run_cross_validation { get; set; }

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
  private delegate IntPtr NativeReadFieldProcess_filenameType(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldProcess_filenameType(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);


  private static NativeReadFieldProcess_filenameType native_read_field_process_filename = null;
  private static NativeWriteFieldProcess_filenameType native_write_field_process_filename = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeReadFieldCamera_config_filenameType(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldCamera_config_filenameType(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);


  private static NativeReadFieldCamera_config_filenameType native_read_field_camera_config_filename = null;
  private static NativeWriteFieldCamera_config_filenameType native_write_field_camera_config_filename = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeReadFieldProcess_uidType(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldProcess_uidType(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);


  private static NativeReadFieldProcess_uidType native_read_field_process_uid = null;
  private static NativeWriteFieldProcess_uidType native_write_field_process_uid = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate float NativeReadFieldImage_display_timeType(
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldImage_display_timeType(
    IntPtr messageHandle, float value);


  private static NativeReadFieldImage_display_timeType native_read_field_image_display_time = null;
  private static NativeWriteFieldImage_display_timeType native_write_field_image_display_time = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate bool NativeReadFieldRun_cross_validationType(
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldRun_cross_validationType(
    IntPtr messageHandle, bool value);


  private static NativeReadFieldRun_cross_validationType native_read_field_run_cross_validation = null;
  private static NativeWriteFieldRun_cross_validationType native_write_field_run_cross_validation = null;

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

  static ExecuteVision_Request()
  {
    Ros2csLogger logger = Ros2csLogger.GetInstance();

    dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
    IntPtr messageLibraryTypesupport = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_c");
    IntPtr messageLibraryGenerator = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_generator_c");
    IntPtr messageLibraryIntro = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_introspection_c");
    MessageTypeSupportPreload();

    IntPtr nativelibrary = dllLoadUtils.LoadLibrary("pm_skills_interfaces_srv_execute_vision__rosidl_typesupport_c");
    IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Request_native_get_type_support");
    ExecuteVision_Request.native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
      native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));

    IntPtr native_create_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Request_native_create_native_message");
    ExecuteVision_Request.native_create_native_message = (NativeCreateNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_create_native_message_ptr, typeof(NativeCreateNativeMessageType));

    IntPtr native_destroy_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Request_native_destroy_native_message");
    ExecuteVision_Request.native_destroy_native_message = (NativeDestroyNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_destroy_native_message_ptr, typeof(NativeDestroyNativeMessageType));

    IntPtr native_read_field_process_filename_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Request_native_read_field_process_filename");
    ExecuteVision_Request.native_read_field_process_filename =
      (NativeReadFieldProcess_filenameType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_process_filename_ptr, typeof(NativeReadFieldProcess_filenameType));

    IntPtr native_write_field_process_filename_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Request_native_write_field_process_filename");
    ExecuteVision_Request.native_write_field_process_filename =
      (NativeWriteFieldProcess_filenameType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_process_filename_ptr, typeof(NativeWriteFieldProcess_filenameType));
    IntPtr native_read_field_camera_config_filename_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Request_native_read_field_camera_config_filename");
    ExecuteVision_Request.native_read_field_camera_config_filename =
      (NativeReadFieldCamera_config_filenameType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_camera_config_filename_ptr, typeof(NativeReadFieldCamera_config_filenameType));

    IntPtr native_write_field_camera_config_filename_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Request_native_write_field_camera_config_filename");
    ExecuteVision_Request.native_write_field_camera_config_filename =
      (NativeWriteFieldCamera_config_filenameType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_camera_config_filename_ptr, typeof(NativeWriteFieldCamera_config_filenameType));
    IntPtr native_read_field_process_uid_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Request_native_read_field_process_uid");
    ExecuteVision_Request.native_read_field_process_uid =
      (NativeReadFieldProcess_uidType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_process_uid_ptr, typeof(NativeReadFieldProcess_uidType));

    IntPtr native_write_field_process_uid_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Request_native_write_field_process_uid");
    ExecuteVision_Request.native_write_field_process_uid =
      (NativeWriteFieldProcess_uidType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_process_uid_ptr, typeof(NativeWriteFieldProcess_uidType));
    IntPtr native_read_field_image_display_time_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Request_native_read_field_image_display_time");
    ExecuteVision_Request.native_read_field_image_display_time =
      (NativeReadFieldImage_display_timeType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_image_display_time_ptr, typeof(NativeReadFieldImage_display_timeType));

    IntPtr native_write_field_image_display_time_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Request_native_write_field_image_display_time");
    ExecuteVision_Request.native_write_field_image_display_time =
      (NativeWriteFieldImage_display_timeType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_image_display_time_ptr, typeof(NativeWriteFieldImage_display_timeType));
    IntPtr native_read_field_run_cross_validation_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Request_native_read_field_run_cross_validation");
    ExecuteVision_Request.native_read_field_run_cross_validation =
      (NativeReadFieldRun_cross_validationType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_run_cross_validation_ptr, typeof(NativeReadFieldRun_cross_validationType));

    IntPtr native_write_field_run_cross_validation_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Request_native_write_field_run_cross_validation");
    ExecuteVision_Request.native_write_field_run_cross_validation =
      (NativeWriteFieldRun_cross_validationType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_run_cross_validation_ptr, typeof(NativeWriteFieldRun_cross_validationType));
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

  public ExecuteVision_Request()
  {
    Process_filename = "";
    Camera_config_filename = "";
    Process_uid = "";
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
      IntPtr pStr = native_read_field_process_filename(handle);
      Process_filename = Marshal.PtrToStringAnsi(pStr);
    }
    {
      IntPtr pStr = native_read_field_camera_config_filename(handle);
      Camera_config_filename = Marshal.PtrToStringAnsi(pStr);
    }
    {
      IntPtr pStr = native_read_field_process_uid(handle);
      Process_uid = Marshal.PtrToStringAnsi(pStr);
    }
    Image_display_time = native_read_field_image_display_time(handle);
    Run_cross_validation = native_read_field_run_cross_validation(handle);
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
    native_write_field_process_filename(handle, Process_filename);
    native_write_field_camera_config_filename(handle, Camera_config_filename);
    native_write_field_process_uid(handle, Process_uid);
    native_write_field_image_display_time(handle, Image_display_time);
    native_write_field_run_cross_validation(handle, Run_cross_validation);
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

  ~ExecuteVision_Request()
  {
    Dispose();
  }

};  // class ExecuteVision_Request
}  // namespace srv
}  // namespace pm_skills_interfaces




namespace pm_skills_interfaces
{
namespace srv
{
// message class
public class ExecuteVision_Response : MessageInternals, Message
{
  private IntPtr _handle;
  private static readonly DllLoadUtils dllLoadUtils;

  public bool IsDisposed { get { return disposed; } }
  private bool disposed;

  // constant declarations

  // members
  public bool Success { get; set; }
  public System.String Results_dict { get; set; }
  public System.String Results_path { get; set; }
  public float[] Points { get; set; }
  public System.String Process_uid { get; set; }

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
  private delegate IntPtr NativeReadFieldResults_dictType(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldResults_dictType(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);


  private static NativeReadFieldResults_dictType native_read_field_results_dict = null;
  private static NativeWriteFieldResults_dictType native_write_field_results_dict = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeReadFieldResults_pathType(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldResults_pathType(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);


  private static NativeReadFieldResults_pathType native_read_field_results_path = null;
  private static NativeWriteFieldResults_pathType native_write_field_results_path = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  internal delegate IntPtr NativeReadFieldPointsType(
    out int array_size,
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  internal delegate bool NativeWriteFieldPointsType(
      [MarshalAs(UnmanagedType.LPArray, ArraySubType = UnmanagedType.R4, SizeParamIndex = 1)]
      float[] values,
      int array_size,
      IntPtr messageHandle);

  private static NativeReadFieldPointsType native_read_field_points = null;
  private static NativeWriteFieldPointsType native_write_field_points = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeReadFieldProcess_uidType(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldProcess_uidType(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);


  private static NativeReadFieldProcess_uidType native_read_field_process_uid = null;
  private static NativeWriteFieldProcess_uidType native_write_field_process_uid = null;

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

  static ExecuteVision_Response()
  {
    Ros2csLogger logger = Ros2csLogger.GetInstance();

    dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
    IntPtr messageLibraryTypesupport = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_c");
    IntPtr messageLibraryGenerator = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_generator_c");
    IntPtr messageLibraryIntro = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_introspection_c");
    MessageTypeSupportPreload();

    IntPtr nativelibrary = dllLoadUtils.LoadLibrary("pm_skills_interfaces_srv_execute_vision__rosidl_typesupport_c");
    IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Response_native_get_type_support");
    ExecuteVision_Response.native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
      native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));

    IntPtr native_create_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Response_native_create_native_message");
    ExecuteVision_Response.native_create_native_message = (NativeCreateNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_create_native_message_ptr, typeof(NativeCreateNativeMessageType));

    IntPtr native_destroy_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Response_native_destroy_native_message");
    ExecuteVision_Response.native_destroy_native_message = (NativeDestroyNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_destroy_native_message_ptr, typeof(NativeDestroyNativeMessageType));

    IntPtr native_read_field_success_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Response_native_read_field_success");
    ExecuteVision_Response.native_read_field_success =
      (NativeReadFieldSuccessType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_success_ptr, typeof(NativeReadFieldSuccessType));

    IntPtr native_write_field_success_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Response_native_write_field_success");
    ExecuteVision_Response.native_write_field_success =
      (NativeWriteFieldSuccessType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_success_ptr, typeof(NativeWriteFieldSuccessType));
    IntPtr native_read_field_results_dict_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Response_native_read_field_results_dict");
    ExecuteVision_Response.native_read_field_results_dict =
      (NativeReadFieldResults_dictType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_results_dict_ptr, typeof(NativeReadFieldResults_dictType));

    IntPtr native_write_field_results_dict_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Response_native_write_field_results_dict");
    ExecuteVision_Response.native_write_field_results_dict =
      (NativeWriteFieldResults_dictType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_results_dict_ptr, typeof(NativeWriteFieldResults_dictType));
    IntPtr native_read_field_results_path_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Response_native_read_field_results_path");
    ExecuteVision_Response.native_read_field_results_path =
      (NativeReadFieldResults_pathType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_results_path_ptr, typeof(NativeReadFieldResults_pathType));

    IntPtr native_write_field_results_path_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Response_native_write_field_results_path");
    ExecuteVision_Response.native_write_field_results_path =
      (NativeWriteFieldResults_pathType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_results_path_ptr, typeof(NativeWriteFieldResults_pathType));
    IntPtr native_read_field_points_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Response_native_read_field_points");
    ExecuteVision_Response.native_read_field_points =
      (NativeReadFieldPointsType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_points_ptr, typeof(NativeReadFieldPointsType));

    IntPtr native_write_field_points_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Response_native_write_field_points");
    ExecuteVision_Response.native_write_field_points =
      (NativeWriteFieldPointsType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_points_ptr, typeof(NativeWriteFieldPointsType));
    IntPtr native_read_field_process_uid_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Response_native_read_field_process_uid");
    ExecuteVision_Response.native_read_field_process_uid =
      (NativeReadFieldProcess_uidType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_process_uid_ptr, typeof(NativeReadFieldProcess_uidType));

    IntPtr native_write_field_process_uid_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__ExecuteVision_Response_native_write_field_process_uid");
    ExecuteVision_Response.native_write_field_process_uid =
      (NativeWriteFieldProcess_uidType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_process_uid_ptr, typeof(NativeWriteFieldProcess_uidType));
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

  public ExecuteVision_Response()
  {
    Results_dict = "";
    Results_path = "";
    Points = new float[0];
    Process_uid = "";
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
      IntPtr pStr = native_read_field_results_dict(handle);
      Results_dict = Marshal.PtrToStringAnsi(pStr);
    }
    {
      IntPtr pStr = native_read_field_results_path(handle);
      Results_path = Marshal.PtrToStringAnsi(pStr);
    }
    { //TODO - (adam) this is a bit clunky. Is there a better way to marshal unsigned and bool types?
      int arraySize = 0;
      IntPtr pArr = native_read_field_points(out arraySize, handle);
      Points = new float[arraySize];
      float[] __Points = new float[arraySize];
      int start = 0;

      Marshal.Copy(pArr, __Points, start, arraySize);
      for (int i = 0; i < arraySize; ++i)
      {
        Points[i] = (float)(__Points[i]);
      }
    }
    {
      IntPtr pStr = native_read_field_process_uid(handle);
      Process_uid = Marshal.PtrToStringAnsi(pStr);
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
    native_write_field_results_dict(handle, Results_dict);
    native_write_field_results_path(handle, Results_path);
    {
            bool success = native_write_field_points(Points, Points.Length, handle);
      
      if (!success)
        throw new System.InvalidOperationException("Error writing field for points");
    }
    native_write_field_process_uid(handle, Process_uid);
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

  ~ExecuteVision_Response()
  {
    Dispose();
  }

};  // class ExecuteVision_Response
}  // namespace srv
}  // namespace pm_skills_interfaces

