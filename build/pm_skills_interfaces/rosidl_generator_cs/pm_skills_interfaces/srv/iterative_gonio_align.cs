// generated from rosidl_generator_cs/resource/idl.cs.em
// with input from pm_skills_interfaces:srv/IterativeGonioAlign.idl
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
public class IterativeGonioAlign_Request : MessageInternals, Message
{
  private IntPtr _handle;
  private static readonly DllLoadUtils dllLoadUtils;

  public bool IsDisposed { get { return disposed; } }
  private bool disposed;

  // constant declarations

  // members
  public bool Confocal_laser { get; set; }
  public System.String Target_alignment_frame { get; set; }
  public System.String Gonio_endeffector_frame { get; set; }
  public int Num_iterations { get; set; }
  public System.String[] Frames_to_measure { get; set; }

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
  private delegate bool NativeReadFieldConfocal_laserType(
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldConfocal_laserType(
    IntPtr messageHandle, bool value);


  private static NativeReadFieldConfocal_laserType native_read_field_confocal_laser = null;
  private static NativeWriteFieldConfocal_laserType native_write_field_confocal_laser = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeReadFieldTarget_alignment_frameType(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldTarget_alignment_frameType(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);


  private static NativeReadFieldTarget_alignment_frameType native_read_field_target_alignment_frame = null;
  private static NativeWriteFieldTarget_alignment_frameType native_write_field_target_alignment_frame = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate IntPtr NativeReadFieldGonio_endeffector_frameType(IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldGonio_endeffector_frameType(
    IntPtr messageHandle, [MarshalAs (UnmanagedType.LPStr)] string value);


  private static NativeReadFieldGonio_endeffector_frameType native_read_field_gonio_endeffector_frame = null;
  private static NativeWriteFieldGonio_endeffector_frameType native_write_field_gonio_endeffector_frame = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate int NativeReadFieldNum_iterationsType(
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  private delegate void NativeWriteFieldNum_iterationsType(
    IntPtr messageHandle, int value);


  private static NativeReadFieldNum_iterationsType native_read_field_num_iterations = null;
  private static NativeWriteFieldNum_iterationsType native_write_field_num_iterations = null;
  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  internal delegate IntPtr NativeReadFieldFrames_to_measureType(
    int index,
    IntPtr messageHandle);

  [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
  internal delegate bool NativeWriteFieldFrames_to_measureType(
      [MarshalAs (UnmanagedType.LPStr)] string value,
      int index,
      IntPtr messageHandle);


  private static NativeReadFieldFrames_to_measureType native_read_field_frames_to_measure = null;
  private static NativeWriteFieldFrames_to_measureType native_write_field_frames_to_measure = null;
  private delegate int NativeGetArraySizeFrames_to_measureType(
    IntPtr messageHandle);
  private static NativeGetArraySizeFrames_to_measureType native_get_array_size_frames_to_measure = null;

  private delegate bool NativeInitSequenceFrames_to_measureType(
    IntPtr messageHandle, int size);
  private static NativeInitSequenceFrames_to_measureType native_init_sequence_frames_to_measure = null;

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

  static IterativeGonioAlign_Request()
  {
    Ros2csLogger logger = Ros2csLogger.GetInstance();

    dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
    IntPtr messageLibraryTypesupport = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_c");
    IntPtr messageLibraryGenerator = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_generator_c");
    IntPtr messageLibraryIntro = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_introspection_c");
    MessageTypeSupportPreload();

    IntPtr nativelibrary = dllLoadUtils.LoadLibrary("pm_skills_interfaces_srv_iterative_gonio_align__rosidl_typesupport_c");
    IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_get_type_support");
    IterativeGonioAlign_Request.native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
      native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));

    IntPtr native_create_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_create_native_message");
    IterativeGonioAlign_Request.native_create_native_message = (NativeCreateNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_create_native_message_ptr, typeof(NativeCreateNativeMessageType));

    IntPtr native_destroy_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_destroy_native_message");
    IterativeGonioAlign_Request.native_destroy_native_message = (NativeDestroyNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_destroy_native_message_ptr, typeof(NativeDestroyNativeMessageType));

    IntPtr native_read_field_confocal_laser_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_read_field_confocal_laser");
    IterativeGonioAlign_Request.native_read_field_confocal_laser =
      (NativeReadFieldConfocal_laserType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_confocal_laser_ptr, typeof(NativeReadFieldConfocal_laserType));

    IntPtr native_write_field_confocal_laser_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_write_field_confocal_laser");
    IterativeGonioAlign_Request.native_write_field_confocal_laser =
      (NativeWriteFieldConfocal_laserType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_confocal_laser_ptr, typeof(NativeWriteFieldConfocal_laserType));
    IntPtr native_read_field_target_alignment_frame_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_read_field_target_alignment_frame");
    IterativeGonioAlign_Request.native_read_field_target_alignment_frame =
      (NativeReadFieldTarget_alignment_frameType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_target_alignment_frame_ptr, typeof(NativeReadFieldTarget_alignment_frameType));

    IntPtr native_write_field_target_alignment_frame_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_write_field_target_alignment_frame");
    IterativeGonioAlign_Request.native_write_field_target_alignment_frame =
      (NativeWriteFieldTarget_alignment_frameType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_target_alignment_frame_ptr, typeof(NativeWriteFieldTarget_alignment_frameType));
    IntPtr native_read_field_gonio_endeffector_frame_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_read_field_gonio_endeffector_frame");
    IterativeGonioAlign_Request.native_read_field_gonio_endeffector_frame =
      (NativeReadFieldGonio_endeffector_frameType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_gonio_endeffector_frame_ptr, typeof(NativeReadFieldGonio_endeffector_frameType));

    IntPtr native_write_field_gonio_endeffector_frame_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_write_field_gonio_endeffector_frame");
    IterativeGonioAlign_Request.native_write_field_gonio_endeffector_frame =
      (NativeWriteFieldGonio_endeffector_frameType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_gonio_endeffector_frame_ptr, typeof(NativeWriteFieldGonio_endeffector_frameType));
    IntPtr native_read_field_num_iterations_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_read_field_num_iterations");
    IterativeGonioAlign_Request.native_read_field_num_iterations =
      (NativeReadFieldNum_iterationsType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_num_iterations_ptr, typeof(NativeReadFieldNum_iterationsType));

    IntPtr native_write_field_num_iterations_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_write_field_num_iterations");
    IterativeGonioAlign_Request.native_write_field_num_iterations =
      (NativeWriteFieldNum_iterationsType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_num_iterations_ptr, typeof(NativeWriteFieldNum_iterationsType));
    IntPtr native_read_field_frames_to_measure_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_read_field_frames_to_measure");
    IterativeGonioAlign_Request.native_read_field_frames_to_measure =
      (NativeReadFieldFrames_to_measureType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_frames_to_measure_ptr, typeof(NativeReadFieldFrames_to_measureType));

    IntPtr native_write_field_frames_to_measure_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_write_field_frames_to_measure");
    IterativeGonioAlign_Request.native_write_field_frames_to_measure =
      (NativeWriteFieldFrames_to_measureType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_frames_to_measure_ptr, typeof(NativeWriteFieldFrames_to_measureType));

    IntPtr native_get_array_size_frames_to_measure_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_get_array_size_frames_to_measure");
    IterativeGonioAlign_Request.native_get_array_size_frames_to_measure =
      (NativeGetArraySizeFrames_to_measureType)Marshal.GetDelegateForFunctionPointer(
    native_get_array_size_frames_to_measure_ptr, typeof(NativeGetArraySizeFrames_to_measureType));

    IntPtr native_init_sequence_frames_to_measure_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Request_native_init_sequence_frames_to_measure");
    IterativeGonioAlign_Request.native_init_sequence_frames_to_measure =
      (NativeInitSequenceFrames_to_measureType)Marshal.GetDelegateForFunctionPointer(
    native_init_sequence_frames_to_measure_ptr, typeof(NativeInitSequenceFrames_to_measureType));
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

  public IterativeGonioAlign_Request()
  {
    Target_alignment_frame = "";
    Gonio_endeffector_frame = "";
    Frames_to_measure = new System.String[0];
  }

  public void ReadNativeMessage()
  {
    ReadNativeMessage(Handle);
  }

  public void ReadNativeMessage(IntPtr handle)
  {
    if (handle == IntPtr.Zero)
      throw new System.InvalidOperationException("Invalid handle for reading");
    Confocal_laser = native_read_field_confocal_laser(handle);
    {
      IntPtr pStr = native_read_field_target_alignment_frame(handle);
      Target_alignment_frame = Marshal.PtrToStringAnsi(pStr);
    }
    {
      IntPtr pStr = native_read_field_gonio_endeffector_frame(handle);
      Gonio_endeffector_frame = Marshal.PtrToStringAnsi(pStr);
    }
    Num_iterations = native_read_field_num_iterations(handle);
    {
      int __native_array_size = native_get_array_size_frames_to_measure(handle);
      Frames_to_measure = new System.String[__native_array_size];
      for (int i = 0; i < __native_array_size; ++i)
      {
        Frames_to_measure[i] = Marshal.PtrToStringAnsi(native_read_field_frames_to_measure(i, handle));
      }
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
    native_write_field_confocal_laser(handle, Confocal_laser);
    native_write_field_target_alignment_frame(handle, Target_alignment_frame);
    native_write_field_gonio_endeffector_frame(handle, Gonio_endeffector_frame);
    native_write_field_num_iterations(handle, Num_iterations);
    {
      bool success = native_init_sequence_frames_to_measure(handle, Frames_to_measure.Length);
      if (!success)
        throw new System.InvalidOperationException("Error initializing sequence for frames_to_measure");
      for (int i = 0; i < Frames_to_measure.Length; ++i)
      {
        native_write_field_frames_to_measure(Frames_to_measure[i], i, handle);
      }
    }
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

  ~IterativeGonioAlign_Request()
  {
    Dispose();
  }

};  // class IterativeGonioAlign_Request
}  // namespace srv
}  // namespace pm_skills_interfaces




namespace pm_skills_interfaces
{
namespace srv
{
// message class
public class IterativeGonioAlign_Response : MessageInternals, Message
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

  static IterativeGonioAlign_Response()
  {
    Ros2csLogger logger = Ros2csLogger.GetInstance();

    dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
    IntPtr messageLibraryTypesupport = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_c");
    IntPtr messageLibraryGenerator = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_generator_c");
    IntPtr messageLibraryIntro = dllLoadUtils.LoadLibraryNoSuffix("pm_skills_interfaces__rosidl_typesupport_introspection_c");
    MessageTypeSupportPreload();

    IntPtr nativelibrary = dllLoadUtils.LoadLibrary("pm_skills_interfaces_srv_iterative_gonio_align__rosidl_typesupport_c");
    IntPtr native_get_typesupport_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Response_native_get_type_support");
    IterativeGonioAlign_Response.native_get_typesupport = (NativeGetTypeSupportType)Marshal.GetDelegateForFunctionPointer(
      native_get_typesupport_ptr, typeof(NativeGetTypeSupportType));

    IntPtr native_create_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Response_native_create_native_message");
    IterativeGonioAlign_Response.native_create_native_message = (NativeCreateNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_create_native_message_ptr, typeof(NativeCreateNativeMessageType));

    IntPtr native_destroy_native_message_ptr = dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Response_native_destroy_native_message");
    IterativeGonioAlign_Response.native_destroy_native_message = (NativeDestroyNativeMessageType)Marshal.GetDelegateForFunctionPointer(
      native_destroy_native_message_ptr, typeof(NativeDestroyNativeMessageType));

    IntPtr native_read_field_success_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Response_native_read_field_success");
    IterativeGonioAlign_Response.native_read_field_success =
      (NativeReadFieldSuccessType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_success_ptr, typeof(NativeReadFieldSuccessType));

    IntPtr native_write_field_success_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Response_native_write_field_success");
    IterativeGonioAlign_Response.native_write_field_success =
      (NativeWriteFieldSuccessType)Marshal.GetDelegateForFunctionPointer(
      native_write_field_success_ptr, typeof(NativeWriteFieldSuccessType));
    IntPtr native_read_field_message_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Response_native_read_field_message");
    IterativeGonioAlign_Response.native_read_field_message =
      (NativeReadFieldMessageType)Marshal.GetDelegateForFunctionPointer(
      native_read_field_message_ptr, typeof(NativeReadFieldMessageType));

    IntPtr native_write_field_message_ptr =
      dllLoadUtils.GetProcAddress(nativelibrary, "pm_skills_interfaces__srv__IterativeGonioAlign_Response_native_write_field_message");
    IterativeGonioAlign_Response.native_write_field_message =
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

  public IterativeGonioAlign_Response()
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

  ~IterativeGonioAlign_Response()
  {
    Dispose();
  }

};  // class IterativeGonioAlign_Response
}  // namespace srv
}  // namespace pm_skills_interfaces

