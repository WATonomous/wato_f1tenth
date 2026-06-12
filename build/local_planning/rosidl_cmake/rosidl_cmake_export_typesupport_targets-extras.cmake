# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:local_planning__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:local_planning__rosidl_typesupport_fastrtps_c;__rosidl_generator_cpp:local_planning__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:local_planning__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_c:local_planning__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:local_planning__rosidl_typesupport_c;__rosidl_typesupport_introspection_cpp:local_planning__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:local_planning__rosidl_typesupport_cpp;:local_planning__rosidl_generator_py")

# populate local_planning_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "local_planning::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'local_planning' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND local_planning_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
