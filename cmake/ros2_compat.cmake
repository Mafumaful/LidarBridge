function(target_link_ros2_package_compat target package_name)
  target_include_directories(${target} PRIVATE ${${package_name}_INCLUDE_DIRS})

  foreach(candidate_target
      "${package_name}::${package_name}"
      "${package_name}::${package_name}__rosidl_typesupport_cpp")
    if(TARGET "${candidate_target}")
      target_link_libraries(${target} PRIVATE "${candidate_target}")
      return()
    endif()
  endforeach()

  if(DEFINED ${package_name}_LIBRARIES AND NOT "${${package_name}_LIBRARIES}" STREQUAL "")
    target_link_libraries(${target} PRIVATE ${${package_name}_LIBRARIES})
    return()
  endif()

  message(FATAL_ERROR
    "No compatible CMake target or library export found for package '${package_name}'")
endfunction()
