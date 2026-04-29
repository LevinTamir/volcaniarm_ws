# Loaded by downstream packages via find_package(onnxruntime_vendor).
# Creates an IMPORTED target `onnxruntime::onnxruntime` pointing at the
# prebuilt library installed by this package.

get_filename_component(_ort_prefix "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

if(NOT TARGET onnxruntime::onnxruntime)
  add_library(onnxruntime::onnxruntime SHARED IMPORTED)
  set_target_properties(onnxruntime::onnxruntime PROPERTIES
    IMPORTED_LOCATION "${_ort_prefix}/lib/libonnxruntime.so"
    INTERFACE_INCLUDE_DIRECTORIES "${_ort_prefix}/include/onnxruntime"
  )
endif()

unset(_ort_prefix)
