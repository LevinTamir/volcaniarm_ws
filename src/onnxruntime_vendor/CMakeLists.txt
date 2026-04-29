cmake_minimum_required(VERSION 3.16)
project(onnxruntime_vendor VERSION 1.25.0)

find_package(ament_cmake REQUIRED)

set(ORT_VERSION "1.25.0")
set(ORT_URL
  "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-x64-${ORT_VERSION}.tgz"
)

include(FetchContent)
FetchContent_Declare(
  onnxruntime_prebuilt
  URL ${ORT_URL}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(onnxruntime_prebuilt)

# Install shared library + symlinks and headers into the package prefix.
install(
  DIRECTORY ${onnxruntime_prebuilt_SOURCE_DIR}/lib/
  DESTINATION lib
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "libonnxruntime*"
)
install(
  DIRECTORY ${onnxruntime_prebuilt_SOURCE_DIR}/include/
  DESTINATION include/onnxruntime
)

ament_package(CONFIG_EXTRAS "cmake/onnxruntime_vendor-extras.cmake")
