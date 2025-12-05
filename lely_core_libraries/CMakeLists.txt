cmake_minimum_required(VERSION 3.8)
project(lely_core_libraries)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
set(python_version "python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")

include(ExternalProject)
ExternalProject_Add(upstr_lely_core_libraries    # Name for custom target
  #--Download step--------------
  SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/upstream
  INSTALL_DIR "${CMAKE_INSTALL_PREFIX}"  # Installation prefix
  BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/build
  GIT_REPOSITORY https://gitlab.com/lely_industries/lely-core.git
  GIT_TAG fb735b79cab5f0cdda45bc5087414d405ef8f3ab
  TIMEOUT 60
  #UPDATE step apply patch to fix dcf-tools install
  UPDATE_COMMAND
  COMMAND git reset --hard
  COMMAND git apply --whitespace=fix --reject ${CMAKE_CURRENT_SOURCE_DIR}/patches/0001-Fix-dcf-tools.patch
  #CONFIGURE step execute autoreconf and configure
  CONFIGURE_COMMAND autoreconf -i <SOURCE_DIR>
  COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> --disable-cython --disable-doc --disable-tests --disable-static --disable-diag
  #BUILD STEP execute make
  BUILD_COMMAND $(MAKE) -C ${CMAKE_CURRENT_BINARY_DIR}/build
  #INSTALL STEP do nothing as we install in main
  INSTALL_COMMAND ""
)

#INSTALL lely_core_libraries - execute make install
install(CODE "execute_process(COMMAND ${CMAKE_MAKE_PROGRAM} install VERBOSE=1 WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/build)")

# Ensure dcf-tools entrypoints use an explicit python3 shebang on platforms
# like RHEL9 where /usr/bin/python is disallowed.
install(CODE "
  foreach(script dcf2dev dcfchk dcfgen)
    set(path \"${CMAKE_INSTALL_PREFIX}/bin/${script}\")
    if(EXISTS \"${path}\")
      file(READ \"${path}\" _content)
      if(_content MATCHES \"^#! */usr/bin/python\\n\")
        string(REGEX REPLACE \"^#! */usr/bin/python\\n\" \"#!/usr/bin/python3\\n\" _content \"${_content}\")
        file(WRITE \"${path}\" \"${_content}\")
        file(CHMOD \"${path}\" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
      endif()
    endif()
  endforeach()
")

set(lely_core_cmake_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include("cmake/lely_core_libraries-extras.cmake" NO_POLICY_SCOPE)


install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(cogen SCRIPTS_DESTINATION lib/cogen)

# install entry-point script(s) in bin as well
install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/cogen/scripts/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
    USE_SOURCE_PERMISSIONS)

ament_export_include_directories(include)
ament_export_libraries(lely-can lely-co lely-coapp lely-ev lely-io2 lely-libc lely-tap lely-util)
ament_package(
  CONFIG_EXTRAS
  "cmake/lely_core_libraries-extras.cmake"
)
