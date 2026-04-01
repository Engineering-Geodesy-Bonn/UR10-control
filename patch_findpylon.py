import os

# 1. Provide a robust FindPylon.cmake
findpylon_path = "/catkin_ws/src/pylon-ros-camera/pylon_camera/cmake/FindPylon.cmake"
new_findpylon = """
set(_PYLON_CONFIG "/opt/pylon/bin/pylon-config")
if (EXISTS "${_PYLON_CONFIG}")
    set(Pylon_FOUND TRUE)
    set(Pylon_INCLUDE_DIRS "/opt/pylon/include")
    execute_process(COMMAND ${_PYLON_CONFIG} --libs OUTPUT_VARIABLE Pylon_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REPLACE " " ";" Pylon_LIBRARIES "${Pylon_LDFLAGS}")
    # Force add all libraries explicitly to avoid stripping / regex issues
    list(APPEND Pylon_LIBRARIES "-lpylonutility")
    list(APPEND Pylon_LIBRARIES "-lpylonbase")
    list(APPEND Pylon_LIBRARIES "-lGenApi_gcc_v3_5_Basler_pylon_v1")
    list(APPEND Pylon_LIBRARIES "-lGCBase_gcc_v3_5_Basler_pylon_v1")
    list(APPEND Pylon_LIBRARIES "-lLog_gcc_v3_5_Basler_pylon_v1")
    list(APPEND Pylon_LIBRARIES "-lMathParser_gcc_v3_5_Basler_pylon_v1")
    list(APPEND Pylon_LIBRARIES "-lNodeMapData_gcc_v3_5_Basler_pylon_v1")
    list(APPEND Pylon_LIBRARIES "-lXmlParser_gcc_v3_5_Basler_pylon_v1")
    list(APPEND Pylon_LIBRARIES "-llog4cpp_gcc_v3_5_Basler_pylon_v1")
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()
"""
with open(findpylon_path, "w") as f:
    f.write(new_findpylon)

# 2. Force pylon_camera to use our FindPylon.cmake instead of the official pylon config if any
cmakelists_path = "/catkin_ws/src/pylon-ros-camera/pylon_camera/CMakeLists.txt"
with open(cmakelists_path, "r") as f:
    cmakelists = f.read()

cmakelists = cmakelists.replace("find_package(Pylon QUIET)", "# find_package(Pylon QUIET)")
cmakelists = cmakelists.replace("if (NOT ${Pylon_FOUND})", "if (TRUE)")

with open(cmakelists_path, "w") as f:
    f.write(cmakelists)

