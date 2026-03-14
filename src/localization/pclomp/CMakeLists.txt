cmake_minimum_required(VERSION 3.10)
project(PCLOMP VERSION "1.0.0")

include(GNUInstallDirs)

# ==================== 编译优化选项 ====================
# ⚠️ 回退优化级别以避免栈溢出和 ABI 不兼容
# 原因: -O3 -march=native -ffast-math 在处理大点云(10万+)时导致:
#   1. 栈溢出 (深度内联 + 循环展开)
#   2. 与使用 -O2 编译的调用方模块 ABI 不兼容
#   3. 内存对齐问题 (Eigen + AVX512 指令集)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# 保守优化: 仅 -O2,无激进选项
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# 移除所有激进编译选项 (原: -O3 -march=native -ffast-math -funroll-loops)
# add_compile_options(...) <-- 已禁用

message(STATUS "� PCLOMP 编译优化(保守模式):")
message(STATUS "   - 优化级别: ${CMAKE_BUILD_TYPE}")
message(STATUS "   - 编译选项: ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "   - ⚠️ 已禁用激进优化 (避免栈溢出/ABI 冲突)")

find_package(PCL 1.7 REQUIRED COMPONENTS registration)
find_package(OpenMP REQUIRED)

set(HEADERS
	src/pclomp/gicp_omp.h
	src/pclomp/ndt_omp.h
	src/pclomp/voxel_grid_covariance_omp.h
)

add_library(registration SHARED
	src/voxel_grid_covariance_omp.cpp
	src/ndt_omp.cpp
	src/gicp_omp.cpp
)

set_target_properties(registration PROPERTIES
	PUBLIC_HEADER "${HEADERS}"
)

target_link_libraries(registration
	PUBLIC ${PCL_REGISTRATION_LIBRARIES}
	PRIVATE OpenMP::OpenMP_CXX
)

target_include_directories(registration PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
	${PCL_INCLUDE_DIRS}
)

# Install binaries
install(TARGETS registration EXPORT pclomp-targets
	ARCHIVE       DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME       DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY       DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/pclomp
)

# Export the library interface
install(EXPORT pclomp-targets
	NAMESPACE pclomp::
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pclomp
)

# Create and install the version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file("pclomp-config-version.cmake"
	VERSION ${PCLOMP_VERSION}
	COMPATIBILITY SameMajorVersion
)

install(
	FILES
		pclomp-config.cmake
		${PROJECT_BINARY_DIR}/pclomp-config-version.cmake
	DESTINATION
		${CMAKE_INSTALL_LIBDIR}/cmake/pclomp
)
