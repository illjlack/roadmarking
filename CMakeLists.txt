cmake_minimum_required(VERSION 3.18)
project(QROADMARKING_PLUGIN)

set(CMAKE_CXX_STANDARD 20)
set(PROJECT_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# 插件开关
option(PLUGIN_STANDARD_QROADMAKING "Enable qRoadMarkings Plugin" OFF)

if (PLUGIN_STANDARD_QROADMAKING)

    # 插件支持
    AddPlugin(NAME ${PROJECT_NAME})

        # 源码目录
    set(SRC_DIRS
        core
        ui
        csf
    )

    # 源文件列表
    set(PLUGIN_SOURCES "")
    set(PLUGIN_HEADERS "")

    # 遍历目录收集文件 & 分组
    foreach(dir ${SRC_DIRS})
        file(GLOB_RECURSE headers "${dir}/*.h")
        file(GLOB_RECURSE sources "${dir}/*.cpp")

        list(APPEND PLUGIN_SOURCES ${sources})
        list(APPEND PLUGIN_HEADERS ${headers})

        # 分组（VS显示） - 保留到项目目录
        foreach(header ${headers})
            get_filename_component(header_dir ${header} DIRECTORY)
            string(REPLACE "${PROJECT_BASE_DIR}/" "PROJECT_BASE_DIR\\" header_group "${header_dir}") # 保留项目目录
            string(REPLACE "/" "\\" header_group "${header_group}") # 转换为反斜杠
            source_group("${header_group}" FILES ${header})
        endforeach()

        foreach(source ${sources})
            get_filename_component(source_dir ${source} DIRECTORY)
            string(REPLACE "${PROJECT_BASE_DIR}/" "PROJECT_BASE_DIR\\" source_group "${source_dir}") # 保留项目目录
            string(REPLACE "/" "\\" source_group "${source_group}") # 转换为反斜杠
            source_group("${source_group}" FILES ${source})
        endforeach()
    endforeach()

    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
        ${CMAKE_CURRENT_SOURCE_DIR}/ui/include
        ${CMAKE_CURRENT_SOURCE_DIR}/csf/include
    )

    # 添加到目标
    target_sources(${PROJECT_NAME}
        PRIVATE ${PLUGIN_SOURCES} ${PLUGIN_HEADERS}
    )

    # 第三方库
    find_package(PCL REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

    find_package(OpenGL REQUIRED)
    include_directories(${OPENGL_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES})

    # OpenCV
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

    # ====================================================== CGAL
    find_package(CGAL REQUIRED)
    include_directories(${CGAL_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${CGAL_LIBRARIES})
    # GMP
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(gmp REQUIRED IMPORTED_TARGET gmp)
    target_link_libraries(${PROJECT_NAME} PkgConfig::gmp)
    # MPFR
    find_package(MPFR REQUIRED)
    include_directories(${MPFR_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${MPFR_LIBRARIES})
    # Boost (system + filesystem)
    find_package(Boost REQUIRED COMPONENTS system filesystem)
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

    # 启用私有访问
    target_compile_definitions(${PROJECT_NAME} PRIVATE CC_ACCESS_PRIVATE_MEMBERS)

endif()
