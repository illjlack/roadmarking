cmake_minimum_required(VERSION 3.18)
set(CMAKE_CXX_STANDARD 20) 



# 定义可选插件变量，默认 OFF（即默认不编译）
option(PLUGIN_STANDARD_QROADMAKING "Enable qRoadMarkings Plugin" OFF)
if (PLUGIN_STANDARD_QROADMAKING)
    project(QROADMARKING_PLUGIN)

    # 添加 CloudCompare 插件支持
    AddPlugin(NAME ${PROJECT_NAME})

    # 添加插件的代码目录
    add_subdirectory(include)
    add_subdirectory(src)

    # 查找 PCL 库
    find_package(PCL REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

    # 递归处理目录中的源文件和头文件，并分组
    function(group_source_files base_path group_prefix)
        file(GLOB_RECURSE h_files "${base_path}/*.h")
        file(GLOB_RECURSE cpp_files "${base_path}/*.cpp")

        # 获取目录名称
        get_filename_component(dir_name ${base_path} NAME)

        # 为当前目录的文件分组
        source_group("${group_prefix}/${dir_name}" FILES ${h_files} ${cpp_files})

        # 递归处理子目录
        file(GLOB sub_dirs LIST_DIRECTORIES true "${base_path}/*")
        foreach(sub_dir ${sub_dirs})
            if(IS_DIRECTORY ${sub_dir} AND NOT sub_dir STREQUAL ${base_path})
                group_source_files(${sub_dir} "${group_prefix}/${dir_name}")
            endif()
        endforeach()
    endfunction()

    # 调用函数处理当前目录
    group_source_files(${CMAKE_CURRENT_LIST_DIR} "Source Files")
endif()
