# 指定CMake的最低版本要求
# Specify the minimum version of CMake required
cmake_minimum_required(VERSION 3.16)

# 设置项目名称和版本，指定使用的语言
# Set the project name, version, and specify the language used
project(qt6_ros_template VERSION 0.1 LANGUAGES CXX)

# 设置C++标准为C++17
# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 启用Qt的自动化工具
# Enable Qt's automation tools
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# 设置Qt UI文件的搜索路径
# Set the search path for Qt UI files
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# 查找Qt包
# Find the Qt package
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# 如果需要添加其他Qt模块，在这里添加
# Add other Qt modules here if needed
# find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Network Multimedia)

# 检查是否在ROS环境中
# Check if we are in a ROS environment
if(DEFINED ENV{ROS_DISTRO})
    # 查找catkin包和其他ROS依赖
    # Find catkin package and other ROS dependencies
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs

        # 在这里添加其他ROS依赖
        # Add other ROS dependencies here
    )

    # 包含目录
    # Include directories
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/ui
        ${catkin_INCLUDE_DIRS}
    )

    # 声明catkin包
    # Declare catkin package
    catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs

        # 在这里添加其他catkin依赖
        # Add other catkin dependencies here
    )
else()
    # 非ROS环境的警告信息
    # Warning message for non-ROS environment
    message(WARNING "ROS environment not detected. Building without ROS support.")

    # 非ROS环境下的包含目录
    # Include directories for non-ROS environment
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/ui
    )
endif()

# 设置项目源文件
# Set project source files
set(PROJECT_SOURCES
    src/main.cpp
    src/mainwindow.cpp
    include/mainwindow.h

    # 在这里添加其他源文件
    # Add other source files here
)

# 设置项目UI文件
# Set project UI files
set(PROJECT_UI
    ui/mainwindow.ui

    # 在这里添加其他UI文件
    # Add other UI files here
)

# 根据Qt版本添加可执行文件
# Add executable based on Qt version
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(${PROJECT_NAME}
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        ${PROJECT_UI}
    )
else()
    if(ANDROID)
        add_library(${PROJECT_NAME} SHARED
            ${PROJECT_SOURCES}
            ${PROJECT_UI}
        )
    else()
        add_executable(${PROJECT_NAME}
            ${PROJECT_SOURCES}
            ${PROJECT_UI}
        )
    endif()
endif()

# 链接Qt库
# Link Qt libraries
target_link_libraries(${PROJECT_NAME} PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

# 如果添加了其他Qt模块，在这里链接
# If you added other Qt modules, link them here
# target_link_libraries(${PROJECT_NAME} PRIVATE Qt${QT_VERSION_MAJOR}::Network Qt${QT_VERSION_MAJOR}::Multimedia)

# ROS特定的设置
# ROS-specific settings
if(DEFINED ENV{ROS_DISTRO})
    # 链接ROS库
    # Link ROS libraries
    target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES})

    # 添加ROS节点可执行文件
    # Add ROS node executable
    add_executable(${PROJECT_NAME}_node src/qt6_ros_template_node.cpp)
    add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
    )

    # 安装目标
    # Install targets
    install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    # 安装launch文件
    # Install launch files
    install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
else()
    # 非ROS环境下的目标属性设置
    # Set target properties for non-ROS builds
    set_target_properties(${PROJECT_NAME} PROPERTIES
        MACOSX_BUNDLE_GUI_IDENTIFIER com.example.${PROJECT_NAME}
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
        MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        MACOSX_BUNDLE TRUE
        WIN32_EXECUTABLE TRUE
    )

    # 非ROS环境下的安装目标
    # Install targets for non-ROS builds
    install(TARGETS ${PROJECT_NAME}
        BUNDLE DESTINATION .
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

# Qt6特定的结束设置
# Qt6-specific finalization
if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(${PROJECT_NAME})
endif()
