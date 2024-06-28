# Qt6 ROS Template

[中文](README_zh.md) | [English](README_en.md)

## Table of Contents

- [Project Introduction](#project-introduction)
- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation Guide](#installation-guide)
- [Project Setup](#project-setup)
- [Running the Project](#running-the-project)
- [Project Structure](#project-structure)
- [Customization and Extension](#customization-and-extension)
- [Frequently Asked Questions](#frequently-asked-questions)
- [Contribution Guidelines](#contribution-guidelines)
- [Version History](#version-history)
- [License](#license)
- [Contact Information](#contact-information)

## Project Introduction

**Qt6 ROS Template** is a ROS Noetic human-machine interface template built on Qt6. This project aims to provide developers with a quick-start framework for developing ROS human-machine interfaces. It combines Qt6's modern UI design capabilities with ROS's powerful robot development ecosystem, offering an ideal starting point for graphical interface development in robotic applications.

This project draws inspiration from the Qt5 version of the [ros_qt_demo](https://github.com/XKHoshizora/ros_qt_demo) project. However, compared to the Qt5 version, this project has made significant improvements:

1. **Dual Compilation Support**: This template not only supports compilation in the ROS environment using `catkin_make`, but can also be opened and compiled directly in Qt Creator without complex configuration steps.

2. **Simplified Development Process**: Compared to the Qt5 version that required various configurations for development in Qt Creator, this project achieves an "out-of-the-box" experience, greatly simplifying the development process.

3. **Enhanced Flexibility**: Developers can choose to use either the ROS toolchain or Qt Creator for development according to their preferences, providing greater flexibility.

## Features

- Integration of Qt6 and ROS Noetic, providing modern UI design capabilities and powerful robot development capabilities
- Dual compilation support: compatible with ROS's `catkin_make` and Qt Creator's build system
- No complex configuration required, projects can be opened directly in Qt Creator for development
- Pre-configured CMakeLists.txt, simplifying the integration process of Qt and ROS
- Includes basic ROS node and Qt main window implementation as a development starting point
- Provides ROS launch files for easy node startup and management
- Supports compilation and running in both ROS and non-ROS environments, enhancing project flexibility
- Greatly simplifies the development process compared to the Qt5 version, improving development efficiency

## System Requirements

- Operating System: Ubuntu 20.04 LTS
- ROS Version: Noetic
- Qt Version: 6.x
- CMake Version: 3.16 or higher

## Installation Guide

### Installing ROS Noetic

There are two methods to install ROS Noetic:

1. Using the official installation script:

   ```bash
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   ```

2. Using Auto ROS Installer (recommended):
   ```bash
   git clone https://github.com/XKHoshizora/auto-ros-installer.git
   cd auto-ros-installer
   # Follow the instructions in README.md for installation
   ```

### Installing Qt6

It is recommended to install Qt6 using the official Qt installer:

1. Download Qt Installer:
   Visit the [Qt official download page](https://www.qt.io/download-qt-installer) and download the online installer for Linux.

2. Grant execution permissions:

   ```bash
   chmod +x qt-online-installer-linux-<x64/arm64>-<version>.run
   ```

3. Run the installer:

   ```bash
   ./qt-online-installer-linux-<x64/arm64>-<version>.run
   ```

4. Configure Qt Creator shortcut:

   ```bash
   sudo nano /usr/bin/qtcreator
   ```

   Add the following content:

   ```shell
   #!/bin/sh
   export QT_HOME=/home/<user>/Qt/Tools/QtCreator/bin
   $QT_HOME/qtcreator $*
   ```

   Set permissions:

   ```bash
   sudo chmod a+x /usr/bin/qtcreator
   ```

### Installing Other Dependencies

If you encounter issues when running Qt Creator, please install the following dependencies:

```bash
sudo apt install libxcb-xinerama0 libxcb-xinerama0-dev libxcb-cursor0
```

## Project Setup

### Creating a Workspace

```bash
mkdir -p ~/qt6_ros_template_ws/src
cd ~/qt6_ros_template_ws/src
```

### Cloning the Project

```bash
git clone https://github.com/XKHoshizora/qt6_ros_template.git
cd qt6_ros_template
```

### Building the Project

1. Using catkin_make (ROS environment):

   ```bash
   cd ~/qt6_ros_template_ws
   catkin_make
   ```

2. Using Qt Creator:
   - Open Qt Creator
   - Select "File" > "Open File or Project"
   - Navigate to `~/qt6_ros_template_ws/src/qt6_ros_template` and select the `CMakeLists.txt` file
   - Configure the project and click "Build"

## Running the Project

1. Running in the ROS environment:

   ```bash
   source ~/qt6_ros_template_ws/devel/setup.bash
   roslaunch qt6_ros_template demo.launch
   ```

2. Running in Qt Creator:
   - Open the project in Qt Creator
   - Click the "Run" button or press Ctrl+R

## Project Structure

```
qt6_ros_template/
├── CMakeLists.txt
├── package.xml
├── include/
│   ├── qt6_ros_template/
│   └── mainwindow.h
├── msg/
├── resources/
│   ├── images/
│   │   └── icon.png
│   └── resource.qrc
├── scripts/
├── src/
│   ├── main.cpp
│   ├── mainwindow.cpp
│   └── qt6_ros_template_node.cpp
├── srv/
├── ui/
│   └── mainwindow.ui
└── launch/
    └── demo.launch
```

## Customization and Extension

1. Modify `src/mainwindow.cpp` and `ui/mainwindow.ui` to customize the UI.
2. Add ROS functionality in `src/qt6_ros_template_node.cpp`.
3. Update `CMakeLists.txt` to include new source files or dependencies.

## Frequently Asked Questions

1. **Issue**: "Qt6 not found" error during compilation.
   **Solution**: Ensure Qt6 is correctly installed and CMAKE_PREFIX_PATH includes the Qt6 installation path.

2. **Issue**: ROS-related functions are undefined.
   **Solution**: Make sure you have sourced devel/setup.bash and all necessary dependencies are included in package.xml.

3. **Issue**: Unable to find ROS header files in Qt Creator.
   **Solution**: Add the ROS include path (usually /opt/ros/noetic/include) in the project settings of Qt Creator.

## Contribution Guidelines

We welcome and appreciate any form of contribution! If you want to contribute to the project, please follow these steps:

1. Fork this repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Create a new Pull Request

## Version History

- 0.1.0
  - Initial version
  - Basic integration of Qt6 and ROS Noetic
  - Implemented dual compilation support (ROS environment and Qt Creator)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

## Contact Information

Project Maintainer: XKHoshizora - hoshizoranihon@gmail.com

Project Link: [https://github.com/XKHoshizora/qt6_ros_template](https://github.com/XKHoshizora/qt6_ros_template)
