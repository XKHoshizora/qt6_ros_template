# Qt6 ROS Template

[中文](README_zh.md) | [English](README_en.md)

## Table of Contents

- [Project Overview](#project-overview)
- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation Guide](#installation-guide)
- [Project Setup](#project-setup)
- [Running the Project](#running-the-project)
- [Project Structure](#project-structure)
- [Customization and Extension](#customization-and-extension)
- [FAQ](#faq)
- [Contributing](#contributing)
- [Version History](#version-history)
- [License](#license)
- [Contact](#contact)

## Project Overview

**Qt6 ROS Template** is a human-machine interface template for ROS Noetic built on Qt6. This project aims to provide developers with a quick-start framework for developing ROS human-machine interaction interfaces. It combines Qt6's modern UI design capabilities with ROS's powerful robot development ecosystem, offering an ideal starting point for developing graphical interfaces for robotic applications.

This project was inspired by the Qt5 version of the [ros_qt_demo](https://github.com/XKHoshizora/ros_qt_demo) project. However, compared to the Qt5 version, this project has made significant improvements:

1. **Dual Compilation Support**: This template supports compilation using `catkin_make` in a ROS environment and can also be opened and compiled directly in Qt Creator without complex configuration steps.

2. **Simplified Development Process**: Compared to the Qt5 version that required various configurations for development in Qt Creator, this project achieves an "out-of-the-box" experience, greatly simplifying the development process.

3. **Enhanced Flexibility**: Developers can choose to use either the ROS toolchain or Qt Creator for development according to their preferences, providing greater flexibility.

## Features

- Integration of Qt6 and ROS Noetic, providing modern UI design and powerful robot development capabilities
- Dual compilation support: compatible with ROS's `catkin_make` and Qt Creator's build system
  - Can be compiled using `catkin_make` in an environment with only ROS
  - Can be compiled and opened in Qt Creator in an environment with only Qt6
  - Can be compiled using either `catkin_make` or Qt Creator in an environment with both ROS and Qt6
- Can be opened directly in Qt Creator for development without complex configuration
- Pre-configured CMakeLists.txt, simplifying the integration process of Qt and ROS
- Includes basic ROS node and Qt main window implementation as a starting point for development
- Provides ROS launch file for easy node startup and management
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

1. Follow the [official guide](https://wiki.ros.org/noetic/Installation/Ubuntu)

2. Use Auto ROS Installer (recommended):
   ```bash
   git clone https://github.com/XKHoshizora/auto-ros-installer.git
   cd auto-ros-installer
   ```
   For detailed installation instructions, please refer to the project [homepage](https://github.com/XKHoshizora/auto-ros-installer).

### Installing Qt6

It is recommended to use the official Qt installer to install Qt6:

1. Download Qt Installer:
   Visit the [Qt official download page](https://www.qt.io/download-qt-installer) and download the online installer for Linux.

2. Grant execute permission:

   ```bash
   chmod +x qt-online-installer-linux-<x64/arm64>-<version>.run
   ```

3. Run the installer:

   ```bash
   ./qt-online-installer-linux-<x64/arm64>-<version>.run
   ```

4. Configure Qt Creator shortcut:
   Create and open the `qtcreator` file using the following command:

   ```bash
   sudo nano /usr/bin/qtcreator
   ```

   Add the following content to the `qtcreator` file:

   ```shell
   #!/bin/sh

   export QT_HOME=/home/<user>/Qt/Tools/QtCreator/bin
   $QT_HOME/qtcreator $*
   ```

5. Add executable permission to the `qtcreator` file:

   ```bash
   sudo chmod a+x /usr/bin/qtcreator
   ```

6. Launch Qt Creator:

   ```bash
   qtcreator
   ```

7. Install other dependencies:

   If you encounter issues when running Qt Creator, please install the following dependencies:

   ```bash
   sudo apt install libxcb-xinerama0 libxcb-xinerama0-dev libxcb-cursor0
   ```

## Project Setup

### Creating a Workspace

```bash
mkdir -p ~/qt6_ws/src
cd ~/qt6_ws/src
```

### Cloning the Project

```bash
git clone https://github.com/XKHoshizora/qt6_ros_template.git
```

### Building the Project

#### In a ROS-only Environment

Execute the following commands in your workspace to compile the package:

```bash
cd ~/qt6_ws
catkin_make
```

#### In a Qt6-only Environment

1. Open Qt Creator
2. Select "File" > "Open File or Project"
3. Navigate to the `~/qt6_ws/src/qt6_ros_template` directory, select the `CMakeLists.txt` file and open it
4. Configure and open the project. If the project files are displayed normally on the left side, it indicates successful compilation. If compilation fails, only a `CMakeLists.txt` file will be displayed.

#### In an Environment with both ROS and Qt6

1. Execute the following commands in your workspace to compile the package:

   ```bash
   cd ~/qt6_ws
   catkin_make
   ```

2. To open the project in Qt Creator for development, first compile using the steps in step 1. Then follow these steps:
   1. Open Qt Creator
   2. Select "File" > "Open File or Project"
   3. Navigate to `~/qt6_ws/src` and select the `CMakeLists.txt` file (**Note: Select the `CMakeLists.txt` file in the workspace, not the one in the package**)
   4. On the configuration page, click the `Details` button next to the `Import Build From...` option. Click the `Browse...` button to find the `qt_ws/build` directory and click the `Open` button in the top right corner. Click the `Import` button to complete the import.
   5. Check the `Imported Kit` and `Build` options, and uncheck all other options.
   6. Click the `Configure Project` button in the bottom right corner to complete the configuration and start compiling the project.
   7. If the project files are displayed normally on the left side, it indicates successful compilation. If compilation fails, only a `CMakeLists.txt` file will be displayed.

## Running the Project

### In a ROS-only Environment

First, ensure that you have completed the compilation according to the steps above. Then execute the following commands in the workspace:

```bash
source ~/qt6_ws/devel/setup.bash
roslaunch qt6_ros_template demo.launch
```

### In a Qt6-only Environment

After compiling and opening the project through Qt Creator, click the green `Run` button on the left side or use the `Ctrl + R` shortcut to run.

### In an Environment with both ROS and Qt6

First, ensure that you have completed the compilation according to the steps above. Then execute the following commands in the workspace:

```bash
source ~/qt6_ws/devel/setup.bash
roslaunch qt6_ros_template demo.launch
```

Or, after compiling and opening the project through Qt Creator, click the green `Run` button on the left side or use the `Ctrl + R` shortcut to run.

## Project Structure

This project uses the standard CMake build system and follows the directory structure of Qt projects.

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

## FAQ

1. **Issue**: "Qt6 not found" error during compilation.
   **Solution**: Ensure Qt6 is correctly installed and CMAKE_PREFIX_PATH includes the Qt6 installation path.

2. **Issue**: ROS-related functions are undefined.
   **Solution**: Make sure you have sourced devel/setup.bash and all necessary dependencies are included in package.xml.

3. **Issue**: Unable to find ROS headers in Qt Creator.
   **Solution**: Add ROS include paths (usually in /opt/ros/noetic/include) to the project settings in Qt Creator.

## Contributing

We welcome and appreciate any form of contribution! If you want to contribute to the project, please follow these steps:

1. Fork this repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Create a new Pull Request

## Version History

- 0.1.0
  - Initial release
  - Basic Qt6 and ROS Noetic integration
  - Implemented dual compilation support (ROS environment and Qt Creator)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

## Contact

Project Maintainer: XKHoshizora - hoshizoranihon@gmail.com

Project Link: [https://github.com/XKHoshizora/qt6_ros_template](https://github.com/XKHoshizora/qt6_ros_template)
