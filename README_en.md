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
- [Contribution Guidelines](#contribution-guidelines)
- [Version History](#version-history)
- [License](#license)
- [Contact Information](#contact-information)

## Project Overview

**Qt6 ROS Template** is a human-machine interaction interface template based on Qt6 for ROS Noetic. This project aims to provide developers with a quick-start framework to facilitate the development of ROS human-machine interaction interfaces. It combines the modern UI design capabilities of Qt6 with the powerful robot development ecosystem of ROS, providing an ideal starting point for graphical interface development for robotic applications.

This project is inspired by the Qt5 version of the [ros_qt_demo](https://github.com/XKHoshizora/ros_qt_demo) project. However, compared to the Qt5 version, this project has significant improvements:

1. **Dual Compilation Support**: This template supports both `catkin_make` in the ROS environment and direct compilation in Qt Creator without complicated configuration steps.
2. **Simplified Development Process**: Compared to the Qt5 version, which required various configurations to develop in Qt Creator, this project offers an "out-of-the-box" experience, greatly simplifying the development process.
3. **Enhanced Flexibility**: Developers can choose to use either the ROS toolchain or Qt Creator for development, offering greater flexibility.

## Features

- Integration of Qt6 and ROS Noetic, providing modern UI design and powerful robot development capabilities
- Dual compilation support: compatible with ROS `catkin_make` and Qt Creator's build system
  - Can be compiled using `catkin_make` in a ROS-only environment
  - Can be compiled and opened in Qt Creator in a Qt6-only environment
  - Can be compiled using either `catkin_make` or Qt Creator in a ROS and Qt6 environment
- No complex configuration required, can be directly opened in Qt Creator for development
- Pre-configured CMakeLists.txt, simplifying the integration process of Qt and ROS
- Includes basic ROS nodes and Qt main window implementation as a starting point for development
- Provides ROS launch files for easy startup and management of nodes
- Supports compilation and running in both ROS and non-ROS environments, enhancing project flexibility
- Compared to the Qt5 version, the development process is greatly simplified, improving development efficiency

## System Requirements

- Operating System: Ubuntu 20.04 LTS
- ROS Version: Noetic
- Qt Version: 6.x
- CMake Version: 3.16 or higher

## Installation Guide

### Installing ROS Noetic

There are two ways to install ROS Noetic:

1. Follow the [Official Guide](https://wiki.ros.org/noetic/Installation/Ubuntu)
2. Use the Auto ROS Installer (recommended):
   ```bash
   git clone https://github.com/XKHoshizora/auto-ros-installer.git
   cd auto-ros-installer
   ```
   For detailed installation instructions, please refer to the [Project Homepage](https://github.com/XKHoshizora/auto-ros-installer).

### Installing Qt6

It is recommended to use the official Qt installer to install Qt6:

1. Download the Qt Installer:
   Visit the [Qt Official Download Page](https://www.qt.io/download-qt-installer) and download the online installer for Linux.

2. Grant execution permission:

   ```bash
   chmod +x qt-online-installer-linux-<x64/arm64>-<version>.run
   ```

3. Run the installer:

   ```bash
   ./qt-online-installer-linux-<x64/arm64>-<version>.run
   ```

4. Configure the Qt Creator shortcut:
   Use the following command to create and open the `qtcreator` file

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
   sudo chmod a+x /usr/bin/
   ```

6. Start Qt Creator:

   ```bash
   qtcreator
   ```

7. Install additional dependencies

   If you encounter issues when running Qt Creator, install the following dependencies:

   ```bash
   sudo apt install libxcb-xinerama0 libxcb-xinerama0-dev libxcb-cursor0
   ```

## Project Setup

### Create Workspace

```bash
mkdir -p ~/qt6_ws/src
cd ~/qt6_ws/src
```

### Clone Project

```bash
git clone https://github.com/XKHoshizora/qt6_ros_template.git
```

### Build Project

#### In ROS-only Environment

Execute the following commands in your workspace to compile the package:

```bash
cd ~/qt6_ws
catkin_make
```

#### In Qt6-only Environment

1. Open Qt Creator
2. Select "File" > "Open File or Project"
3. Navigate to the `~/qt6_ws/src/qt6_ros_template` directory and select the `CMakeLists.txt` file to open
4. Configure and open the project. If the project files are displayed normally on the left side, it means the compilation is successful. If the compilation fails, only the `CMakeLists.txt` file will be displayed.

#### In ROS and Qt6 Coexisting Environment

1. Execute the following commands in your workspace to compile the package:

   ```bash
   cd ~/qt6_ws
   catkin_make
   ```

2. To open the project in Qt Creator for development, first compile it as described above. Then follow these steps:
   1. Open Qt Creator
   2. Select "File" > "Open File or Project"
   3. Navigate to `~/qt6_ws/src` and select the `CMakeLists.txt` file (**Note: select the `CMakeLists.txt` file in the workspace, not the one inside the package**)
   4. In the configuration page, click the `Import Build From...` option's `Details` button. Click the `Browse...` button to find the `qt_ws/build` directory and click the `Open` button in the top right corner. Click the `Import` button to complete the import.
   5. Check the `Imported Kit` and `Build` options, and uncheck all other options.
   6. Click the `Configure Project` button in the bottom right corner to complete the configuration and start building the project.
   7. If the project files are displayed normally on the left side, it means the compilation is successful. If the compilation fails, only the `CMakeLists.txt` file will be displayed.

## Running the Project

### Run the Project

#### In ROS-only Environment

First, ensure that the project has been compiled according to the above steps. Then execute the following commands in your workspace:

```bash
source ~/qt6_ws/devel/setup.bash
roslaunch qt6_ros_template demo.launch

# If you want to specify different ROS_HOSTNAME or ROS_IP, use this:
roslaunch qt6_ros_template demo.launch ros_hostname:=your_hostname ros_ip:=your_ip
```

#### In Qt6-only Environment

After compiling and opening the project in Qt Creator, click the green `Run` button on the left, or use the `Ctrl + R` shortcut to run it.

#### In ROS and Qt6 Coexisting Environment

First, ensure that the project has been compiled according to the above steps. Then execute the following commands in your workspace:

```bash
source ~/qt6_ws/devel/setup.bash
roslaunch qt6_ros_template demo.launch

# If you want to specify different ROS_HOSTNAME or ROS_IP, use this:
roslaunch qt6_ros_template demo.launch ros_hostname:=your_hostname ros_ip:=your_ip
```

Or after compiling and opening the project in Qt Creator, click the green `Run` button on the left, or use the `Ctrl + R` shortcut to run it.

### Functional Testing

It is recommended to use the `rosrun` command for testing. In this case, you need to run the `roscore` command to run the `ros master` in a new terminal in advance, and then enter the following command in another new terminal to start the ROS node:

```bash
rosrun qt6_ros_template qt6_ros_template_node
```

After the node runs successfully, follow these steps to test the various functions of the application:

1. **Connect to ROS**

   - Click the "Connect" button in the application.
   - Verify that the "ROS Status" label turns green and displays "Connected".

2. **Change Topic**

   - Enter a new topic name in the "Change Topic" input box (e.g., "/test_topic").
   - Click the "Change Topic" button.

3. **Publish Message**

   - Enter a message in the "Publish" input box.
   - Click the "Publish" button.
   - Use the following command in a new terminal to verify the published message:
     ```
     rostopic echo /test_topic
     ```

4. **Subscribe to Message**

   - In a new terminal, publish a message to the subscribed topic:
     ```
     rostopic pub /test_topic std_msgs/String "data: 'Greetings from the command line'" -1
     ```
   - Verify that the message appears in the application's log view.

5. **Test Disconnect**

   - Click the "Disconnect" button.
   - Verify that the "ROS Status" label turns red and displays "Disconnected".
   - Attempt to publish a message and verify that it fails.

6. **Test ROS Master Failure**

   - Connect to ROS using the application.
   - In the terminal running roscore, stop it using Ctrl+C.
   - Observe the application's log view and console output for disconnection messages.

7. **Use Environment Variables**

   - Check the "Use environment variables" checkbox.
   - Verify that the master URL and ROS IP fields are populated from environment variables.

8. **Save and Load Settings**
   - Change some settings in the application.
   - Close and reopen the application.
   - Verify that the settings are correctly restored.

### Troubleshooting

If you encounter any issues:

1. Ensure that `roscore` is running before starting the application.
2. Check that your ROS environment variables (`ROS_MASTER_URI` and `ROS_IP`) are set correctly.
3. If running ROS on multiple machines, verify network connectivity.
4. Check the console output and application logs for any error messages.

## Project Structure

This project uses a standard CMake build system and follows the directory structure of a Qt project.

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

1. **Problem**: "Qt6 not found" error during compilation.
   **Solution**: Ensure that Qt6 is installed correctly and that CMAKE_PREFIX_PATH includes the Qt6 installation path.

2. **Problem**: ROS-related functions are undefined.
   **Solution**: Ensure you have sourced devel/setup.bash and that all necessary dependencies are included in package.xml.

3. **Problem**: Unable to find ROS header files in Qt Creator.
   **Solution**: Add the ROS include path (usually /opt/ros/noetic/include) in the project settings of Qt Creator.

## Contribution Guidelines

We welcome and appreciate any form of contribution! If you would like to contribute to the project, please follow these steps:

1. Fork this repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Create a new Pull Request

## Version History

- 0.1.0
  - Initial release
  - Basic integration of Qt6 and ROS Noetic
  - Dual compilation support (ROS environment and Qt Creator)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact Information

Project Maintainer: XKHoshizora - hoshizoranihon@gmail.com

Project Link: [https://github.com/XKHoshizora/qt6_ros_template](https://github.com/XKHoshizora/qt6_ros_template)
