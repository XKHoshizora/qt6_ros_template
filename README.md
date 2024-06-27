# Qt6 ROS Template

## 项目简介
**Qt6-ROS-Template** 是一个基于 Qt6 构建的 ROS Noetic 人机交互界面模板，旨在为开发者提供一个快速入门的框架，方便用户在此基础上进行重命名和重构。本项目借鉴了 Qt5 版本的 [ros_qt_demo](https://github.com/XKHoshizora/ros_qt_demo) 项目模板，并进行了改进和优化，以适应 Qt6 的新特性和功能。

## 安装依赖
1. 安装 ROS Noetic:
   - 方法一：
     ```bash
     sudo apt update
     sudo apt install ros-noetic-desktop-full
     ```
   - 方法二（推荐）：
     从 GitHub 克隆 [Auto ROS Installer](https://github.com/XKHoshizora/auto-ros-installer) 项目：
     ```bash
     git clone https://github.com/XKHoshizora/auto-ros-installer.git
     ```
     然后按照 `README.md` 的说明进行一键安装。

2. 安装 Qt6:
   - 方法一（推荐）：
      - 下载 Qt Installer：
        前往 [Qt 官方下载页面](https://www.qt.io/download-qt-installer) 下载适用于 Linux 的在线安装程序。
      - 授予执行权限：
        下载完成后，导航到下载的文件夹并授予安装程序执行权限：
        ```bash
        chmod +x qt-online-installer-linux-<x64/arm64>-<version>.run
        ```
      - 运行安装程序：
        执行安装程序，按照屏幕上的指示进行安装：
        ```bash
        ./qt-online-installer-linux-<x64/arm64>-<version>.run
        ```
      - 添加环境变量：
        在终端运行以下命令新建文件并打开编辑器：
        ```bash
        sudo nano /usr/bin/qtcreator
        ```
        在文件中写入以下内容：
        ```shell
        #!/bin/sh
        
        export QT_HOME=/home/<user>/Qt/Tools/QtCreator/bin
        $QT_HOME/qtcreator $*
        ```
        更改文件权限：
        ```bash
        sudo chmod a+x /usr/bin/qtcreator
        ```
        之后即可直接输入以下命令直接运行 Qt Creator：
        ```bash
        qtcreator
        ```

   - 方法二：
     ```bash
     sudo apt install qt6-base-dev
     ```

4. 安装其他必要依赖:
   如果在打开 Qt Creator 时出现问题，请先安装以下依赖：
   ```bash
   sudo apt install libxcb-xinerama0 libxcb-xinerama0-dev libxcb-cursor0
   ```

## 创建工作空间
```bash
mkdir -p ~/qt6_ros_template_ws/src
cd ~/qt6_ros_template_ws/src
```

## 克隆项目仓库
```bash
git clone https://github.com/XKHoshizora/Qt6-ROS-Template.git
cd Qt6-ROS-Template
```

## 构建项目
1. 初始化 catkin 工作空间:
    ```bash
    cd ~/qt6_ros_template_ws
    catkin_make
    ```
2. 构建 Qt6 项目:
    ```bash
    cd src/Qt6-ROS-Template
    mkdir build
    cd build
    cmake ..
    make
    ```

## 运行项目
1. 启动 ROS 节点:
    ```bash
    source ~/qt6_ros_template_ws/devel/setup.bash
    roslaunch qt6_ros_template demo.launch
    ```

## 修改和重命名
根据需要修改项目中的代码和配置文件，重命名相关类和文件以适应您的项目需求。

## 贡献
欢迎贡献！请 fork 本项目并提交 pull request。

## 许可证
本项目基于 MIT 许可证，请参阅 [LICENSE](LICENSE) 文件了解更多详情。
