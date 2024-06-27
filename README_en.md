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
   ./qt-online-installer-linux-
   ```
