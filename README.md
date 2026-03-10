# Project Manhattan

A school project for controlling robot demonstrating simple capabilities like motion and computer
vision.

## Environment Setup (macOS silicon)
Follow these steps to set up the development environment on macOS:

### 1. Install UTM (QEMU)

Download and install UTM Supervisor from:
https://mac.getutm.app

### 2. Install Ubuntu 22.04 Live Server (ARM)
1. Download the **Ubuntu 22.04 LIVE Server ISO for ARM.**
2. Create a **new virtual machine** in UTM using the ISO image.
    - Keep all installation options **default**.
    - Do not update installer.
    - At the end, **enable SSH host**
3. Boot the virtual machine and verify SSH:
```bash
ssh user@127.0.0.1 -p 2222
```
> If SSH does not work, add the following UTM arguments (you may need to stop machine):
```bash
-net nic -net user,hostfwd=tcp::2222-:22
```
> ⚠️ ROS domain functionality may require further configuration.

### 3. Configure ROS Environment
On the virtual machine modify `.bashrc` by adding there
```bash
source /opt/ros/humble/setup.bash
```

### 4. Install JetBrains Gateway
1. Install **JetBrains Toolbox** and launch **JetBrains Gateway**.
2. Navigate to Settings → Advanced Settings.
   - Under SSH, set a custom path to the OpenSSH tool.
   - Use `ssh -l` to launch the backend so ROS sourcing works.
3. Create a new project in Gateway
   - Click **Connections → SSH → New Project**
   - Enter the same SSH credentials (`user@127.0.0.1 -p 2222`)
   - Launch the console from CLion to prepare your workspace.

### 5. Prepare Workspace
1. Create a workspace folder
```bash
mkdir ~/ros_ws
cd ~/ros_ws
git clone <project-url> project
```
2. Select the project path in CLion and the setup is complete!

## 📄 Copyright

© 2025 xpitonm00, xpoulsa00. All rights reserved.

This repository is private and intended for use by authorized individuals only. 
Unauthorized copying, distribution, or use of any part of this codebase is strictly prohibited.


