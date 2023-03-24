JeejioOS编译方式：

Windows平台：

打开命令行界面，在项目代码路径下执行命令初始化esp32c3编译环境。

   build\env.bat

Linux平台：

打开终端，执行命令进行编译环境安装：

   ./build/esp/install.sh

安装完成后，导入环境变量。注意，每次打开新的终端窗口进行编译前都需要运行：

   source build/env.sh

1. 执行命令进行编译

   build

2. 连接esp32c3开发板，直接执行以下命令进行刷机。

   flash

3. 可选命令：

   config  -  参数配置图形化界面

   monitor  -  monitor调试工具

   clean   -  清除编译产物(删除out目录)


注意事项：
   Linux平台首次执行flash命令下载镜像可能会由于权限问题无法打开串口设备，需要执行

   sudo usermod -aG dialout $USER

   重启设备后即可正常flash