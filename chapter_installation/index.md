# 安装
:label:`chap_installation`

ROS 2 Humble支持以下几种操作系统：

  * Ubuntu Linux
  * Windows
  * RHEL
  * macOS

建议使用Ubuntu Linux，推荐采用Debian软件包的安装方法（Debian软件包方法是Ubuntu所独有的）。这种方式更便捷，因为它会自动安装所需的依赖项。同时，它还会随着系统的常规更新而同步更新。

## 安装前的准备工作

ROS 2 Humble依赖Ubuntu22.04操作系统，因此首先需要安装该操作系统。注意，尽量使用实际物理机器安装，因为虚拟机或WSL可能在使用某些底层功能时有一些限制。

推荐从清华源下载Ubuntu镜像进行安装：https://mirrors.tuna.tsinghua.edu.cn/ubuntu-releases/22.04/ubuntu-22.04.4-desktop-amd64.iso

安装完成后，先更新系统。ROS 2软件包建立在频繁更新的Ubuntu系统上。在安装新软件包之前，始终建议您确保系统是最新的。

```bash
sudo apt update    # 更新软件源
sudo apt upgrade   # 升级已安装的软件包
```

如果您处于最小环境（例如Docker容器），则该系统的语言环境可能是最小的Posix。我们需要测试一下是否您您的系统使用的是不同的UTF-8支持的语言环境。如果不是，需要改为UTF-8支持的语言环境。

```bash
locale  # 查看是否支持UTF-8

sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # 确认设置
```

## 安装ROS 2 Humble

需要将ROS 2的软件源添加到系统中。先确保Ubuntu Universe软件库已启用。

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

然后使用apt添加ROS 2的GPG密钥。

```bash
sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

然后将存储库添加到软件源列表中，并更新软件源。

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

ROS 2提供了多种软件包的安装组合，包括：

  * 桌面安装（推荐），包括：ROS，RViz，演示，教程。
  * 基本安装（裸机），包括：通信库、消息包、命令行工具。没有GUI工具。
  * 仅开发工具，包括：用于构建ROS包的编译器和其他工具。

```bash
sudo apt install ros-humble-desktop
# sudo apt install ros-humble-ros-base
# sudo apt install ros-dev-tools
```

ROS 2依赖于使用shell环境组合的工作空间（workspace）。“工作空间”是ROS术语，指用于组织、构建和管理ROS软件包的目录结构。工作空间允许开发者创建一个独立的环境来开发和测试ROS应用程序。ROS2的核心工作空间称为底层（underlay），后续的本地工作空间称为叠加层（overlays）。当使用ROS 2进行开发时，通常会同时有几个工作空间处于活动状态。

组合工作空间可以更容易地针对不同版本的ROS 2或针对不同的软件包组合进行开发。通常，需要先source安装的ROS 2，加载底层，然后再source特定包的叠加层。注意，如果不source安装的ROS 2，则无法访问ros2命令，也无法找到或使用ROS 2软件包。换句话说，您将无法使用ROS 2。

您需要在打开的每个新shell上运行此命令才能访问ros2命令，如下所示：

```bash
source /opt/ros/humble/setup.bash
```

如果不想每次打开一个新的shell都要source安装文件，那么可以将该命令添加到shell启动脚本中：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
bash
```

该步骤将会设置几个运行ROS 2所必须的环境变量。可以使用下面的命令检查是否设置了ROS_DISTRO和ROS_VERSION等变量。

```bash
printenv | grep -i ROS
```

## 安装后的额外步骤

**确认安装成功**

如果你安装的是ros-humble-desktop，你可以尝试一些例子。

在一个终端中运行C++ talker：

```bash
ros2 run demo_nodes_cpp talker
```

在另一个终端中运行Python listener：

```bash
ros2 run demo_nodes_py listener
```

你应该看到talker输出发布的消息，而listener输出收到的消息。这将验证C++和Python API是否正常工作。

考虑到运行ROS 2的程序通常需要用到多个终端窗口，建议安装terminator。

```bash
sudo apt install terminator
```

安装后再次打开终端，你可以试试Contrl+Shift+E（垂直分割窗口），和Control+Shift+O（水平分割窗口）。

**为开发做准备**

ROS 2支持多种编程语言，最常用的是Python和C++。目前，ROS 2的绝大部分的功能均同时提供Python和C++接口，只有极少数功能只提供C++接口。建议安装conda来管理开发环境，它既能管理Python包，也能管理C++包。

最简单的方法就是安装依赖Python 3.10的Miniconda（因为Ubuntu 22.04系统自带的Python版本为Python3.10，需要保持环境依赖的一致性）。如果已安装conda，则可以跳过以下步骤。

访问清华源镜像网站（https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/），下载基于Python3.10的最新的Miniconda版本。

```bash
wget https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/Miniconda3-py310_24.1.2-0-Linux-x86_64.sh
sh Miniconda3-py310_24.1.2-0-Linux-x86_64.sh -b
~/miniconda3/bin/conda init
bash
```

更新一下conda和pip这两个命令行包管理器，并将pip的软件源更换为清华源。

```bash
conda update conda
conda update pip
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
```

基于ROS 2编写的源代码，无论Python还是C++，都需要使用colcon构建工具进行编译打包。ROS 2默认是没有安装colcon的，需要手动安装，并配置环境变量。

```bash
pip install colcon-common-extensions
echo "source /home/<current_user>/miniconda3/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
echo "source /home/<current_user>/miniconda3/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```

注意，需要将上面命令行中的<current_user>更换为自己系统的当前用户名。

开发环境对开发效率有很大的影响，我们推荐使用VSCode来做ROS 2开发。访问VSCode的官网（https://code.visualstudio.com/），下载合适的安装包进行安装。

打开VSCode，并安装如下扩展：

  * C++
  * Python
  * CMake
  * CMake Tools
  * XML
  * ROS

如果使用本地安装的VSCode开发远端的ROS 2系统，则需要进行如下设置：

1、在远端计算机上安装ssh服务

```bash
sudo apt install ssh
```

2、在本地打开命令行窗口，并生成ssh密钥

```bash
cd ~/.ssh
ssh-keygen -t rsa -f my.pem
```

3、从本地上传公钥到远程服务器

```bash
sftp <username>@<remote-ip>
cd .ssh
put my.pem.pub
exit
```

4、添加公钥到authorized_keys文件中

```bash
ssh <username>@<remote-ip>
cd .ssh
cat my.pem.pub >> authorized_keys
exit
```

5、验证免密登录

```bash
ssh -i my.pem <username>@<remote-ip>
exit
```

6、在本地的VSCode上安装Remote-SSH扩展，并在插件的config配置文件，添加如下内容：

```
Host remote-ros2
  HostName <remote-ip>
  User <username>
  IdentityFile C:\Users\<local-username>\.ssh\my.pem
```

7、重新打开VSCode并尝试使用Remote-SSH扩展连接远端机器
