# 工作空间与包管理
:label:`sec_workspace`

## 创建一个工作空间

工作空间是一个具有特定结构的目录。通常包含一个src子目录，该子目录是ROS 2包的源代码所在的位置。通常情况下，该子目录开始时为空。

首先，创建一个目录（ros2_ws）来包含我们的工作空间。打开一个终端窗口，运行：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

此时，工作空间中包含一个空目录src。让我们使用colcon来构建这个空的工作空间。

```bash
colcon build
```

构建完成后，应该看到新生成了build、install和log这3个目录。所有在本工作区中的包，构建后都会在install目录中安装。

## 创建一个包

“包”（package）是组织代码和资源的基本单元。一个包可以包含一个或多个节点，以及这些节点所需的配置文件、库、数据集等资源。

如前所述，包应放在工作区的src子目录内。所以让我们导航到src目录，并在该目录内创建一个新的包。可以使用命令ros2 package create，后跟用于创建包所需的构建工具参数，来生成一个空的包。

```bash
# 创建一个Python包
ros2 pkg create --build-type ament_python pkg_py_example
# 创建一个C++包
ros2 pkg create --build-type ament_cmake pkg_cpp_example
```

此时，在src目录中生成了两个子目录：pkg_py_example和pkg_cpp_example。返回到工作空间，再次进行构建。

```bash
cd ..
colcon build
```

现在这个命令将重新构建整个工作区，包括我们刚刚创建的两个新包。在调用这两个包之前，需要先激活当前工作区，即让ROS 2将当前工作区识别为叠加层（overlays）。这样，我们已经添加的以及将要在这个工作区中添加的所有包都将被识别为ROS 2包。之后，我们就能够使用ros2命令执行其中的内容。

为此，我们需要source一个位于install文件夹中的特定文件：

```bash
source install/setup.bash
```

注意，执行source后，只有在当前终端的上下文环境中当前工作区被加载。新打开的终端，需要从新source才行。

为了验证工作区是否加载成功，以及ROS 2是否识别到了我们刚刚创建的两个新包，可以使用命令：

```bash
ros2 package list
```

该命令列出了所有当前可用的包。可以看到两个新包出现在输出列表中。如果我们打开一个新的终端，列出所有当前可用的包，可以发现两个新包并没有出现在输出列表中。

记住，无论何时打开一个新的终端，在使用工作区中的包之前，必须先source工作区。

## ROS图和ROS节点

ROS图（Graph）是指所有正在运行的节点以及它们之间的通信关系的集合，是所有组件同时处理数据的网络。这个图包括了节点之间的所有连接，如：话题（Topics）、服务（Services）、动作（Actions）和参数服务器（Parameter Server）的交互。ROS图是动态的，它可以随着节点的启动和关闭而变化，也可以随着节点之间建立或断开连接而变化。

节点（Node）是ROS图中的基本执行单元，它是运行在操作系统上的一个进程，可以发布或订阅话题、提供或请求服务、执行动作等。节点通过ROS的通信机制与其他节点进行交互，这些交互关系定义了节点在ROS图中的连接。

一个完整的机器人系统由许多协同工作的节点组成。在ROS 2中，单个可执行文件（C++程序、Python程序等）可以包含一个或多个节点。

命令ros2 run <package_name> <executable_name>用于从包中启动一个可执行文件，例如：

```bash
ros2 run turtlesim turtlesim_node
```

这里，包的名称是turtlesim，可执行文件的名称是turtlesim_node。然而，我们仍然不知道节点名称。

可以使用ros2 node list命令查看节点名称。该命令将显示所有正在运行的节点的名称。当您想要与一个节点交互时，或者当您的系统运行许多节点并需要跟踪它们时，这尤其有用。

当turtlesim仍在运行时，打开一个新终端，并输入以下命令：

```bash
ros2 node list
```

终端将返回节点名称：

```
/turtlesim
```

在运行可执行文件时，重映射允许您将默认节点的属性（如节点名称、主题名称、服务名称等）重新分配给自定义值。例如，要重新分配/turtlesim节点的名称，可以在新终端中运行以下命令：

```bash
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

此时返回到运行ros2 node list的终端，并再次运行它，您将看到两个节点名称：

```
/my_turtle
/turtlesim
```

当知道各节点的名称后，可以通过命令行ros2 node info <node_name>访问节点的更多信息，例如：

```bash
ros2 node info /my_turtle
```

该命令返回此节点的订阅者、发布者、服务和操作的列表，即与此节点交互的ROS图连接。
