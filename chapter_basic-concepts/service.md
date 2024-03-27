# 服务详述
:label:`sec_service`

服务是一种基于请求/响应（request/response）模式的通信机制。这种机制允许一个节点（客户端）发送一个请求给另一个节点（服务端），服务端处理请求后返回一个响应给客户端。

服务端（Service Server）是一个节点，它提供了一个特定的服务。当收到客户端的请求时，服务端会处理这个请求，并产生一个响应。服务端可以同时处理多个来自不同客户端的请求。

客户端（Service Client）是一个节点，它向服务端发送请求并接收响应。客户端在发送请求时会阻塞，直到收到服务端的响应或者超时。客户端通常在需要服务端执行特定操作或查询信息时使用服务。

服务（Service）是一个命名的调用接口，用于在客户端和服务端之间进行请求/响应通信。它由一对消息定义：一个用于请求和一个用于响应。服务是通过ROS 2的接口定义语言（IDL）来定义的，支持多种编程语言。

请求（Request）是客户端发送给服务端的数据，它包含了客户端想要服务端执行的操作所需的所有信息。响应（Response）是服务端发送回客户端的数据，它包含了请求处理的结果或相关信息。

ROS 2使用DDS作为其底层通信机制，服务通信也是建立在DDS之上的。服务端在启动时会向ROS 2的发现服务器注册自己的服务。客户端在需要服务时，会通过发现服务器找到服务端，并建立通信连接。一旦连接建立，客户端就可以发送请求，服务端在处理完请求后发送响应。

服务通信机制是ROS 2中用于实现同步、确定性通信的一种机制。它适用于那些需要即时响应和对结果有确定要求的场景，如配置参数的读取和写入、远程过程调用等。

## 使用命令行工具操作服务

打开一个新终端并运行：

```bash
ros2 run turtlesim turtlesim_node
```

打开另一个终端并运行：

```bash
ros2 run turtlesim turtle_teleop_key
```

此时，两个节点``/turtlesim``和``/teolep_turtle``被启动。

在一个新终端中运行``ros2 service list``命令将返回系统中当前活动的所有服务的列表：

```
/clear
/kill
/reset
/spawn
/teleop_turtle/describe_parameters
/teleop_turtle/get_parameter_types
/teleop_turtle/get_parameters
/teleop_turtle/list_parameters
/teleop_turtle/set_parameters
/teleop_turtle/set_parameters_atomically
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```

此处可以看到，这两个节点都有相同的六个服务，它们的名称中都有“parameters”，这表明这些服务与参数有关。更多关于参数的内容将在下一节讨论，这里先忽略。

请关注特定于turtlesim的服务：``/clear``、``/kill``、``/reset``、``/spawn``、``/turtle1/set_pen``、``/turtle1/teleport_absolute``和``/turtle1/teleport_relative``。

服务具有描述服务的请求和响应数据结构的类型。服务类型的定义与话题类型类似，只是服务类型有两部分：一部分用于请求，另一部分用于响应。要查找服务的类型，请使用以下命令``ros2 service type <service_name>``，例如要查看turtlesim的/clear服务，需要在新终端中，输入以下命令：

```bash
ros2 service type /clear
```

该命令将返回如下信息：

```
std_srvs/srv/Empty
```

Empty类型表示服务调用在发出请求时不发送数据，在接收响应时不接收数据。

要同时查看所有活动服务的类型，可以将``-t``选项附加到list命令中：

```bash
ros2 service list -t
```

该命令将返回如下信息：

```
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
......
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
......
```

如果要查找特定类型的所有服务，可以使用命令``ros2 service find <type_name>``，例如要找到所有Empty类型的服务，需要输入以下命令：

```bash
ros2 service find std_srvs/srv/Empty
```

该命令将返回如下信息：

```
/clear
/reset
```

可以从命令行调用服务，但首先需要了解输入参数的结构，需要用到命令``ros2 interface show <type_name>``。比如``/spawn``，其类型是``turtlesim/srv/Spawn``，要查看``/spawn``服务的请求和响应参数，需要运行以下命令：

```bash
ros2 interface show turtlesim/srv/Spawn
```

该命令将返回如下信息：

```
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

三条横线将请求的数据结构（上）与响应的数据结构（下）分离。三横线上方的信息描述了调用/spawn所需的参数。x、y和theta决定了出生乌龟的2D位姿，而name显然是可选的。如果只是调用服务，三横线下面的信息并不需要知道，但是它可以帮助调用者理解从调用中所获响应的数据类型。

此时，可以使用以下方法调用服务：

```bash
ros2 service call <service_name> <service_type> <arguments>
```

``<arguments>``部分是可选的，例如Empty类型的服务就没有任何参数：

```bash
ros2 service call /clear std_srvs/srv/Empty
```

此命令将清除乌龟在turtlesim窗口中绘制的任何线条。

``/spawn``服务则带有参数，调用它可以通过设置参数来生成一只新海龟。命令行的服务调用中输入的``<arguments>``需要使用YAML语法，例如：

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```

该命令会返回服务的响应：

```
requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.2, name='')

response:
turtlesim.srv.Spawn_Response(name='turtle2')
```

turtlesim窗口将立即刷新，并产生一只新的乌龟。

## 编程实现服务端

**使用Python实现**

下面将开发一个简单的Python服务端节点，提供计算两个整数和的功能。导航到``~/ros2_ws/src``，利用之前创建的包``pkg_py_example``来实现相关功能。

按照惯例在每个ROS 2的Python包中使用与包同名的子目录，即：pkg_py_example子目录，来保存源代码。在该子目录中创建一个名为simple_service_server.py的新文件，并添加如下内容：

```python

```

**使用C++实现**



## 编程实现客户端

**使用Python实现**



**使用C++实现**



