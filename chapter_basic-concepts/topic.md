# 话题详述
:label:`sec_topic`

话题（topic）是一种基于发布/订阅（publish/subscribe）模式的通信机制。这种机制允许不同的节点之间异步地交换数据，而无需知道彼此的存在。

发布者（Publisher）是一个节点，它定期（或根据事件）向一个特定的话题发布特定类型的消息（Message）数据。发布者不需要知道是否有订阅者接收数据，也不需要知道订阅者的数量。

订阅者（Subscriber）是一个节点，它订阅一个特定的话题以接收数据。当发布者向话题发布消息时，所有订阅了这个话题的订阅者都会接收到这些消息。订阅者不需要知道发布者的身份，也不需要知道是否有其他订阅者存在。

话题（Topic）是一个命名的通道，用于在发布者和订阅者之间传递消息。话题的消息类型必须由发布者和所有订阅者共同定义，以确保数据的兼容性。话题是ROS图中的一个关键组件，它们通过名称进行标识，通常使用反向域名风格的命名约定。

消息（Message）是发布者和订阅者之间传递的数据。每个话题都有一个与之关联的消息类型，定义了数据的结构。消息可以是简单的数据类型，如整数、浮点数、字符串，也可以是复杂的数据结构，包含多个字段和其他消息类型。

ROS 2使用DDS作为其底层通信机制，话题通信也是建立在DDS之上的。发布者和订阅者在启动时会向ROS 2的发现服务器注册自己，包括它们要发布或订阅的话题。发现服务器负责管理话题的名称和节点的注册信息，帮助发布者和订阅者建立连接。一旦发布者和订阅者之间建立了连接，它们就可以直接通信，不再需要发现服务器的介入。

综上，话题通信机制是ROS 2中最常用的数据传输方式，它提供了一种灵活、松耦合的机制，使得不同的节点可以轻松地交换数据，而无需关心彼此的实现细节。

## 使用命令行工具操作话题

打开一个新终端并运行：

```bash
ros2 run turtlesim turtlesim_node
```

打开另一个终端并运行：

```bash
ros2 run turtlesim turtle_teleop_key
```

默认情况下，这两个节点的名称为``/turtlesim``和``/teolep_turtle``。

为了可视化不断变化的节点和话题，以及它们之间的连接，请打开一个新终端并输入命令：

```bash
rqt_graph
```

如果将鼠标悬停在中心的话题上，则可以看到图像中的颜色高亮显示，如 :numref:`fig_rqt_topic_graph` 所示：

![节点/话题关系图](../img/rqt_topic_graph.png)
:label:`fig_rqt_topic_graph`

该图描述了``/turtlesim``节点和``/teleop_turtle``节点如何在一个话题上相互通信。``/teleop_turtle``节点正在向``/turtle1/cmd_vel``话题发布数据（用于移动乌龟的按键输入），``/turtlesim``节点订阅该话题以接收数据。rqt_graph的高亮显示功能在检查以多种不同方式连接的许多节点和话题的更复杂系统时非常有用。

在新终端中运行``ros2 topic list``命令将返回系统中当前活动的所有话题的列表：

```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

命令``ros2 topic list -t``将返回相同的话题列表，只不过话题消息类型将附加在方括号中：

```
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```

要查看关于某个话题的发布数据，可以使用``ros2 topic echo <topic_name>``。由于我们知道``/teleop_turtle``通过``/turtle1/cmd_vel``话题向``/turtlesim``发布数据，这里运行：

```bash
ros2 topic echo /turtle1/cmd_vel
```

起初，此命令不会返回任何数据。这是因为它正在等待/teleop_turtle发布一些内容。

返回turtle_teleop_key正在运行的终端，使用箭头移动乌龟。观察同时在运行echo的终端，可以看到每次移动的位置数据都在发布：

```
......
---
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -2.0
---
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
......
```

现在返回rqt_graph并取消选中Debug复选框，可以看到发布者正在cmd_vel话题上发布数据，并且有两个订阅者订阅了该话题。/_ros2cli_15762是刚刚运行的echo命令创建的节点（编号可能不同），如 :numref:`fig_rqt_topic_graph2` 所示。

![一个话题有两个订阅者](../img/rqt_topic_graph2.png)
:label:`fig_rqt_topic_graph2`

另一种了解这一点的方式是``ros2 topic info``命令：

```bash
ros2 topic info /turtle1/cmd_vel
```

该命令返回：

```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 2
```

节点使用消息通过话题发送数据。发布者和订阅者必须发送和接收相同类型的消息才能进行通信。``cmd_vel``话题的类型为：``geometry_msgs/msg/Twist``，这意味着在包``geometry_msgs``中有一个名为``Twist``的msg。

此时，可以在这个类型上运行``ros2 interface show <msg type>``来了解它的详细信息，即消息所期望的数据结构。

```bash
ros2 interface show geometry_msgs/msg/Twist
```

对于上面的消息类型，它会返回：

```
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
        float64 x
        float64 y
        float64 z
Vector3  angular
        float64 x
        float64 y
        float64 z
```

这意味着``/turtlesim``节点期望得到一个消息，其中包含两个向量，linear和angular，每个向量包含三个元素。参考之前用echo命令看到的``/teleop_turtle``发送给``/turtlesim``的数据，它的结构是相同的：

现在有了消息结构，可以使用以下命令直接从命令行将数据发布到话题上：

```bash
ros2 topic pub <topic_name> <msg_type> '<args>'
```

其中，```<args>```参数是将要传递给话题的实际数据，包装在所要求的结构中。需要注意的是，这个参数需要以YAML语法输入。完整的输入命令，如下所示：

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

其中，``–-once``是一个可选参数，意思是“发布一条消息然后退出”。终端中将显示以下输出：

```
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.8))
```

乌龟会像这样移动，如 :numref:`fig_topic_move_once` 所示：

![用命令行发送话题数据](../img/topic_move_once.png)
:label:`fig_topic_move_once`

乌龟需要稳定的数据流才能做连续运动。所以，为了让乌龟连续运动，可以运行：

```bash
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

这里的不同之处在于删除了``--once``选项并添加了``--rate 1``选项，这告诉``ros2 topic pub``以1 Hz的稳定速率连续发布数据。效果如 :numref:`fig_topic_move_1hz` 所示：

![用命令行连续发送话题数据](../img/topic_move_1hz.png)
:label:`fig_topic_move_1hz`

此时，``/turtle1/cmd_vel``话题具有多个发布者和多个订阅者。

对pose话题运行echo，``ros2 topic echo /turtle1/pose``，并重新检查rqt_graph，如 :numref:`fig_rqt_topic_pose` 所示：

![发送数据到pose话题](../img/rqt_topic_pose.png)
:label:`fig_rqt_topic_pose`

可以看到/turtlesim节点也在发布到pose话题，新的echo节点已经订阅了该话题。

对于一个话题，可以通过命令行查看其数据发布频率，例如：

```bash
ros2 topic hz /turtle1/pose
```

该命令将返回/turtlesim节点向pose话题发布数据的速率数据。

```
average rate: 60.914
        min: 0.000s max: 0.339s std dev: 0.01308s window: 615
```

至此，我们介绍了如何用命令行工具在终端和可视化界面中使用话题。此时，有许多节点正在运行。不要忘记在每个终端中输入``Ctrl+C``来停止它们。

## 编程实现话题的发布

**使用Python实现**

导航到``~/ros2_ws/src``，利用之前创建的包``pkg_py_example``来实现相关功能。

按照惯例在每个ROS 2的Python包中使用与包同名的子目录，即：pkg_py_example子目录，来保存源代码，也就是将要创建的Python脚本。

在该子目录中，创建一个名为simple_publisher.py的新文件。使用VSCode代开该文件，开始编写Python程序。

```python
import rclpy  # 导入rclpy库，这是ROS2的Python客户端库
from rclpy.node import Node  # 从rclpy.node模块导入Node类，用于创建节点
from std_msgs.msg import String  # 从std_msgs.msg模块导入String消息类型

# 定义一个SimplePublisher类，继承自Node类
class SimplePublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher")  # 调用父类Node的构造函数，创建一个名为"simple_publisher"的节点
        self.pub_ = self.create_publisher(String, "chatter", 10)  # 创建一个发布者，发布String类型的消息到"chatter"话题，消息队列长度为10
        self.counter_ = 0  # 初始化计数器为0
        self.frequency_ = 1.0  # 设置发布频率为1Hz
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)  # 输出日志信息，表示发布频率
        
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)  # 创建一个定时器，每隔1秒调用一次timerCallback函数

    def timerCallback(self):
        msg = String()  # 创建一个String类型的消息对象
        msg.data = "Hello ROS 2 - counter: %d" % self.counter_  # 设置消息内容为"Hello ROS 2 - counter: "加上计数器的值
        self.pub_.publish(msg)  # 发布消息
        self.counter_ += 1  # 计数器递增

# 定义main函数，程序的入口点
def main():
    rclpy.init()  # 初始化rclpy库
    
    simple_publisher = SimplePublisher()  # 创建SimplePublisher类的实例
    rclpy.spin(simple_publisher)  # 进入事件循环，等待回调函数执行
    
    simple_publisher.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭rclpy库

# 判断当前文件是否作为主程序运行，如果是，则执行main函数
if __name__ == '__main__':
    main()
```

保存上述脚本后，为了运行，首先需要告诉编译器如何构建该脚本，以便将其变成ROS 2中的可执行文件。

打开包``pkg_py_example``根目录中的setup.py文件，该文件可以被看作是一张指令表，告诉编译器如何将Python脚本转换成可执行文件。为此，编辑该文件，在entry_points（入口点）中的console_scripts（控制台脚本）列表中，添加一个名为simple_publisher的新的可执行文件，并保存。具体如下所示：

```python
    entry_points={
        'console_scripts': [
            'simple_publisher = pkg_py_example.simple_publisher:main',
        ],
    },
```

最后，在实际构建工作空间并启动节点之前，还需要声明新节点的依赖关系，即用于执行它的库。新节点的源代码中使用了rclpy和std_msgs包，因此需要在包``pkg_py_example``根目录中的package.xml文件中声明这些依赖关系。在``<test_depend>``之前添加如下内容并保存：

```xml
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
```

现在可以构建工作空间并执行新建的节点了。

打开一个新的终端，进入工作空间，运行：

```bash
colcon build
```

在运行新节点的可执行程序之前，首先需要source当前工作空间：

```bash
. install/setup.bash
```

运行节点中的可执行程序：

```bash
ros2 run pkg_py_example simple_publisher
```

打开一个新的终端，运行：

```bash
ros2 topic echo /chatter
```

可以看到

```
data: 'Hello ROS 2 - counter: 31'
---
data: 'Hello ROS 2 - counter: 32'
---
data: 'Hello ROS 2 - counter: 33'
```

话题收到了发布者发布的消息。

可以尝试使用命令行工具来获得有关这个新建话题更多的信息。

```bash
ros2 topic info /chatter --verbose
ros2 topic hz /chatter
```

这里使用verbose标志可以获取此话题的完整概览。

**使用C++实现**

导航到``~/ros2_ws/src``，利用之前创建的包``pkg_cpp_example``来实现相关功能。

按照惯例，ROS 2的C++包中，C++的函数和类的声明头文件位于include目录中与包同名的子目录中。C++源文件，即实现函数和类的行为的C++代码位于src目录中。

在src目录中，新建一个名为simple_publisher.cpp的文件，添加如下内容，并保存。

```cpp
#include <rclcpp/rclcpp.hpp>  // 引入ROS2客户端库的核心头文件
#include <std_msgs/msg/string.hpp>  // 引入标准消息包std_msgs中String消息类型的头文件

#include <chrono>  // 引入chrono库，用于处理时间相关的功能

using namespace std::chrono_literals;  // 允许使用后缀"s"来表示秒

// 定义一个SimplePublisher类，继承自rclcpp::Node
class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher() : Node("simple_publisher"), counter_(0)  // 构造函数，初始化节点名称和计数器
  {
    // 创建一个发布者，发布std_msgs::msg::String类型的消息到"chatter"话题，消息队列长度为10
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    // 创建一个定时器，每秒调用一次timerCallback函数
    timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));
    // 输出日志信息，表示发布频率为1Hz
    RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
  }

  // 定时器回调函数，当定时器触发时调用
  void timerCallback()
  {
    auto message = std_msgs::msg::String();  // 创建一个String类型的消息
    // 设置消息内容为"Hello ROS 2 - counter:"加上计数器的值，并将计数器递增
    message.data = "Hello ROS 2 - counter:" + std::to_string(counter_++);
    // 发布消息
    pub_->publish(message);
  }

private:
  // 创建一个std_msgs::msg::String类型的发布者共享指针
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  // 创建一个定时器共享指针
  rclcpp::TimerBase::SharedPtr timer_;
  // 创建一个无符号整型计数器
  unsigned int counter_;
};

// main函数，程序的入口点
int main(int argc, char* argv[])
{
  // 初始化ROS2客户端库
  rclcpp::init(argc, argv);
  // 创建SimplePublisher节点的一个共享指针实例
  auto node = std::make_shared<SimplePublisher>();
  // 进入事件循环，等待回调函数执行
  rclcpp::spin(node);
  // 关闭ROS2客户端库
  rclcpp::shutdown();
  // 返回0，表示程序正常退出
  return 0;
}
```

在运行上述代码之前，首先需要告诉编译器如何构建代码，并将其变成ROS 2的可执行文件。打开包目录下的CMakeLists.txt文件，为了便于阅读该文件，可以先删除文件中的所有注释。可以将这个文件视为一个指令表，用于告诉编译器如何将C++代码编译成可执行文件。

这里，需要声明代码中引入的依赖关系，即代码中使用的包，比如rclcpp和std_msgs，需要在这里声明它们的使用。在已有的find_package行下面添加如下两行：

```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```

在正确声明了所有的依赖项后，可以继续添加可执行文件的定义。这里添加名为simple_publisher的可执行文件，由src/simple_publisher.cpp源代码编译生成。此处还需要为simple_publisher目标添加依赖，确保在构建时链接到rclcpp和std_msgs。

```cmake
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)
```

正确编译生成可执行文件后，还需要正确进行安装。这里设置将其安装到lib文件夹中的一个与包同名的子文件夹中。在CMakeLists.txt文件中，``${PROJECT_NAME}``变量可以用于表示包名。具体如下：

```cmake
install(TARGETS
  simple_publisher
  DESTINATION lib/${PROJECT_NAME}
)
```

最后，还需要在package.xml中再次声明依赖关系。在``<test_depend>``之前添加如下内容并保存：

```xml
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
```

现在可以构建工作空间并执行新建的节点了。

打开一个新的终端，进入工作空间，运行：

```bash
colcon build
```

在运行新节点的可执行程序之前，首先需要source当前工作空间：

```bash
. install/setup.bash
```

运行节点中的可执行程序：

```bash
ros2 run pkg_cpp_example simple_publisher
```

打开一个新的终端，运行：

```bash
ros2 topic echo /chatter
```

可以看到

```
data: 'Hello ROS 2 - counter: 31'
---
data: 'Hello ROS 2 - counter: 32'
---
data: 'Hello ROS 2 - counter: 33'
```

话题收到了发布者发布的消息。

可以尝试使用命令行工具来获得有关这个新建话题更多的信息。

```bash
ros2 topic info /chatter --verbose
ros2 topic hz /chatter
```

