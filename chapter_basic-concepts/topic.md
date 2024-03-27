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

按照惯例，ROS 2的C++包中，C++的函数和类的声明头文件位于include目录中与包同名的子目录中。C++源文件，即实现函数和类的行为的C++代码，位于src目录中。

在src目录中，新建一个名为simple_publisher.cpp的文件，添加如下内容，并保存。

```cpp
#include <rclcpp/rclcpp.hpp>       // 引入ROS2客户端库的核心头文件
#include <std_msgs/msg/string.hpp> // 引入std_msgs中String消息类型的头文件

#include <chrono> // 引入chrono库，用于处理时间相关的功能

using namespace std::chrono_literals; // 允许使用后缀"s"来表示秒

// 定义一个SimplePublisher类，继承自rclcpp::Node
class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher() : Node("simple_publisher"), counter_(0) // 构造函数，初始化节点名称和计数器
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
        auto message = std_msgs::msg::String(); // 创建一个String类型的消息
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
int main(int argc, char *argv[])
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

## 编程实现话题的订阅

**使用Python实现**

以下步骤将使用Python实现一个使用发布者-订阅者通信协议的订阅者节点，以接收并订阅一个话题，并读取通过它发布的消息。导航到``~/ros2_ws/src``，进入之前创建的包``pkg_py_example``的目录。

在与包名同名的子目录中创建文件simple_subscriber.py，并添加如下内容：

```python
import rclpy  # 导入rclpy库，这是Python的ROS2客户端库
from rclpy.node import Node  # 从rclpy.node模块中导入Node类，用于创建节点
from std_msgs.msg import String  # 从std_msgs.msg模块中导入String消息类型，用于发布和订阅字符串消息

# 定义一个SimpleSubscriber类，继承自Node类
class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__("simple_subscriber")  # 调用父类Node的构造函数，创建一个名为"simple_subscriber"的节点
        self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, 10)  # 创建一个订阅者，订阅名为"chatter"的话题，消息类型为String，回调函数为msgCallback，队列长度为10

    # 定义一个msgCallback方法，用于处理接收到的消息
    def msgCallback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)  # 使用get_logger方法获取日志记录器，并打印接收到的消息内容

# 定义一个main函数，作为程序的入口点
def main():
    rclpy.init()  # 初始化rclpy库

    simple_publisher = SimpleSubscriber()  # 创建一个SimpleSubscriber对象
    rclpy.spin(simple_publisher)  # 进入事件循环，处理节点上的回调函数

    simple_publisher.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭rclpy库

# 检查是否为主程序，如果是则运行main函数
if __name__ == '__main__':
    main()
```

与之前的步骤类似，为了正确构建脚本，还需要在setup.py文件中的entry_points部分添加新的可执行文件声明。声明一个名为simple_subscriber的可执行文件，具体如下：

```python
    entry_points={
        'console_scripts': [
            'simple_publisher = pkg_py_example.simple_publisher:main',
            'simple_subscriber = pkg_py_example.simple_subscriber:main',
        ],
    },
```

由于没有增加额外的依赖项，这里不需要修改package.xml文件。

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
ros2 run pkg_py_example simple_subscriber
```

此时，终端中并没有任何输出。

打开一个新的终端，运行：

```bash
ros2 topic list
```

可以看到``/chatter``话题是存在的。

如果手动向这个话题发布数据：

```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello ROS 2'"
```

可以看到当前终端正在以1Hz，向``/chatter``话题发送字符串。

切换回运行simple_subscriber的终端，可以看到该终端正在以1Hz，输出收到的字符串消息。

保持simple_subscriber运行，停止手动话题的发送。

打开一个新的终端，并source当前工作空间，运行C++实现的发布者程序。

```bash
ros2 run pkg_cpp_example simple_publisher
```

可以看到，simple_subscriber运行的终端打印出了它收到的所有消息。ROS 2的优点在于，它使得使用不同编程语言开发的执行不同操作的节点之间的通信成为可能，基本上不需要开发者为此付出任何努力。

**使用C++实现**

导航到``~/ros2_ws/src``，利用之前创建的包``pkg_cpp_example``来实现相关功能。

在src目录中新建文件simple_subscriber.cpp，并添加以下内容：

```cpp
#include <rclcpp/rclcpp.hpp>       // 导入rclcpp库，这是C++的ROS2客户端库
#include <std_msgs/msg/string.hpp> // 导入std_msgs消息库中的String消息类型

using std::placeholders::_1; // 使用C++标准库中的placeholders命名空间，用于绑定回调函数

// 定义一个SimpleSubscriber类，继承自rclcpp::Node类
class SimpleSubscriber : public rclcpp::Node
{
public:
    // 构造函数
    SimpleSubscriber() : Node("simple_subscriber")
    {
        // 创建一个订阅者，订阅名为"chatter"的话题，消息类型为std_msgs::msg::String，队列长度为10
        // 使用std::bind绑定回调函数msgCallback，_1代表传入回调函数的第一个参数（消息）
        sub_ = create_subscription<std_msgs::msg::String>(
            "chatter", 10, std::bind(&SimpleSubscriber::msgCallback, this, _1));
    }

private:
    // 定义一个共享指针，用于存储订阅者的句柄
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    // 定义一个msgCallback方法，用于处理接收到的消息
    // const关键字表示这个方法不会修改类的成员变量
    void msgCallback(const std_msgs::msg::String &msg) const
    {
        // 使用RCLCPP_INFO_STREAM宏打印接收到的消息内容
        // this->get_logger()获取节点的日志记录器
        // msg.data.c_str()将接收到的字符串消息转换为C风格字符串
        RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg.data.c_str());
    }
};

// 定义一个main函数，作为程序的入口点
int main(int argc, char *argv[])
{
    // 初始化rclcpp库
    rclcpp::init(argc, argv);
    // 创建一个指向SimpleSubscriber类对象的新共享指针
    auto node = std::make_shared<SimpleSubscriber>();
    // 将node其传递给rclcpp::spin进入事件循环
    rclcpp::spin(node);
    // 关闭rclcpp库
    rclcpp::shutdown();
    // 程序正常退出
    return 0;
}
```

注意，``createSubscription()``是一个模板函数，需要在这个函数的尖括号中指明将用于消息交换的接口类型。该函数的输出保存在SimpleSubscriber类的一个新对象的私有变量中。该变量是一个类型为``rclcpp::Subscription``的对象，这也是一个模板类，需要在这个类的尖括号中指明将用于消息交换的接口类型。这里不会创建这种类型的实例，而是一个名为sub_的共享指针。

在C++中，``std::bind``是一个函数适配器，它的主要作用是将一个可调用对象（如函数、函数对象、lambda表达式等）与其参数一起绑定，生成一个新的可调用对象，这个新的可调用对象可以带有预设的参数。在上面的代码中，``std::bind``用于绑定``SimpleSubscriber``类的成员函数``msgCallback``和类的实例（this指针），以及一个占位符``_1``，代表从话题中接收到的消息。这样，当消息到达时，``msgCallback``函数就会被调用，并且``this``指针指向正确的``SimpleSubscriber``实例，``_1``会被替换为实际接收到的消息。这是在C++中实现回调函数的常见做法。

接下来需要告诉编译器如何构建这个文件并将其变成在ROS 2中可执行的文件。为此，需要修改CMakeLists.txt文件，并在其中添加如下内容：

```cmake
add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)
```

告诉编译器编译目标为名为simple_subscriber的可执行文件，使用源文件src/simple_subscriber.cpp，且编译时需要链接rclcpp和std_msgs这两个依赖库。

最后，还需要其添加安装配置中，以便其能够被安装到正确的位置：

```cmake
install(TARGETS
  simple_publisher
  simple_subscriber
  DESTINATION lib/${PROJECT_NAME}
)
```

由于没有增加额外的依赖项，这里不需要修改package.xml文件。

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
ros2 run pkg_cpp_example simple_subscriber
```

此时，终端中并没有任何输出。

打开一个新的终端，运行：

```bash
ros2 topic list
```

可以看到``/chatter``话题是存在的。

如果手动向这个话题发布数据：

```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello ROS 2'"
```

可以看到当前终端正在以1Hz，向``/chatter``话题发送字符串。

切换回运行simple_subscriber的终端，可以看到该终端正在以1Hz，输出收到的字符串消息。

保持simple_subscriber运行，停止手动话题的发送。

打开一个新的终端，并source当前工作空间，运行Python实现的发布者程序。

```bash
ros2 run pkg_py_example simple_publisher
```

可以看到，simple_subscriber运行的终端打印出了它收到的所有消息，再一次验证了使用不同编程语言开发的执行不同操作的节点之间的通信能力。

## 接口与自定义消息

ROS应用程序主要通过三种类型的接口进行通信：话题、服务或动作。ROS 2使用一种简化的描述语言————接口定义语言（IDL）来描述这些接口。这种描述方式使得ROS工具能够轻松地为多种编程语言自动生成对应的接口源代码。

消息是话题通信机制中传递的数据，在ROS软件包的msg/目录下的.msg文件中进行描述和定义。.msg文件包含两部分内容：字段和常量。

每个字段包含一个类型和一个名称，两者之间由空格分隔，例如：

```
int32 my_int
string my_string
```

字段类型包括：

  * 内置类型
  * 自定义的消息描述名称，例如：“geometry_msgs/PoseStamped”

目前支持的内置类型有：

  * 布尔：bool
  * 字节：byte
  * 字符：char
  * 32位浮点数：float32
  * 64位浮点数：float64
  * 8位整数：int8
  * 8位无符号整数：uint8
  * 16位整数：int16
  * 16位无符号整数：uint16
  * 32位整数：int32
  * 32位无符号整数：uint32
  * 64位整数：int64
  * 64位无符号整数：uint64
  * 字符串：string
  * 宽字符串：wstring

每个内置类型都可以用于定义数组，ROS支持定义多种类型的数组：

  * 静态数组：static array
  * 无边界动态数组：unbounded dynamic array
  * 有边界动态数组：bounded dynamic array
  * 有边界字符串数组：bounded string

对于那些在ROS定义中更为宽松的类型，将通过软件实施ROS在范围和长度上的限制。

使用数组和有界类型定义消息的例子：

```
int32[] unbounded_integer_array
int32[5] five_integers_array
int32[<=5] up_to_five_integers_array

string string_of_unbounded_size
string<=10 up_to_ten_characters_string

string[<=5] up_to_five_unbounded_strings
string<=10[] unbounded_array_of_strings_up_to_ten_characters_each
string<=10[<=5] up_to_five_strings_up_to_ten_characters_each
```

字段名称应由小写字母和数字组成，使用下划线分隔单词。名称必须以字母开头，且不能以下划线结束，也不得包含连续的下划线。

可以为消息类型的任意字段设置默认值。目前，字符串数组和非内置的复杂类型（即上表中未列出的类型；这适用于所有嵌套消息）不支持设置默认值。

定义默认值的方法是在字段定义行的末尾添加第三个元素，例如：

```
uint8 x 42
int16 y -2000
string full_name "John Doe"
int32[] samples [-200, -100, 0, 100, 200]
```

注意，字符串的值需要用单引号’或双引号"括起来进行定义。目前，字符串的值不会被转义处理。

常量定义类似于具有默认值的字段描述，但这个值一旦设定便不能通过编程更改。常量的赋值是通过使用等号‘=’来表示的，例如：

```
int32 X=123
int32 Y=-123
string FOO="foo"
string EXAMPLE='bar'
```

注意，常量名称必须大写。

**编程实现自定义消息**

在编写自定义消息之前，首先查看已经安装的常用接口：

```bash
ros2 interface list
```

对于某一个接口，也可以通过命令行查看其具体定义。例如要查看消息geometry_msgs/msg/Point的定义，可以运行：

```bash
ros2 interface show geometry_msgs/msg/Point
```

上述命令将返回如下信息：

```
# This contains the position of a point in free space
float64 x
float64 y
float64 z
```

尽量使用系统提供的常用接口，不要重复定义接口。

接口定义规范了相互通信的多个ROS 2进程所交换数据的类型，因此会被工作空间中的多个相互通信的包所共同依赖。为了管理这样的依赖关系，通常会单独创建一个包，将接口独立进行定义和生成，而不将其放入具体使用这个接口的某个包中。

在ROS 2中，自定义消息的生成仍然依赖于CMake和ament_cmake，因为消息的生成涉及到从.msg文件生成C++和Python代码。下面，使用命令行工具创建一个名为pkg_interfaces的包，专门用于定义接口。

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake pkg_interfaces
```

进入新建的包，并在其中创建msg子目录。由于该包仅用于定义接口，而不实现任何其它功能，可以删除自动创建的include和src目录。

```bash
cd pkg_interfaces
mkdir msg
rm -rf include
rm -rf src
```

在msg子目录中创建一个名为TargetCoordinates.msg的文件，并在其中添加如下内容：

```
string name
geometry_msgs/Point coordinates
```

由于自定义消息依赖了geometry_msgs包，需要在package.xml文件中添加这一依赖。打开该文件，在``<test_depend>``上方添加：

```xml
<depend>geometry_msgs</depend>
```

为了能够根据定义自动生成接口程序，还需要继续添加如下内容：

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

为了正确编译，还需要编辑CMakeLists.txt文件。打开该文件，删除BUILD_TESTING部分和所有注释。在``find_package``之后添加如下内容：

```cmake
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetCoordinates.msg"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
```

现在可以构建工作空间了。当构成功建工作空间，并source当前工作空间后，再次运行：

```bash
ros2 interface list
```

可以发现，自定义的``pkg_interfaces/msg/TargetCoordinates``消息出现在接口列表中。
