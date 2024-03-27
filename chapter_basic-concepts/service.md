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

**定义服务接口**

下面将开发一个简单服务，用于提供计算两个整数和的功能。客户端发送一个包含两个整数的请求数据，服务端计算完毕后返回一个包含一个整数的响应数据。

服务接口在ROS软件包的srv/目录下的.srv文件中进行描述和定义。.srv文件用于描述一项服务，包括两个部分：请求和响应，其中的请求和响应是具体的消息声明。

在上一节新建的包pkg_interfaces中，新建子目录srv。

```bash
cd cd ~/ros2_ws/src/pkg_interfaces
mkdir srv
```

在srv子目录中创建一个名为AddTwoInts.srv的文件，并在其中添加如下内容：

```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

此接口并没有引入新的依赖，因此不需要修改package.xml文件。

为了正确编译此接口，需要修改CMakeLists.txt文件，在rosidl_generate_interfaces的部分添加接口定义文件的描述，具体如下：

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetCoordinates.msg"
  "srv/AddTwoInts.srv"
  DEPENDENCIES geometry_msgs
)
```

现在可以构建工作空间了。当构成功建工作空间，并source当前工作空间后，运行：

```bash
ros2 interface list
```

可以发现，自定义的``pkg_interfaces/srv/AddTwoInts``服务接口出现在接口列表中。

## 编程实现服务端

**使用Python实现**

导航到``~/ros2_ws/src``，利用之前创建的包``pkg_py_example``来实现相关功能。

按照惯例在每个ROS 2的Python包中使用与包同名的子目录，即：pkg_py_example子目录，来保存源代码。在该子目录中创建一个名为simple_service_server.py的新文件，并添加如下内容：

```python
import rclpy  # 导入rclpy库，这是ROS2的Python客户端库
from rclpy.node import Node  # 从rclpy.node模块中导入Node类，用于创建节点
from pkg_interfaces.srv import AddTwoInts  # 从pkg_interfaces服务接口中导入AddTwoInts服务类型

# 定义一个名为SimpleServiceServer的类，继承自Node类
class SimpleServiceServer(Node):

    def __init__(self):
        super().__init__("simple_service_server")  # 调用父类的构造函数，创建节点，节点名称为simple_service_server
        self.service_ = self.create_service(AddTwoInts, "add_two_ints", self.serviceCallback)  # 创建一个服务，服务类型为AddTwoInts，服务名为add_two_ints，回调函数为serviceCallback
        self.get_logger().info("Service add_two_ints Ready")  # 在日志中输出服务add_two_ints已准备好的信息

    # 定义服务回调函数
    def serviceCallback(self, req, res):
        self.get_logger().info("New Request Received a: %d, b: %d" % (req.a, req.b))  # 在日志中输出接收到的新的请求信息
        res.sum = req.a + req.b  # 计算两个整数的和，并赋值给响应对象的sum字段
        self.get_logger().info("Returning sum: %d" % res.sum)  # 在日志中输出返回的和
        return res  # 返回响应对象

# 定义主函数
def main():
    rclpy.init()  # 初始化rclpy

    simple_service_server = SimpleServiceServer()  # 创建SimpleServiceServer对象
    rclpy.spin(simple_service_server)  # 保持节点运行，等待服务请求
    
    simple_service_server.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭rclpy

# 判断是否为主程序
if __name__ == '__main__':
    main()  # 如果是主程序，则运行主函数
```

服务使用了``pkg_interfaces/srv/AddTwoInts``接口，因此引入了对pkg_interfaces包的依赖，需要在package.xml文件中添加``<depend>pkg_interfaces</depend>``来处理新增的这个依赖。

为了正确构建脚本，还需要在setup.py文件中的entry_points部分添加新的可执行文件声明。声明一个名为simple_service_server的可执行文件，具体如下：

```python
    entry_points={
        'console_scripts': [
            'simple_publisher = pkg_py_example.simple_publisher:main',
            'simple_subscriber = pkg_py_example.simple_subscriber:main',
            'simple_service_server = pkg_py_example.simple_service_server:main',
        ],
    },
```

现在可以构建工作空间了。当成功构建工作空间，并source当前工作空间后，运行``ros2 run pkg_py_example simple_service_server``启动服务端。

打开一个新终端，运行``ros2 service list``，可以看到``/add_two_ints``服务已经开启。

运行命令调用此服务：

```bash
ros2 service call /add_two_ints pkg_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

该命令会返回服务的响应：

```
requester: making request: pkg_interfaces.srv.AddTwoInts_Request(a=1, b=2)

response:
pkg_interfaces.srv.AddTwoInts_Response(sum=3)
```

响应结果显示，服务被成功调用，计算被正确执行和返回。

**使用C++实现**

导航到``~/ros2_ws/src``，利用之前创建的包``pkg_cpp_example``来实现相关功能。

在src目录中新建文件simple_service_server.cpp，并添加以下内容：

```cpp
#include <rclcpp/rclcpp.hpp>                   // 导入rclcpp头文件，这是ROS2的C++客户端库
#include "pkg_interfaces/srv/add_two_ints.hpp" // 导入自定义服务消息头文件

#include <memory> // 导入内存头文件，用于智能指针

using namespace std::placeholders; // 导入 placeholders 命名空间，用于 std::bind

// 定义一个名为SimpleServiceServer的类，继承自rclcpp::Node
class SimpleServiceServer : public rclcpp::Node
{
public:
    // 构造函数
    SimpleServiceServer() : Node("simple_service_server")
    {
        // 创建一个服务，服务类型为pkg_interfaces::srv::AddTwoInts，服务名为add_two_ints，回调函数为serviceCallback
        service_ = create_service<pkg_interfaces::srv::AddTwoInts>("add_two_ints", std::bind(&SimpleServiceServer::serviceCallback, this, _1, _2));
        // 在日志中输出服务add_two_ints已准备好的信息
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service add_two_ints Ready");
    }

private:
    // 声明一个服务句柄
    rclcpp::Service<pkg_interfaces::srv::AddTwoInts>::SharedPtr service_;

    // 定义服务回调函数
    void serviceCallback(const std::shared_ptr<pkg_interfaces::srv::AddTwoInts::Request> req,
                         const std::shared_ptr<pkg_interfaces::srv::AddTwoInts::Response> res)
    {
        // 在日志中输出新的请求接收到的信息
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "New Request Received a: " << req->a << " b: " << req->b);
        // 计算两个整数的和，并赋值给响应对象的sum字段
        res->sum = req->a + req->b;
        // 在日志中输出返回的和
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Returning sum: " << res->sum);
    }
};

// 定义主函数
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                            // 初始化rclcpp
    auto node = std::make_shared<SimpleServiceServer>(); // 创建SimpleServiceServer对象
    rclcpp::spin(node);                                  // 保持节点运行，等待服务请求
    rclcpp::shutdown();                                  // 关闭rclcpp
    return 0;                                            // 程序正常退出
}
```

其中，``create_service``函数创建了一个名为``add_two_ints``的服务，服务的类型为``pkg_interfaces::srv::AddTwoInts``，并且绑定了``SimpleServiceServer``类的``serviceCallback``成员函数作为回调函数。当服务接收到请求时，``serviceCallback``函数将被调用，请求和响应的参数通过``std::bind``自动传递给回调函数。``std::bind``通常用于将服务回调函数绑定到服务服务器上。当服务接收到请求时，``std::bind``会将请求和响应对象作为参数传递给回调函数。在这种情况下，``_1``和``_2``分别代表回调函数的第一个和第二个参数，即：``_1``代表服务请求（req），``_2``代表服务响应（res）。

服务使用了``pkg_interfaces/srv/AddTwoInts``接口，因此引入了对pkg_interfaces包的依赖，需要在package.xml文件中添加``<depend>pkg_interfaces</depend>``来处理新增的这个依赖。

为了正确编译脚本，还需要修改CMakeLists.txt文件，并在其中添加如下内容：

```cmake
find_package(pkg_interfaces REQUIRED)

add_executable(simple_service_server src/simple_service_server.cpp)
ament_target_dependencies(simple_service_server rclcpp pkg_interfaces)
```

告诉编译器引入pkg_interfaces库，并编译目标为名为simple_service_server的可执行文件，使用源文件src/simple_service_server.cpp，且编译时需要链接rclcpp和pkg_interfaces这两个依赖库。

最后，还需要在其install配置中添加``simple_service_server``，以便其能够被安装到正确的位置。

现在可以构建工作空间了。当成功构建工作空间，并source当前工作空间后，运行``ros2 run pkg_cpp_example simple_service_server``启动服务端。

打开一个新终端，运行``ros2 service list``，可以看到``/add_two_ints``服务已经开启。

运行命令调用此服务：

```bash
ros2 service call /add_two_ints pkg_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

该命令会返回服务的响应：

```
requester: making request: pkg_interfaces.srv.AddTwoInts_Request(a=1, b=2)

response:
pkg_interfaces.srv.AddTwoInts_Response(sum=3)
```

响应结果显示，服务被成功调用，计算被正确执行和返回。

## 编程实现客户端

**使用Python实现**

导航到``~/ros2_ws/src``，利用之前创建的包``pkg_py_example``来实现相关功能。

按照惯例在每个ROS 2的Python包中使用与包同名的子目录，即：pkg_py_example子目录，来保存源代码。在该子目录中创建一个名为simple_service_client.py的新文件，并添加如下内容：

```python
import sys  # 导入sys模块，用于访问与Python解释器相关的变量和函数
import rclpy  # 导入rclpy模块，这是ROS2的Python客户端库
from rclpy.node import Node  # 从rclpy.node模块中导入Node类，用于创建节点
from pkg_interfaces.srv import AddTwoInts  # 从bumperbot_msgs服务中导入AddTwoInts服务类型

# 定义一个名为SimpleServiceClient的类，继承自Node类
class SimpleServiceClient(Node):

    def __init__(self, a, b):
        super().__init__("simple_service_client")  # 调用父类的构造函数，创建节点，节点名称为simple_service_client
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")  # 创建一个客户端，用于调用add_two_ints服务

        # 等待服务服务器可用
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")  # 如果服务不可用，则每秒打印一次等待信息
        
        self.req_ = AddTwoInts.Request()  # 创建一个请求对象
        self.req_.a = a  # 设置请求对象的a字段
        self.req_.b = b  # 设置请求对象的b字段
        self.future_ = self.client_.call_async(self.req_)  # 发送异步请求
        self.future_.add_done_callback(self.responseCallback)  # 添加回调函数，用于处理响应

    # 定义响应回调函数
    def responseCallback(self, future):
        self.get_logger().info('Service Response %d' % future.result().sum)  # 打印服务响应的结果

# 定义主函数
def main():
    rclpy.init()  # 初始化rclpy

    # 检查命令行参数数量是否正确
    if len(sys.argv) != 3:
        print("Wrong number of arguments! Usage: simple_service_client A B")
        return -1

    # 创建SimpleServiceClient对象，传入命令行参数作为两个整数
    simple_service_client = SimpleServiceClient(int(sys.argv[1]), int(sys.argv[2]))

    rclpy.spin(simple_service_client)  # 保持节点运行，等待服务响应

    simple_service_client.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭rclpy

# 判断是否为主程序
if __name__ == '__main__':
    main()  # 如果是主程序，则运行主函数
```

由于在实现服务端时已经引入了对服务接口的依赖，没有引入新的依赖，因此不需要修改package.xml文件。

为了正确构建脚本，还需要在setup.py文件中的entry_points部分添加新的可执行文件声明。声明一个名为simple_service_server的可执行文件，具体如下：

```python
    entry_points={
        'console_scripts': [
            'simple_publisher = pkg_py_example.simple_publisher:main',
            'simple_subscriber = pkg_py_example.simple_subscriber:main',
            'simple_service_server = pkg_py_example.simple_service_server:main',
            'simple_service_client = pkg_py_example.simple_service_client:main',
        ],
    },
```

现在可以构建工作空间了。当成功构建工作空间，并source当前工作空间后，运行``ros2 run pkg_py_example simple_service_server``启动服务端。

打开一个新终端，运行``ros2 service list``，可以看到``/add_two_ints``服务已经开启。

在source当前工作空间后，启动客户端，并传递两个整数参数：

```bash
ros2 run pkg_py_example simple_service_client 1 2
```

该命令会打印服务的响应的结果：

```
[INFO] [1711546055.676060625] [simple_service_client]: Service Response 3
```

响应结果显示，服务被成功调用，计算被正确执行和返回。

**使用C++实现**

导航到``~/ros2_ws/src``，利用之前创建的包``pkg_cpp_example``来实现相关功能。

在src目录中新建文件simple_service_client.cpp，并添加以下内容：

```cpp
#include <rclcpp/rclcpp.hpp>                   // 导入rclcpp头文件，这是ROS2的C++客户端库
#include "pkg_interfaces/srv/add_two_ints.hpp" // 导入自定义服务消息头文件

#include <chrono> // 导入chrono头文件，用于处理时间相关的操作
#include <memory> // 导入memory头文件，用于智能指针

using namespace std::chrono_literals; // 使用chrono_literals命名空间，允许使用时间字面量，如1s
using std::placeholders::_1;          // 使用placeholders命名空间中的_1占位符，用于回调函数

// 定义一个名为SimpleServiceClient的类，继承自rclcpp::Node
class SimpleServiceClient : public rclcpp::Node
{
public:
    // 构造函数，接收两个整数参数
    SimpleServiceClient(int a, int b) : Node("simple_service_client")
    {
        client_ = create_client<pkg_interfaces::srv::AddTwoInts>("add_two_ints"); // 创建一个客户端，用于调用add_two_ints服务

        auto request = std::make_shared<pkg_interfaces::srv::AddTwoInts::Request>(); // 创建一个请求对象
        request->a = a;                                                              // 设置请求对象的a字段
        request->b = b;                                                              // 设置请求对象的b字段

        // 等待服务服务器可用
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting."); // 如果等待过程中被中断，则打印错误信息并退出
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again..."); // 如果服务不可用，则每秒打印一次等待信息
        }

        // 发送异步请求，并绑定响应回调函数
        auto result = client_->async_send_request(request, std::bind(&SimpleServiceClient::responseCallback, this, _1));
    }

private:
    rclcpp::Client<pkg_interfaces::srv::AddTwoInts>::SharedPtr client_; // 声明一个服务客户端共享指针

    // 定义响应回调函数
    void responseCallback(rclcpp::Client<pkg_interfaces::srv::AddTwoInts>::SharedFuture future)
    {
        if (future.valid())
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Service Response " << future.get()->sum); // 如果响应有效，则打印服务响应的结果
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service Failure"); // 如果响应无效，则打印服务失败的信息
        }
    }
};

// 定义主函数
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // 初始化rclcpp

    // 检查命令行参数数量是否正确
    if (argc != 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Wrong number of arguments! Usage: simple_service_client A B");
        return 1;
    }

    // 创建SimpleServiceClient对象，传入命令行参数作为两个整数
    auto node = std::make_shared<SimpleServiceClient>(atoi(argv[1]), atoi(argv[2]));
    rclcpp::spin(node); // 保持节点运行，等待服务响应
    rclcpp::shutdown(); // 关闭rclcpp
    return 0;           // 程序正常退出
}
```

由于在实现服务端时已经引入了对服务接口的依赖，没有引入新的依赖，因此不需要修改package.xml文件。

为了正确编译脚本，需要修改CMakeLists.txt文件，并在其中添加如下内容：

```cmake
add_executable(simple_service_client src/simple_service_client.cpp)
ament_target_dependencies(simple_service_client rclcpp pkg_interfaces)
```

告诉编译器编译目标为名为simple_service_client的可执行文件，使用源文件src/simple_service_client.cpp，且编译时需要链接rclcpp和pkg_interfaces这两个依赖库。

最后，还需要在其install配置中添加``simple_service_server``，以便其能够被安装到正确的位置。

现在可以构建工作空间了。当成功构建工作空间，并source当前工作空间后，运行``ros2 run pkg_cpp_example simple_service_server``启动服务端。

打开一个新终端，运行``ros2 service list``，可以看到``/add_two_ints``服务已经开启。

在source当前工作空间后，启动客户端，并传递两个整数参数：

```bash
ros2 run pkg_cpp_example simple_service_client 1 2
```

该命令会打印服务的响应的结果：

```
[INFO] [1711547354.643911465] [rclcpp]: Service Response 3
```

响应结果显示，服务被成功调用，计算被正确执行和返回。
