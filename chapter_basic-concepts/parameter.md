# 参数服务器详述
:label:`sec_parameter`

参数服务器（Parameter Server）是一个用于存储和检索参数的分布式系统。参数服务器允许节点在运行时动态地共享和访问配置信息，如硬件设置、算法参数、系统配置等。与ROS 1中的集中式参数服务器不同，ROS 2的参数服务器是分布式系统的一个组成部分，这意味着参数的存储和检索是在节点本地进行的，但参数的更改会通过DDS传播到所有订阅了相应参数的节点。

参数服务器存储了键值对，其中键是一个唯一的字符串，值可以是任何支持的数据类型，如整数、浮点数、字符串、列表、字典等。每个节点都可以在其本地存储参数，并可以通过网络与其他节点的参数进行同步。

节点可以设置（set）、获取（get）和删除（delete）参数。参数操作通常用于节点启动时加载配置，或者在运行时根据某些条件动态调整配置。节点可以注册参数事件监听器，以便在参数更改时收到通知。参数事件监听器允许节点响应参数的动态变化，例如更新内部状态或重新配置系统。参数的更改会通过DDS传播到所有订阅了相应参数的节点。

参数服务器支持层级结构的参数名，这允许在逻辑上组织参数，例如将相关的参数分组到共同的命名空间下。参数可以具有不同的生命周期，例如有些参数可能在节点启动时设置一次，而其他参数可能会频繁更改。参数服务器支持不同的QoS设置，以控制参数通信的可靠性、持久性和实时性。每个节点会在其本地存储参数的副本，这样可以减少对网络的依赖，并提高访问速度。

参数服务器为机器人系统提供了一种强大的配置和管理工具。它允许开发者轻松地部署和维护复杂的机器人系统，同时确保了系统配置的灵活性和一致性。通过这种方式，开发者可以更专注于实现机器人的功能和行为，而不是处理配置和管理的问题。