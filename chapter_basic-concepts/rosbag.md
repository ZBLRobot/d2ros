# 数据记录与回放
:label:`sec_rosbag`

``ros2 bag``是一个命令行工具，用于记录系统中话题发布的数据。它累积在任意数量的话题上传递的数据，并将其保存在数据库中。然后，根据需要可以回放数据以重现测试和实验的结果。同时，分享记录的数据并共享给他人也是促进与不同团队开展技术合作的常见方式。

## 使用命令行记录和回放数据

打开一个新终端并运行：

```bash
ros2 run turtlesim turtlesim_node
```

打开另一个终端并运行：

```bash
ros2 run turtlesim turtle_teleop_key
```

此时，两个节点``/turtlesim``和``/teolep_turtle``被启动。

打开一个新终端，创建一个名为bag_files的目录，用于保存记录的文件。

```bash
mkdir bag_files
cd bag_files
```

运行命令``ros2 topic list``查看当前话题列表：

```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

此时，``/turtle_teleop``节点发布关于``/turtle1/cmd_vel``话题的命令，以使乌龟在turtlesim中移动。查看``/turtle1/cmd_vel``正在发布的数据，运行：

```bash
ros2 topic echo /turtle1/cmd_vel
```

一开始什么都不会出现，因为teleop没有发布任何数据。返回到运行teleop的终端，使其处于活动状态。使用箭头键移动乌龟，此时可以看到运行ros2 topic echo的终端上发布的数据。

```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```

记录发布到话题的数据，需要使用命令``ros2 bag record <topic_name>``。打开一个新终端并进入到之前创建的bag_files目录中，因为rosbag文件将保存在运行记录命令的目录中。运行命令：

```bash
ros2 bag record /turtle1/cmd_vel
```

现在ros2 bag正在记录/turtle1/cmd_vel主题上发布的数据。返回到teleop终端，再次移动乌龟。具体移动动作并不重要，但试着制作一个可识别的模式，以便稍后回放数据时查看。

按Ctrl+C停止录制。从执行记录命令到停止，终端输出了以下消息：

```
[INFO] [1711633912.266286569] [rosbag2_recorder]: Press SPACE for pausing/resuming
[INFO] [1711633912.268338073] [rosbag2_storage]: Opened database 'rosbag2_2024_03_28-21_51_52/rosbag2_2024_03_28-21_51_52_0.db3' for READ_WRITE.
[INFO] [1711633912.270201539] [rosbag2_recorder]: Listening for topics...
[INFO] [1711633912.270210677] [rosbag2_recorder]: Event publisher thread: Starting
[INFO] [1711633912.270447793] [rosbag2_recorder]: Recording...
[INFO] [1711633912.376753606] [rosbag2_recorder]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [1711633912.376861072] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...
[INFO] [1711633940.386251665] [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
[INFO] [1711633940.388463824] [rosbag2_recorder]: Event publisher thread: Exiting
[INFO] [1711633940.388719714] [rosbag2_recorder]: Recording stopped

```

数据将累积在当前目录的一个新的子目录中，子目录名为：rosbag2_year_month_day-hour_minute_second。该子目录将包含一个和目录同名的bag数据文件和一个名为metadata.yaml的文件。

利用上述命令还可以记录多个主题，以及更改ros2 bag保存的文件名称，例如：

```bash
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
```

``-o``选项允许定义输出文件的文件名。跟在后面的字符串（在本例中为subset）是文件名。

要一次录制多个主题，只需列出用空格分隔的每个主题即可。

也可以向命令中添加另一个选项``-a``，该选项将记录系统中的所有主题。




