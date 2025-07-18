创建功能包，依赖有std_msgs,geometry_msgs,rospy,roscpp

## 1. py脚本

在[官网](https://wiki.ros.org/cn/ROS/Tutorials/WritingPublisherSubscriber%28python%29)可以找到python的发布者模板，然后CmakeLists添加以下内容即可

```
catkin_install_python(PROGRAMS scripts/talker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

## 2. 