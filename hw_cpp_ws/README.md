## 1. 作业一

在coneprojection功能包，试了试用类来实现

另有一个tools.hpp头文件，貌似没有什么用

使用

```
roslaunch coneprojection cone.launch
```

查看效果

![](imgs/2025-07-22%2011-47-01屏幕截图.png)

## 2. 作业二

在pointcloud功能包，同时订阅三个话题，用pcl拼起来，在转换成Pointloud2格式然后发布即可

```
roslaunch pointcloud vis.launch
```

查看效果

![](imgs/2025-07-22%2011-35-02屏幕截图.png)