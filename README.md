# gazebo_ros_demos
* Author: Dave Coleman <davetcoleman@gmail.com>
* License: GNU General Public License, version 3 (GPL-3.0)
* Inception Date: 4 June 2013
* Version: 1.0.0

Example robots and code for interfacing Gazebo with ROS

## Documentation and Tutorials
[On gazebosim.org](http://gazebosim.org/tutorials?cat=connect_ros)

## Develop and Contribute

We welcome any contributions to this repo and encourage you to fork the project then send pull requests back to this parent repo. Thanks for your help!
Настройка:
1. Забрать в <workspace>/src/
2. В <workspace> вызвать catkin_make
3. В каждой запускаемой консоли вызывать source <workspace>/devel/setup.bash  

Запуск:
1. Запуск симуляции: 
  roslaunch rrbot_gazebo rrbot_world.launch
2. Запуск управления:
  roslaunch pendulum_control pendulum.launch

Алгоритм управления реализован в pendulum_control/scripts/pendulum.py
Конкретно, в функции callback.

