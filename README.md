## Настройка:
1. Забрать в workspace/src/

2. В workspace вызвать catkin_make

3. В каждой запускаемой консоли вызывать source workspace/devel/setup.bash  

## Запуск:
1. Запуск симуляции: 
  roslaunch rrbot_gazebo rrbot_world.launch

2. Запуск управления:
  roslaunch pendulum_control pendulum.launch

Алгоритм управления реализован в pendulum_control/scripts/pendulum.py

Конкретно, в функции callback.
