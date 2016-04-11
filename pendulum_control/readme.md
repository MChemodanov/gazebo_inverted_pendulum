Виртуальную машину брать [тут](http://owncloud.dnouglublenie.ru/index.php/s/jq7RLqghAxHjZEe) (9 Gb)

Инструкция:

1. Как создать workspace, если его еще нет:
```
   smtu@smtu-VirtualBox:~$ mkdir ws_w
   smtu@smtu-VirtualBox:~$ cd ws_w
   smtu@smtu-VirtualBox:~/ws_w$ mkdir src
   smtu@smtu-VirtualBox:~/ws_w$ cd src
   smtu@smtu-VirtualBox:~/ws_w/src$ catkin_init_workspace 
```
2. Как получить последнюю версию кода:
```
   smtu@smtu-VirtualBox:~$ cd ~/ws_w/src
   smtu@smtu-VirtualBox:~/ws_w/src$ git clone https://github.com/MChemodanov/Malachite_ros.git
   smtu@smtu-VirtualBox:~/ws_w/src$ mv Malachite_ros turtle
   smtu@smtu-VirtualBox:~/ws_w/src$ cd ..
   smtu@smtu-VirtualBox:~/ws_w$ catkin_make
```
3. Как запустить:
  
   Сначала добавляем нужные пути:
   ```
   smtu@smtu-VirtualBox:~/ws_w$ source devel/setup.bash 
   ```
  * Запускаем черепаху в режиме телеуправления (управляется стрелками клавиатуры из консоли)
  ```
  roslaunch turtle lesson1.launch
  ```

  * Запускаем черепаху, которая ездит по квадрату, не используя обратную связь.
  ```
  roslaunch turtle lesson2.launch
  ```
   Исходный код управляющей программы тут: ~/ws_w/src/turtle/scripts/square.py
  * Запускаем черепаху, которая ездит по шестиугольнику, используя обратную связь и алгоритм p-регулятора.
 
  ```
  roslaunch turtle lesson3.launch
  ```
  Исходный код управляющей программы тут: ~/ws_w/src/turtle/scripts/square_p.py
  
  В программе можно менять k_p и наблюдать, как изменяется поведение черепахи.

4. Добавляем динамику

  * Управление черепахой с клавиатуры. Параметры симулятора (масса, момент иннерции, коэффициент сопротивления задаются в файле lesson4_1.launch)
  ```
  roslaunch turtle lesson4_1.launch
  ```
  * Запускаем черепаху, которая ездит по шестиугольнику, используя обратную связь и алгоритм p-регулятора.
  ```
  roslaunch turtle lesson4_2.launch
  ```
  Исходный код управляющей программы тут: ~/ws_w/src/turtle/scripts/square_pid.py
  Реализован только П-регулятор. Остальное предлагается реализовать студентам.

  Параметры симулятора (масса, момент иннерции, коэффициент сопротивления) и регуляторов задаются в файле lesson4_2.launch

