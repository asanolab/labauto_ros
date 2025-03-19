# setup memo
## ufactory conveyor
- Arduino setup
  1. Install arduino ide
  2. Install u8glib
  - online manual (ufactory conveyor)
    - https://github.com/uArm-Developer/Controller/blob/master/doc/uArmController%20User%20Manual(Geek%20Edition)20181228V1.0.1.pdf

- Arduino source code
  - sketchbookへのsymblic link を貼ってある
    - conveyor_control/sketchbook/conveyor_belt
  - link先のino
    - https://github.com/yuki-asano/Controller/blob/devel/scene_demo/conveyor_belt/src/conveyor_belt/conveyor_belt.ino

- ros_lib setup
```
cd /home/utokyo/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
```

- example
```
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200

start:
rostopic pub -1 /start_conveyor std_msgs/Bool "data: true" 

stop:
rostopic pub -1 /stop_conveyor std_msgs/Bool "data: true"
```

- topic
  - /is_conveyor_on: on/off status of the conveyor
