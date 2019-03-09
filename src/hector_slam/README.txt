В ROS есть замечательный пакет по управлению картами map_server.
С его помощью например можно сохранить сгенерированную карту в файл.
Установим пакет:
sudo apt-get install ros-kinetic-map-server

Запустим узел map_saver из пакета:
rosrun map_server map_saver
rosrun map_server map_saver -f mymap

После выполнения будет создано два файла: map.pgm и map.yaml в текущей директории.
В файле map.pgm будет сохранена карта.
В файле map.yaml хранятся метаданные о карте: разрешение карты (количество метров на пиксель), путь до файла pgm и другие.

Очистить карту:
rostopic pub syscommand std_msgs/String "reset"


# hector_slam

See the ROS Wiki for documentation: http://wiki.ros.org/hector_slam
