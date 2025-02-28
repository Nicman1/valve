Commandes à faire après avoir clone le git
```
cd ~/valve_ws
colcon build
source ~/valve_ws/install/setup.bash
ros2 run valve_pkg valve_node
```
Dans un autre terminal ouvrir le controlleur
```
cd ~/valve_ws
source ~/valve_ws/install/setup.bash
ros2 run valve_pkg valve_controller
```
Peser sur enter dans ce terminal pour partir le timer et envoyer la commande 
à la node de la valve.
