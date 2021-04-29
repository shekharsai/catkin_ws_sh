# armadillo_navigation_upgrade
Additional configuration for armadillo navigation

## installation

First of all, clone the package to ~/catkin_ws/src. In new terminal use the command: 
```bash
$ cd $HOME/catkin_ws/src
$ git clone https://github.com/bguplp/armadillo_navigation_upgrade.git
$ sudo apt-get install ros-kinetic-teb-local-planner
$ cd ..
$ catkin_make
```
* armadillo2

copy armadillo2_for_navigation_upgrade.launch to ~/catkin_ws/src/armadillo/armadillo2/launch:
```bash
$ cp armadillo_navigation_upgrade/launch/armadillo2_for_navigation_upgrade.launch armadillo/armadillo2/launch
```

* armadillo1

copy armadillo1_for_navigation_upgrade.launch to ~/catkin_ws/src/armadillo/armadillo1/launch:
```bash
$ cp armadillo_navigation_upgrade/launch/armadillo1_for_navigation_upgrade.launch armadillo/armadillo1/launch
```

## runing

for the real robot use:
```bash
$ roslaunch armadillo_navigation_upgrade armadillo2.launch lidar:=true amcl:=true have_map:=true map:="<paht_to_map>/<map_name.yaml>" move_base:=true
```
### simulation 

#### armadillo2

launch armadillo2:
```bash
$ roslaunch armadillo2 armadillo2_for_navigation_upgrade.launch lidar:=true gazebo:=true world_name:="<paht_to_world>/<world_name.world>"
```

run amcl and move_base API's with known map:
```bash
$ roslaunch armadillo_navigation_upgrade armadillo_navigation_upgrade.launch amcl:=true have_map:=true move_base:=true map:="<paht_to_map>/<map_name.yaml>"
```

* dynamic reconfiguration for move_base, use the following commands:

to navigate through doors, use:
```bash
$ rosrun dynamic_cfg move_base_pass_through_doors.py
```

for basics configuration, use:
```bash
$ rosrun dynamic_cfg move_base_basic.py
```

* use navigation_services_for_simulation.py. 

run the file with:
```bash
$ rosrun armadillo_navigation_upgrade navigation_services_for_simulation.py 
```
and then call the service you need, for exmpel:
```bash
$ rosservice call /elevator_go "{}"
```

#### armadillo1

launch armadillo1:
```bash
$ roslaunch armadillo1 armadillo1_for_navigation_upgrade.launch lidar:=true gazebo:=true world_name:="<paht_to_world>/<world_name.world>"
```

run amcl and move_base API's with known map:
```bash
$ roslaunch armadillo_navigation_upgrade armadillo_navigation_upgrade.launch amcl:=true have_map:=true move_base:=true map:="<paht_to_map>/<map_name.yaml>"
```

* dynamic reconfiguration for move_base, use the following commands:

to navigate through doors, use:
```bash
$ rosrun dynamic_cfg move_base_pass_through_doors.py
```

for basics configuration, use:
```bash
$ rosrun dynamic_cfg move_base_basic.py
```

* use navigation_services_for_simulation.py. 

run the file with:
```bash
$ rosrun armadillo_navigation_upgrade navigation_services_for_simulation.py 
```
and then call the service you need, for exmpel:
```bash
$ rosservice call /elevator_go "{}"
```

### real robot 

#### armadillo2

launch armadillo2:
```bash
$ roslaunch armadillo2 armadillo2_for_navigation_upgrade.launch lidar:=true world_name:="<paht_to_world>/<world_name.world>"
```

run amcl and move_base API's with known map:
```bash
$ roslaunch armadillo_navigation_upgrade armadillo_navigation_upgrade.launch amcl:=true have_map:=true move_base:=true map:="<paht_to_map>/<map_name.yaml>"
```

* dynamic reconfiguration for move_base, use the following commands:

to navigate through doors, use:
```bash
$ rosrun dynamic_cfg move_base_pass_through_doors.py
```

for basics configuration, use:
```bash
$ rosrun dynamic_cfg move_base_basic.py
```

* use navigation_services_for_simulation.py. 

run the file with:
```bash
$ rosrun armadillo_navigation_upgrade navigation_services_for_simulation.py 
```
and then call the service you need, for exmpel:
```bash
$ rosservice call /elevator_go "{}"
```

#### armadillo1

launch armadillo1:
```bash
$ roslaunch armadillo1 armadillo1_for_navigation_upgrade.launch lidar:=true world_name:="<paht_to_world>/<world_name.world>"
```

run amcl and move_base API's with known map:
```bash
$ roslaunch armadillo_navigation_upgrade armadillo_navigation_upgrade.launch amcl:=true have_map:=true move_base:=true map:="<paht_to_map>/<map_name.yaml>"
```

* dynamic reconfiguration for move_base, use the following commands:

to navigate through doors, use:
```bash
$ rosrun dynamic_cfg move_base_pass_through_doors.py
```

for basics configuration, use:
```bash
$ rosrun dynamic_cfg move_base_basic.py
```

* use navigation_services_for_simulation.py. 

run the file with:
```bash
$ rosrun armadillo_navigation_upgrade navigation_services_for_simulation.py 
```
and then call the service you need, for exmpel:
```bash
$ rosservice call /elevator_go "{}"
```
