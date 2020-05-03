# Fourth simulation
## Detecting RFID antennas and making them spawn when we detect them

In this simulation we will detect RFID antennas positioned in an exact position. When the drone it is in range to detect them we will see the RFID antenna in the simulation. When the drone it is not more in range with the RFID antenna, the RFID antenna will disappear in the simulation. 

The code is very similar to the third simulation. You can see the changes in the **commander.py** file. It has been changed to see in a better way the RFID antennas. New entries have been added to **data.yaml**.

There are new files:
* /catkin\_ws/src/Firmware/RFID\_publish.py: This file creates a new node that will implement the RFID antennas. It will publish into a topic when the drone it is close to them. Also it calculates which it is the antenna of the drone who detects the RFID antenna. 
* /catkin\_ws/src/Firmware/Tools|sitl\_gazebo/models/RFID_1: This directory contains the sdf file to create the box that will simulate the RFID antenna. 



## Paths that are mandatory to be changed

In the following files change the part **'your_path'** with the path in your environment. 

In the file **listener/listener.cc**:

* char pathBack[] = "/**'your\_path'**/catkin_ws/src/Firmware/avoidobstacleback.py"; 
* char pathFront[] = "/**'your\_path'**/catkin_ws/src/Firmware/avoidobstaclefront.py";
* char pathRight[] = "/**'your\_path'**/catkin_ws/src/Firmware/avoidobstacleright.py";
* char pathLeft[] = "/**'your\_path'**/catkin_ws/src/Firmware/avoidobstacleleft.py";
* char pathUp[] = "/**'your\_path'**/catkin_ws/src/Firmware/avoidobstacleup.py";
* char pathDown[] = "/**'your\_path'**/catkin_ws/src/Firmware/avoidobstacledown.py";

In the file **catkin_ws/src/Firmware/avoidobstacleback.py**:

* yamlpath = "/**'your\_path'**/catkin_ws/src/Firmware/data.yaml"

In the file **catkin_ws/src/Firmware/avoidobstacledown.py**:

* yamlpath = "/**'your\_path'**/catkin_ws/src/Firmware/data.yaml"

In the file **catkin_ws/src/Firmware/avoidobstaclefront.py**:

* yamlpath = "/**'your\_path'**/catkin_ws/src/Firmware/data.yaml"

In the file **catkin_ws/src/Firmware/avoidobstacleleft.py**:

* yamlpath = "/**'your\_path'**/catkin_ws/src/Firmware/data.yaml"

In the file **catkin_ws/src/Firmware/avoidobstacleright.py**:

* yamlpath = "/**'your\_path'**/catkin_ws/src/Firmware/data.yaml"

In the file **catkin_ws/src/Firmware/avoidobstacleup.py**:

* yamlpath = "/**'your\_path'**/catkin_ws/src/Firmware/data.yaml"

In the file **catkin_ws/src/Firmware/commander.py**:

* yamlpath = "/**'your\_path'**/catkin_ws/src/Firmware/data.yaml"

In the file **catkin\_ws/src/Firmware/px4\_mavros\_run.py**:

* yamlpath = "/**'your\_path'**/catkin_ws/src/Firmware/data.yaml"

In the file **catkin\_ws/src/Firmware/data.yaml**:

* sdfpath: "/**'your\_path'**/catkin_ws/src/Firmware/Tools/sitl\_gazebo/models/RFID\_1/model.sdf.yaml"  

## Commands

In a terminal:

```
roslaunch px4 final_advanbird.launch
```

In a second terminal:

```
cd 'your_path'/listener/build
make 
./listener
```

In a third terminal:

```
roslaunch px4 final_pythonscripts.launch
```


## Commands file by file

In terminal 1:

```
roslaunch px4 posix_sitl.launch
```

In terminal 2:

```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

In terminal 3:

```
cd 'your_path'/listener/build
make 
./listener
```

In terminal 4:

```
cd 'your_path'/catkin_ws/src/Firmware
python px4_mavros_run.py
```

In terminal 5:

```
cd 'your_path'/catkin_ws/src/Firmware
python RFID_publish.py
```

In terminal 6:

```
cd 'your_path'/catkin_ws/src/Firmware
python commander.py
```





