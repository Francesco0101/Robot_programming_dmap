# Robot_programming_dmap

Questo progetto implementa un nodo ROS che:
- Riceve una mappa come una **Occupancy Grid** (`/map`)
- Riceve una **posa iniziale** (`/initialpose`)
- Riceve misure da un **laser scanner** (`/base_scan`)
- Usa un **Distance Map (DMap)** per **calcolare la posa del robot** e pubblicare una stima dell'**odometria corretta** su `/odom`.


## Installazione

### 1. Requisiti
- ROS1 con **Noetic** su Ubuntu 20.04 

### 2. Clonare la repository

```bash
git clone https://github.com/tuo-username/dmap_localization.git

```

### 3. Buildare il progetto 

```bash

source opt/ros/noetic/setup.bash

catkin build

source devel/setup.bash
```
## Ognuno dei punti successivi deve essere fatto su un terminale separato e usando i comandi
``` bash
source opt/ros/noetic/setup.bash
source devel/setup.bash
```
### 1. lancia il server centrale di ROS 

```bash
roscore
```

### 2. lancia il nodo 

```bash
rosrun dmap_localizer main_node

```

### 3. lancia il nodo per la localizzazione  

```bash
rosrun dmap_localizer main_node

```

### 4. pubblica la mappa  

```bash
rosrun map_server map_server <path/to/map.yaml>

```

### 5. lancia il visualizzatore 
```bash
rviz

```

### 6. lancia la simulazione dove è possibile muovere il robot

```bash
rosrun map_server map_server <path/to/map.yaml>

```
