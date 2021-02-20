# jetyak

## Requirements
- python2
- matplotlib
- opencv3
- ros melodic

## Installation
```
sudo apt install ros-melodic-mavlink
sudo apt install ros-melodic-geographic-msgs
sudo apt install libgeographic-dev
```

#### Install packages in catkin_ws
```
git clone https://github.com/juancamilog/br24.git
git clone https://github.com/mavlink/mavros.git
catkin_make
```

#### pip install
```
pip2 install geopandas
pip2 install scipy
pip2 install scikit-image
```

## Run
```
cd scripts
python2 my_listener.py

```

#### Play bag
```
rosbag play *.bag
```
