# jetyak

## Requirements
- ros melodic
- python

## Installation
```
sudo apt install ros-melodic-mavlink
sudo apt install ros-melodic-geographic-msgs
sudo apt install libgeographic-dev
```

#### install packages in catkin_ws
```
git clone https://github.com/juancamilog/br24.git
git clone https://github.com/mavlink/mavros.git
catkin_make
```

#### pip install
```
pip install matplotlib
pip install opencv-python
pip install geopandas
pip install scipy
pip install utm
```

## Run

#### start roscore

```
roscore
```

#### run listener script

```
cd scripts
python my_listener.py
```

#### play bags
Download dataset from https://drive.google.com/file/d/1U8Xa5Jk4Rzh9-SgfYF97TdHZqxbLix5W/view?usp=sharing
```
cd your_dataset_folder
rosbag play *.bag
```
