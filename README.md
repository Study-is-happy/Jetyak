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

#### install packages in catkin_ws
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

#### start roscore

```
roscore
```

#### run python script

```
cd scripts
python2 my_listener.py
```

#### play bags
Download dataset from https://drive.google.com/file/d/1U8Xa5Jk4Rzh9-SgfYF97TdHZqxbLix5W/view?usp=sharing
```
cd your_dataset_folder
rosbag play *.bag
```
