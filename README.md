# jetyak

### Requirements
- python2
- matplotlib
- opencv3
- ros melodic

### Installation
```
sudo apt install ros-melodic-mavlink
sudo apt install ros-melodic-geographic-msgs
sudo apt install libgeographic-dev

git clone https://github.com/juancamilog/br24.git
git clone https://github.com/mavlink/mavros.git
catkin_make

pip install geopandas
pip install scipy
pip install scikit-image
```

### Run
```
python2 my_listener.py
```


