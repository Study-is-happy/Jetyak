# jetyak
pip install geopandas
pip install scipy
pip install scikit-image

sudo apt install ros-melodic-mavlink
sudo apt install ros-melodic-geographic-msgs
sudo apt install libgeographic-dev

git clone https://github.com/juancamilog/br24.git
git clone https://github.com/mavlink/mavros.git
catkin_make

python2 my_listener.py
