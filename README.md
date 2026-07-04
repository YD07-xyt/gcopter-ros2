    

## build
```bash
    sudo apt update
    sudo apt install cpufrequtils
    sudo apt install libompl-dev
```

mpc casadi
```bash
    sudo apt-get install swig
    git clone https://github.com/casadi/casadi.git
    cd casadi
    mkdir build
    cd build
    cmake -DWITH_PYTHON=ON -DWITH_PYTHON3=ON -DWITH_IPOPT=ON _DWITH_MUMPS=ON ..
    make
    sudo make install
```

```bash
wget http://fishros.com/install -O fishros && . fishros 
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build
```