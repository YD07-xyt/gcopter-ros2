    

## build
```bash
    sudo apt update
    sudo apt install cpufrequtils
    sudo apt install libompl-dev
```

```bash
wget http://fishros.com/install -O fishros && . fishros 
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build
```