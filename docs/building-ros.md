Before being able to use the ros workspace, you will have to follow the steps described [here](get-ready-to-develop.md).

You also have to:

- Build AirSim. From the root of this repository, run:
```
cd AirSim
./setup.sh
./build.sh
```
- Build ROS package

```
cd Simulator
catkin build
```

If your default GCC isn't 8 or greater (check using `gcc --version`), then compilation will fail. In that case, use `gcc-8` explicitly as follows:

```
catkin build -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
```