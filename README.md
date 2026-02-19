# ARGoS3 TurtleBot 4
This repository contains ARGoS3 plugins for the TurtleBot 4.

<p align="center">
  <img src="./turtlebots4.gif" width="500" alt="Turtlebot4 simulation snapshot">
</p>

## Prerequisites
- Install ARGoS3 from the official [repository](https://github.com/ilpincy/argos3) so that headers, libraries, and the `argos3` executable are present on your system.
- Optional (but recommended): skim through the [developer manual](https://www.argos-sim.info/dev_manual.php) to understand how ARGoS3 expects plugins to be structured.

## Build and Install
Clone this repository and navigate to the root directory:

```bash
git clone https://github.com/CPS-Konstanz/argos3-turtlebot4.git
cd ~/argos3-turtlebot4
```

Use the helper script or run the commands manually:

```bash
# configures and builds in ./build (Debug by default)
./build.sh           

# same as above plus sudo make install
./build.sh install   
```

Manual steps:

```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)

# to install libraries/headers into your ARGoS3 prefix
sudo make install    
```

If you prefer running without `sudo make install`, set `ARGOS_PLUGIN_PATH` to `<repo>/build/argos3/plugins/robots/<plugin>` before launching `argos3` so it can find the shared libraries.
```bash
export EXAMPLEDIR=../argos3_plugins/new_robots
export ARGOS_PLUGIN_PATH=$EXAMPLEDIR/build/newepuck
```

## Repository Layout
- `argos3/plugins/robots/turtlebot4`: implementation of the TurtleBot 4 plugin 
- `argos3/testing`: folder contain an example controller, experiment and loop function to test the plugin.
- `build.sh`: convenience script for rebuilding and optionally installing.


## Sensors
### Sensor implemented and tested
- LiDAR
- Proximity sensors (i.e., infrared sensors)
- Light sensors
- Ground sensors

### Sensors implemented to be tested
- Camera

## Actuators
The robot is implemented as a differential drive robot, the example controller provieded (see example) shows how to convert the linear and angular velocity commands into left and right wheel velocities. 


## Run the Sample Experiment
The `testing` folder contains a simple obstacle-avoidance experiment using the TurtleBot 4 plugin. Obstacle avoidance is based only on the proximity sensors.
To run the experiments:

```bash
cd ~/argos3_turtlebot4
argos3 -c testing/experiments/turtlebot4_test.argos
```

## TODO
- [ ] Test the camera already implemented in the plugin
- [ ] Check credits and license of the name in the files
- [ ] Improve the Build and Install instructions
