# pubsub-benchmark
Baseline performance metrics for various on-device publisher-subscriber implementations.

Focused on edge computing (computer vision) use cases.

Python 3.8.10 & Ubuntu 20.04 or 18.04 used throughout.

## Variables

+ Frameworks
   + [Python Multiprocessing Queue](https://docs.python.org/3/library/multiprocessing.html?highlight=multiprocessing%20queue#multiprocessing.Queue)
   + [Python Asyncio](https://docs.python.org/3/library/asyncio.html)
   + [ROS2](https://docs.ros.org/) [Humble](https://docs.ros.org/en/rolling/Releases/Release-Humble-Hawksbill.html#humble-hawksbill-humble)
   + [commaai/cereal](https://github.com/commaai/cereal)
+ Message Sizes
   + Small: Single `char`
   + Medium: `dict` containing a handful of scalars/floats
   + Large: `numpy` array of RGB image
+ Devices
  + Desktop (Ubuntu 20.04)
  + NVIDIA Jetson Xavier NX (Ubuntu 18.04)
  + NVIDIA Jetson Nano (Ubuntu 18.04)
  
## Benchmarking Strategy

Preliminary considerations:
+ Time synchronization cannot be relied on between processes, so timer start/stop should happen within one process.
+ Repeat publish-subscribe `n` times should reduce relative noise from benchmarking logic.
+ Nice to have: use [scalene](https://github.com/plasma-umass/scalene) to do performance profiling.

![](strategy.drawio.svg)

## Run All Benchmarks

Clone this repository
```
git clone https://github.com/tensorturtle/pubsub-benchmark.git
```

Install `poetry` and Docker:
```
pip3 install poetry
```

[Install Docker Engine (link for Ubuntu)](https://docs.docker.com/engine/install/ubuntu/)


To run the full suite:
```
python3 main.py
```
This script creates a sqlite3 database `pubsub_results.db` to store the results.
Then, it creates plots, which are stored in `result_plots` directory.

To run them separately, continue reading.

## Run Individual Benchmarks

### Python Builtins (asyncio, multithreading)

#### Prerequisites 

Install Python

For Ubuntu:
```
sudo apt install python3 python3-pip
```

Install poetry

```
pip3 install poetry
```

#### Run benchmark

```
cd python_builtins
poetry install
poetry run python3 python_builtins/main.py
```

**Notes**
+ At high speeds, Python multiprocessing queue gets concurrency errors (probably need to use locks)

### commaai/cereal

We use the [official Dockerfile](https://github.com/commaai/cereal/blob/master/Dockerfile) to set up cereal within Docker.


Simply:

```
./run_cereal.sh
```

(or, if you want to build the docker image yourself, run `./build_cereal.sh` before that)

Inside the docker shell:
```
cd /benchmark
python3 pub.py
```

In another docker shell,
```
cd /benchmark
python3 sub.py
```

#TODO: Write capnp and services.py to match testing data


## ROS2 Humble

#TODO: implement medium messages with custom msg definition
#TODO: databse logging

We use the community-contributed ROS2 Docker image from OSRF

ROS2 requires messages definitions, which are at `custom_msgs/` in this repo.

```
cd ros2_bench
docker run -it --rm -v $(pwd)/benchmark:/benchmark osrf/ros:humble-desktop
```

In separate docker shells,
```
/benchmark/ros2_ws/pub_start.sh
```

```
/benchmark/ros2_ws/rep_start.sh
```

```
/benchmark/ros2_ws/sub_start.sh
```

This will launch a single repeater node. To launch multiple repeaters, run `rep_start.sh` after incrementing the `NODE_COUNTER` variable in the script. `NODE_COUNTER` starts from 1.

## Feature Limitations

**commaai/cereal** has the fewest features, so we use its "one publisher, multiple subscriber" as the lowest common denominator basis for benchmarking.


