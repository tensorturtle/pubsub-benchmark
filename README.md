# pubsub-benchmark
Baseline performance metrics for various on-device publisher-subscriber implementations, with a focus on mobile (single board computer) use.

Python 3.8.10 & Ubuntu 20.04 or 18.04 used throughout.

## Variables

+ Frameworks
   + [Python Multiprocessing Queue](https://docs.python.org/3/library/multiprocessing.html?highlight=multiprocessing%20queue#multiprocessing.Queue)
   + [Python Multithreading Queue](https://docs.python.org/3/library/queue.html)
   + [ROS2](https://docs.ros.org/) [Humble](https://docs.ros.org/en/rolling/Releases/Release-Humble-Hawksbill.html#humble-hawksbill-humble)
   + [commaai/cereal](https://github.com/commaai/cereal) - cloned 2022-10-24
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

## Run Benchmarks

```
git clone https://github.com/tensorturtle/pubsub-benchmark.git
```

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

[**Install Docker Engine on Ubuntu**](https://docs.docker.com/engine/install/ubuntu/)

Pull pre-made docker image:

```
cd cereal_bench
sudo docker run -it --rm -v $(pwd):/root tensorturtle/cereal
```

To build the docker image yourself, use the [official cereal Dockerfile](https://github.com/commaai/cereal/blob/master/Dockerfile)

Then, in the docker shell,

```
```


## Feature Limitations

**commaai/cereal** has the fewest features, so we use its "one publisher, multiple subscriber" as the lowest common denominator basis for benchmarking.


