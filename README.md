# pubsub-benchmark
Baseline performance metrics for various on-device publisher-subscriber implementations, with a focus on mobile (single board computer) use.

## 

## Variables

+ Frameworks
   + [Python Multiprocessing Queue](https://docs.python.org/3/library/multiprocessing.html?highlight=multiprocessing%20queue#multiprocessing.Queue) - Python 3.8.10
   + [Python Multithreading Queue](https://docs.python.org/3/library/queue.html) - Python 3.8.10
   + [ROS2](https://docs.ros.org/) [Humble](https://docs.ros.org/en/rolling/Releases/Release-Humble-Hawksbill.html#humble-hawksbill-humble)
   + [commaai/cereal](https://github.com/commaai/cereal): "cereal is both a messaging spec for robotics systems as well as generic high performance IPC pub sub messaging" - cloned 2022-10-24
   + (Maybe) [ZMQ](https://zeromq.org/) via [PyZMQ](https://pyzmq.readthedocs.io/en/latest/)
+ Message Sizes
   + Small: Single `char`
   + Medium: Dictionary of a handful of scalars/floats
   + Large: RGB image
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

  
## Feature Limitations

**commaai/cereal** has the fewest features, so we use its "one publisher, multiple subscriber" as the lowest common denominator basis for benchmarking.


