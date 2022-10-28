BENCHMARK_ABS_PATH=$1
RESULTS_ABS_PATH=$2
DB_ABS_PATH=$3
NODE_TYPE=$4
REP_COUNTER=$5

BENCHMARK_ABS_PATH="/home/jasonsohn/Repos/pubsub-benchmark/ros2_bench/benchmark"
RESULTS_ABS_PATH="/home/jasonsohn/Repos/pubsub-benchmark/results"

# if benchmark path is not set, use default
# assuming user cloned the repo in home directory
if [ -z "$BENCHMARK_ABS_PATH" ]; then
	BENCHMARK_ABS_PATH="/home/$(whoami)/pubsub-benchmark/ros2_bench/benchmark"
fi

# if results path is not set, use default
# assuming user cloned the repo in home directory
if [ -z "$RESULTS_ABS_PATH" ]; then
	RESULTS_ABS_PATH="/home/$(whoami)/pubsub-benchmark/results"
fi

# if node type is not set, use default
if [ -z "$NODE_TYPE" ]; then
	NODE_TYPE="pub"
fi



#sudo docker run -it --rm -e "DB_PATH"=$DB_PATH -v $(pwd)/benchmark:/benchmark osrf/ros:humble-desktop /benchmark/echo_path.sh

sudo docker run -it --rm \
	-v $BENCHMARK_ABS_PATH:/benchmark \
	-v $RESULTS_ABS_PATH:/results \
	osrf/ros:humble-desktop \
	#/benchmark/ros2_ws/$NODE_TYPE\_start.sh
