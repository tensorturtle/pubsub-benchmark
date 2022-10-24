import multiprocessing
from multiprocessing import Process, Queue

import statistics
import time
import sys
sys.path.append('../../') # to access common_data

from common_data import LARGE_IMAGE, MEDIUM_DICT, SMALL_CHAR

# Publisher using multiprocessing
def root_publisher_mp(
    t1_queue: multiprocessing.Queue,
    t2_head_queue: multiprocessing.Queue,
    num_sends: int,
    root_sleep_time: float,
    data,
):
    for _ in range(num_sends):
        time.sleep(root_sleep_time)
        t1_queue.put(data)
        t2_head_queue.put(data)

    t1_queue.put(None)
    t2_head_queue.put(None)

# Last subscriber using multiprocessing
def last_subscriber_mp(
    t1_queue: multiprocessing.Queue,
    t2_tail_queue: multiprocessing.Queue,
):
    # compare time difference between item received from t2_tail_queue vs t1_queue
    # t1_queue should be faster
    latencies = []
    while True:
        item1 = t1_queue.get()
        t1 = time.perf_counter()
        item2 = t2_tail_queue.get()
        t2 = time.perf_counter()
        if item1 is None or item2 is None:
            break
        latencies.append(t2-t1)
        print(f"last_subscriber: Time taken for data to flow through once = {s_to_ms(t2-t1)}ms")

    print(f"Mean: {s_to_ms(statistics.mean(latencies))}ms")
    print(f"Max: {s_to_ms(max(latencies))}ms")
    print(f"Min: {s_to_ms(min(latencies))}ms")
    print(f"Standard Deviation: {s_to_ms(statistics.stdev(latencies))}ms")
    print(f"First quartile: {s_to_ms(statistics.quantiles(latencies, n=4)[0])}ms")
    print(f"Third quartile: {s_to_ms(statistics.quantiles(latencies, n=4)[2])}ms")

def s_to_ms(x):
    return round(x * 1000, 3)

# Subscriber and publisher using multiprocessing
def subpub_mp(
    listen_to: multiprocessing.Queue,
    publish_to: multiprocessing.Queue,
    node_sleep_time: float,
):
    while True:
        item = listen_to.get()
        if item is None:
            publish_to.put(None)
            break
        time.sleep(node_sleep_time)
        publish_to.put(item)

def run_benchmark_mp(
    num_subpubs: int = 10,
    root_sleep_time: float = 1.0,
    node_sleep_time: float = 0.01,
    num_sends: int = 10,
    data = LARGE_IMAGE,

):
    # Create queues
    t1_queue = multiprocessing.Queue()
    t2_queues = [multiprocessing.Queue() for _ in range(num_subpubs)]
    t2_head_queue = t2_queues[0]
    t2_tail_queue = t2_queues[-1]

    # check that sleeps make sense
    if root_sleep_time < (node_sleep_time * num_subpubs):
        raise ValueError("root_sleep_time must be greater than node_sleep_time * num_subpubs + some buffer. \
            Otherwise, the t1 message will get to the last subscriber before the last t2 message.")

    # start processes
    root_publisher = Process(target=root_publisher_mp, args=(t1_queue, t2_head_queue, num_sends, root_sleep_time, data))
    last_subscriber = Process(target=last_subscriber_mp, args=(t1_queue, t2_tail_queue))
    subpubs = [Process(target=subpub_mp, args=(t2_queues[i], t2_queues[i+1], node_sleep_time)) for i in range(num_subpubs-1)]

    root_publisher.start()
    last_subscriber.start()
    for subpub in subpubs:
        subpub.start()

    # join processes
    root_publisher.join()
    last_subscriber.join()
    for subpub in subpubs:
        subpub.join()

    # print ideal time:
    ideal_time =  s_to_ms(node_sleep_time * num_subpubs)
    print(f"Ideal time: {ideal_time}ms")


    #print(results) # results gathers the returned value from the last task
    # in streaming tasks like this, it is usually None

if __name__ == "__main__":
    run_benchmark_mp()

