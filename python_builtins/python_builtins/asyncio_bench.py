import asyncio
import statistics
import time
import sys
sys.path.append('../../') # to access common_data

from common_data import LARGE_IMAGE, MEDIUM_DICT, SMALL_CHAR

def s_to_ms(x):
    return round(x * 1000, 3)

async def root_publisher(
    t1_queue: asyncio.Queue,
    t2_head_queue: asyncio.Queue,
    num_sends: int,
    root_sleep_time: float,
    data,
):
    for _ in range(num_sends):
        await asyncio.sleep(root_sleep_time)
        await t1_queue.put(data)
        await t2_head_queue.put(data)

    await t1_queue.put(None)
    await t2_head_queue.put(None)


async def subpub(
    listen_to: asyncio.Queue,
    publish_to: asyncio.Queue,
    node_sleep_time: float,
):
    while True:
        item = await listen_to.get()
        if item is None:
            await publish_to.put(None)
            break
        await asyncio.sleep(node_sleep_time)
        await publish_to.put(item)

async def last_subscriber(
    t1_queue: asyncio.Queue,
    t2_tail_queue: asyncio.Queue,
):
    # compare time difference between item received from t2_tail_queue vs t1_queue
    # t1_queue should be faster
    latencies = []
    while True:
        item1 = await t1_queue.get()
        t1 = time.perf_counter()
        item2 = await t2_tail_queue.get()
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

async def run_benchmark(
    num_subpubs: int = 100,
    root_sleep_time: float = 0.1,
    node_sleep_time: float = 0.0,
    num_sends: int = 100,

):
    t1_queue = asyncio.Queue(maxsize=1)
    t2_queues = [asyncio.Queue(maxsize=1) for _ in range(num_subpubs+1)]
    t2_head_queue = t2_queues[0]
    t2_tail_queue = t2_queues[-1]

    # check that sleeps make sense
    if root_sleep_time < (node_sleep_time * num_subpubs):
        raise ValueError("root_sleep_time must be greater than node_sleep_time * num_subpubs + some buffer. \
            Otherwise, the t1 message will get to the last subscriber before the last t2 message.")

    results = await asyncio.gather(
        asyncio.create_task(
            root_publisher(
                t1_queue,
                t2_head_queue,
                num_sends=num_sends,
                root_sleep_time=root_sleep_time,
                data=LARGE_IMAGE,
                )
        ),
        asyncio.create_task(
            last_subscriber(t1_queue, t2_tail_queue)
        ),
        *[
            asyncio.create_task(
                subpub(t2_queues[i], t2_queues[i+1],
                node_sleep_time=node_sleep_time)
            )
            for i in range(num_subpubs)
        ],
        return_exceptions=False,
    )

    # print ideal time
    ideal_time = node_sleep_time * num_subpubs
    print(f"Ideal_time: {s_to_ms(ideal_time)}ms")


    #print(results) # results gathers the returned value from the last task
    # in streaming tasks like this, it is usually None

if __name__ == "__main__":
    asyncio.run(run_benchmark())

