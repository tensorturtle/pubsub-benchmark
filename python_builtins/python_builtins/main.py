import asyncio

from asyncio_bench import run_benchmark
from mproc_bench import run_benchmark_mp

if __name__ == "__main__":
    asyncio.run(run_benchmark())
    run_benchmark_mp()



