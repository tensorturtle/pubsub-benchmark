#!/usr/bin/python3

import logging
from multiprocessing import Process
from typing import List
import os
import sys
import sqlite3
from time import sleep

logging.basicConfig(level=logging.DEBUG)

def create_db(
    path: str,
    keys: List[str],
):
    conn = sqlite3.connect(path)
    c = conn.cursor()
    c.execute('CREATE TABLE IF NOT EXISTS test (id INTEGER PRIMARY KEY, {})'.format(','.join(keys)))
    conn.commit()
    conn.close()

def plot_results(db_path, plot_results_path):
    conn = sqlite3.connect(db_path)
    c = conn.cursor()
    c.execute('SELECT * FROM test')
    results = c.fetchall()
    conn.close()
    # plot results with matplotlib
    #TODO




if __name__ == "__main__":
    db_abs_path = os.path.join(os.getcwd(), 'pubsub_results.sqlite')
    keys = ['framework', 'details', 'datetime', 'test_type', 'test_result']
    create_db(db_abs_path, keys)

    # create a new directory called 'result_plots'
    os.makedirs('result_plots', exist_ok=True)

    # run the benchmarks
    want_to_run = {
        'python_builtin': False,
        'cereal': False,
        'ros2': True,
    }

    test_type = 'small'
    # choose from 'small', 'medium', 'large'

    if want_to_run['python_builtin']:
        logging.info("Starting Python builtin benchmark")
        proc_builtin = Process(target=os.system, args=(f'poetry run python3 python_builtins/main.py {db_abs_path}',))
        proc_builtin.start()
        proc_builtin.join()
        logging.info("Finished Python builtin benchmark")
    
    if want_to_run['cereal']:
        logging.info("Starting cereal benchmark")
        proc_cereal_pub = Process(target=os.system, args=(f'./run_cereal.sh publish',))
        proc_cereal_sub = Process(target=os.system, args=(f'./run_cereal.sh subscribe {db_abs_path}',))

        proc_cereal_pub.start()
        proc_cereal_sub.start()

        proc_cereal_pub.join()
        proc_cereal_sub.join()
        logging.info("Finished cereal benchmark")
    
    if want_to_run['ros2']:
        logging.info("Starting ROS2 benchmark")
        # start root publisher and repeaters

        ros2_bench_root = os.path.join(
            os.getcwd(),
            'ros2_bench',
        )

        try:
            pub_proc = Process(target=os.system, args=(' '.join(
                [
                    os.path.join(ros2_bench_root, 'start_docker.sh'),
                    os.path.join(ros2_bench_root, 'benchmark'),
                    os.path.join(ros2_bench_root, 'results'),
                    "pub",
                    str(test_type),
                ]
            ), ))

            rep_procs = []
            for rep_i in range(1, 11):
                logging.info(f"Starting repeater {rep_i}")
                rep_proc = Process(target=os.system, args=(' '.join(
                        [  
                            os.path.join(ros2_bench_root, 'start_docker.sh'),
                            os.path.join(ros2_bench_root, 'benchmark'),
                            os.path.join(ros2_bench_root, 'results'),
                            "rep",
                            str(test_type),
                            str(rep_i),
                        ]
                    ), ))
                rep_procs.append(rep_proc)
                rep_proc.start()

            sub_proc = Process(target=os.system, args=(' '.join(
                [
                    os.path.join(ros2_bench_root, 'start_docker.sh'),
                    os.path.join(ros2_bench_root, 'benchmark'),
                    os.path.join(ros2_bench_root, 'results'),
                    "sub",
                    str(test_type),
                ]
            ), ))
            pub_proc.start()
            sub_proc.start()
            logging.info("Finished ROS2 benchmark")
        
        except KeyboardInterrupt:
            # kill all docker containers from ros image
            os.system('docker kill $(docker ps --filter "ancestor=osrf/ros:humble-desktop" -q)')

            pub_proc.terminate()
            pub_proc.join()
            sub_proc.terminate()
            pub_proc.join()
            for rep_proc in rep_procs:
                rep_proc.terminate()
                rep_proc.join()

            raise KeyboardInterrupt
        
        sub_proc.join()
        logging.warning("Sub proc has been joined")
        for rep_proc in rep_procs:
            rep_proc.join()
        logging.warning("Rep procs have been joined")
        pub_proc.join()
        logging.warning("Pub proc has been joined")

        logging.info("Reading database and creating plots")
        plot_results(db_abs_path, plot_results_path='result_plots')

        logging.info("Finished. See plots in result_plots/")



