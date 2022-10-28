import sys
import time
import subprocess

def main():
    pub_proc = subprocess.Popen(['python3', 'pub.py'])
    sub_proc = subprocess.Popen(['python3', 'sub.py'])

    time.sleep(10)

    if pub_proc.poll() is None:
        print("pub.py is running")
        pub_proc.terminate()
        pub_proc.join()
    else:
        print("pub.py failed to start")
    
    if sub_proc.poll() is None:
        print("sub.py is running")
        sub_proc.terminate()
        sub_proc.join()
    else:
        print("sub.py failed to start")

    print("Finished")
    

if __name__ == '__main__':
    main()