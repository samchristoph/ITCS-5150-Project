import subprocess
import os

# import mapper
# import aStar
# import obstacleDetection
# import planner
# import observation_model
# import motion_model_SER

CWD = os.path.dirname(os.path.realpath(__file__))

def main(args=None):
    process1 = subprocess.Popen(["python3", f"{CWD}/mapper.py"])
    # process2 = subprocess.Popen(["python3", f"{CWD}/aStar.py"])
    process3 = subprocess.Popen(["python3", f"{CWD}/obstacleDetection.py"])
    process4 = subprocess.Popen(["python3", f"{CWD}/planner.py"])
    process5 = subprocess.Popen(["python3", f"{CWD}/observation_model.py"])
    process6 = subprocess.Popen(["python3", f"{CWD}/motion_model_SER.py"])

    process1.wait()
    # process2.wait()
    process3.wait()
    process4.wait()
    process5.wait()
    process6.wait()

if __name__ == '__main__':
    main()