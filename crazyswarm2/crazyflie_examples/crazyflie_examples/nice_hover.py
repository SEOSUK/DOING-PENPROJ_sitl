import numpy as np
from crazyflie_py import *
import time

def main():
    Z = 1.0                          # 비행 고도 (m)
    MOVE_DISTANCE_X = -0.2           # X 방향 이동 거리 (m)
    Y_AMPLITUDE = 0.5               # Y 방향 이동 거리 (m)
    MOVE_DURATION = 3.              # 각 구간 이동 시간 (초)

    YAW_ANGLE = np.deg2rad(20)       # 15도 yaw 회전 (라디안 단위)

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # 드론 이륙
    allcfs.takeoff(targetHeight=Z, duration=1.0 + Z)
    timeHelper.sleep(1.5 + Z)

    # 초기 위치에서 호버링
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.goTo(pos, yaw=YAW_ANGLE, duration=1.0)
    timeHelper.sleep(1.0)

    print("Moving to (0.28, 0.0)...")
    for cf in allcfs.crazyflies:
        cf.goTo(np.array([MOVE_DISTANCE_X, 0.0, Z]), yaw= - YAW_ANGLE, duration=MOVE_DURATION)
    timeHelper.sleep(MOVE_DURATION + 0.5)

    print("Returning to (0.28, 0.0)...")
    for cf in allcfs.crazyflies:
        cf.goTo(np.array([MOVE_DISTANCE_X, 0.0, Z]),  yaw= - 3.1, duration=MOVE_DURATION)
    timeHelper.sleep(MOVE_DURATION + 0.5)

    print("Returning to (0.28, 0.0)...")
    for cf in allcfs.crazyflies:
        cf.goTo(np.array([MOVE_DISTANCE_X, 0.0, Z]),  yaw= 0, duration=MOVE_DURATION)
    timeHelper.sleep(MOVE_DURATION + 0.5)


    print("Returning to (0.28, 0.0)...")
    for cf in allcfs.crazyflies:
        cf.goTo(np.array([MOVE_DISTANCE_X, 0.0, Z]),  yaw= - 3.1, duration=MOVE_DURATION)
    timeHelper.sleep(MOVE_DURATION + 0.5)

    print("Returning to (0.28, 0.0)...")
    for cf in allcfs.crazyflies:
        cf.goTo(np.array([MOVE_DISTANCE_X, 0.0, Z]),  yaw= 0, duration=MOVE_DURATION)
    timeHelper.sleep(MOVE_DURATION + 0.5)    


    print("Landing...")
    allcfs.land(targetHeight=0.02, duration=1.0 + Z)
    timeHelper.sleep(1.0 + Z)

if __name__ == "__main__":
    main()
