import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import Position
from crazyflie_py import Crazyswarm
import numpy as np


class CrazyflieDirectGoto(Node):
    def __init__(self):
        self.swarm = Crazyswarm()  # Crazyswarm 초기화
        self.timeHelper = self.swarm.timeHelper
        self.cf = self.swarm.allcfs.crazyflies[0]

        super().__init__('crazyflie_direct_goto')

        self.subscription = self.create_subscription(
            Position,
            'cf2/cmd_position',
            self.cmd_callback,
            10)

        self.cmd_position = np.array([0.0, 0.0, 0.0])  # 초기 위치
        self.cmd_yaw = 0.0  # 초기 yaw (radian)
        self.cmd_received = False

        self.timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def cmd_callback(self, msg):
        self.cmd_position = np.array([msg.x, msg.y, msg.z])
        self.cmd_yaw = np.deg2rad(msg.yaw)  # yaw을 radian으로 변환
        self.cmd_received = True

    def timer_callback(self):
        if self.cmd_received:
            # goTo()를 사용하여 절대 좌표계 기준으로 yaw 반영
            self.cf.goTo(self.cmd_position, self.cmd_yaw, 0.008)  # goTo()는 절대 좌표계에서 yaw를 반영

    def shutdown(self):
        self.cf.land(targetHeight=0.04, duration=2.5)
        self.timeHelper.sleep(3.0)


def main(args=None):
    node = CrazyflieDirectGoto()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
