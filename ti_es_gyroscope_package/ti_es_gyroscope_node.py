import rclpy
from rclpy.node import Node


class GyroscopeNode(Node):
    def __init__(self):
        super().__init__('gyroscope_topic')

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        print(f'F strings zijn super chill')


def main(args=None):
    rclpy.init(args=args)
    gn = GyroscopeNode()

    print('Hi from ti_es_gyroscope_package.')

    rclpy.spin(gn)



if __name__ == '__main__':
    main()
