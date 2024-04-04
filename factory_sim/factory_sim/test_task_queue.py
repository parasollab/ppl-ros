import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class TQTest(Node):
    def __init__(self):
        super().__init__("tq_test")

        self.tq_pub = self.create_publisher(
            PoseStamped,
            '/robot1/task_queue',
            1000
        )

        # TODO: pub stuff
        #       x, y, z, ox, oy, oz, ow
        # goal = [2.2, -3.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        # goal = [2.1, -1.8, 0.0, 0.0, 0.0, 1.0, 0.0]
        # goal = [0.4, -1.8, 0.0, 0.0, 0.0, 0.0, 1.0]

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = goal[2]
        msg.pose.orientation.x = goal[3]
        msg.pose.orientation.y = goal[4]
        msg.pose.orientation.z = goal[5]
        msg.pose.orientation.w = goal[6]

        self.tq_pub.publish(msg)

        
        

def main(args=None):
    rclpy.init(args=args)

    tq_test_node = TQTest()

    rclpy.spin(tq_test_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


