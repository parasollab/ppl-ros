import sys

from ppl_interfaces.srv import ReceiveParts
import rclpy
from rclpy.node import Node

class WorkstationNodeClient(Node):
  def __init__(self):
    super().__init__('workstation_node_client')
    self.cli = self.create_client(ReceiveParts, 'receive_parts')
    while not self.cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')
    self.req = ReceiveParts.Request()

  def send_request(self, t, k, ns, ct):
    self.req.time_per_assembly = t
    self.req.num_new_parts = k
    self.req.ns = ns
    self.req.completed_threshold = ct
    self.future = self.cli.call_async(self.req)
    rclpy.spin_until_future_complete(self, self.future)
    return self.future.result()


def main():
  rclpy.init()

  workstation_node_client = WorkstationNodeClient()
  response = workstation_node_client.send_request(float(sys.argv[1]), int(sys.argv[2]), str(sys.argv[3]), float(sys.argv[4]))
  
  workstation_node_client.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()