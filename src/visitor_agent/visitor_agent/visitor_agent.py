import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class Visitor:
    def __init__(self, visitor_id, host_building_id):
        self.visitor_id = visitor_id
        self.host_building_id = host_building_id

class VisitorAgent(Node):
    def __init__(self, visitor_id, host_building_id):
        super().__init__(f'visitor_agent_{visitor_id}')
        self.visitor = Visitor(visitor_id, host_building_id)
        self.ci_agent_pub = self.create_publisher(String, f'/visitor_agent/{self.visitor.visitor_id}/request_escort', 10)
        self.host_building_pub = self.create_publisher(String, f'/visitor_agent/{self.visitor.visitor_id}/host_building_info', 10)
        
        # Subscribe to CI Agent updates (e.g., navigation status)
        self.ci_agent_sub = self.create_subscription(String, f'/ci_agent/{self.visitor.visitor_id}/escort_update', self.handle_escort_update, 10)

        # Let's simulate sending a request for escort to the CI Agent
        self.request_escort_from_ci_agent()

    def request_escort_from_ci_agent(self):
        """Simulate requesting an escort from the CI Agent"""
        self.get_logger().info(f"Visitor {self.visitor.visitor_id} requesting escort to building {self.visitor.host_building_id}")
        message = {
            'visitor_id': self.visitor.visitor_id,
            'host_building_id': self.visitor.host_building_id
        }
        self.ci_agent_pub.publish(String(data=json.dumps(message)))

    def handle_escort_update(self, msg):
        """Handle updates from the CI Agent"""
        update = json.loads(msg.data)
        self.get_logger().info(f"Received escort update: {update}")
        if update['status'] == 'arrived':
            self.get_logger().info(f"Visitor {self.visitor.visitor_id} has arrived at building {self.visitor.host_building_id}")

def main(args=None):
    rclpy.init(args=args)
    # Example visitor_id = 1, and the visitor is heading to building_id = 5
    visitor_agent = VisitorAgent(visitor_id=1, host_building_id=5)
    rclpy.spin(visitor_agent)
    visitor_agent.destroy_node()
    rclpy.shutdown()
