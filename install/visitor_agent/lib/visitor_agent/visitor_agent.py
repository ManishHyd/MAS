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
        self.current_status = "waiting_for_escort"

        # Publishers and Subscribers
        self.ci_agent_pub = self.create_publisher(String, f'/visitor_agent/{self.visitor.visitor_id}/request_escort', 10)
        self.host_building_pub = self.create_publisher(String, f'/visitor_agent/{self.visitor.visitor_id}/host_building_info', 10)
        self.ci_agent_sub = self.create_subscription(String, f'/visitor_agent/{self.visitor.visitor_id}/escort_update', self.handle_escort_update, 10)
        
        # Request escort from CI Agent
        self.request_escort_from_ci_agent()

    def request_escort_from_ci_agent(self):
        """Request escort from the CI Agent"""
        if self.current_status == "waiting_for_escort":
            self.get_logger().info(f"Visitor {self.visitor.visitor_id} requesting escort to building {self.visitor.host_building_id}")
            message = {
                'visitor_id': self.visitor.visitor_id,
                'host_building_id': self.visitor.host_building_id
            }
            self.ci_agent_pub.publish(String(data=json.dumps(message)))
            self.current_status = "escort_requested"
            self.start_response_timeout()

    def start_response_timeout(self):
        """Start a timer to wait for a response from CI Agent"""
        self.timer = self.create_timer(10.0, self.handle_timeout)

    def handle_timeout(self):
        """Handle timeout if no response received from CI Agent"""
        if self.current_status == "escort_requested":
            self.get_logger().warn(f"No response from CI Agent for visitor {self.visitor.visitor_id}. Retrying...")
            self.request_escort_from_ci_agent()
            self.timer.cancel()

    def handle_escort_update(self, msg):
        """Handle updates from the CI Agent"""
        update = json.loads(msg.data)
        self.get_logger().info(f"Received escort update: {update}")
        if update['status'] == 'en_route':
            self.current_status = "being_escorted"
            self.get_logger().info(f"CI Agent is en route to escort visitor {self.visitor.visitor_id}")
        elif update['status'] == 'arrived':
            self.current_status = "arrived"
            self.get_logger().info(f"Visitor {self.visitor.visitor_id} has arrived at building {self.visitor.host_building_id}")
            self.notify_host_arrival()
        elif update['status'] == 'denied':
            self.current_status = "escort_denied"
            self.get_logger().error(f"Escort request denied: {update['reason']}")
            # Additional handling (e.g., re-request after some time or notify user)

    def notify_host_arrival(self):
        """Notify that the visitor has arrived at the host building"""
        message = {
            'visitor_id': self.visitor.visitor_id,
            'host_building_id': self.visitor.host_building_id,
            'status': 'arrived'
        }
        self.host_building_pub.publish(String(data=json.dumps(message)))
        self.get_logger().info(f"Notified host of arrival for visitor {self.visitor.visitor_id}")

def main(args=None):
    rclpy.init(args=args)
    # Example visitor_id = 1, and the visitor is heading to building_id = 5
    visitor_agent = VisitorAgent(visitor_id=1, host_building_id=5)
    rclpy.spin(visitor_agent)
    visitor_agent.destroy_node()
    rclpy.shutdown()
