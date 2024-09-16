import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class CIAgent(Node):
    def __init__(self, agent_id):
        super().__init__(f'ci_agent_{agent_id}')
        self.agent_id = agent_id
        self.current_visitor = None
        self.campus_map = self.load_campus_map()
        
        # ROS Publishers and Subscribers
        self.navigation_request_pub = self.create_publisher(String, f'/ci_agent/{self.agent_id}/navigation_request', 10)
        self.navigation_response_sub = self.create_subscription(String, f'/ci_agent/{self.agent_id}/navigation_response', self.receive_navigation_response, 10)
        self.oos_notification_sub = self.create_subscription(String, f'/ci_agent/{self.agent_id}/oos_notification', self.handle_oos_notification, 10)

    def load_campus_map(self):
        # Load campus map data (could be a graph structure)
        return {}  # Placeholder

    def escort_visitor(self, visitor):
        self.current_visitor = visitor
        building_id = self.determine_building(visitor)
        self.request_navigation_path(building_id)

    def determine_building(self, visitor):
        # Logic to determine the correct building for the visitor
        return visitor.host_building_id

    def request_navigation_path(self, building_id):
        request_message = self.create_navigation_request(building_id)
        self.navigation_request_pub.publish(String(data=json.dumps(request_message)))

    def create_navigation_request(self, building_id):
        return {
            'type': 'navigation_request',
            'ci_agent_id': self.agent_id,
            'building_id': building_id,
            'visitor_id': self.current_visitor.visitor_id
        }

    def receive_navigation_response(self, msg):
        response = json.loads(msg.data)
        if response['status'] == 'success':
            self.guide_to_host(response['path'])
        else:
            self.get_logger().info(f"Navigation denied: {response['reason']}")

    def guide_to_host(self, path):
        # Logic to guide visitor to host using the path provided by BI Agent
        self.get_logger().info(f"Guiding visitor {self.current_visitor.visitor_id} using path: {path}")

    def handle_oos_notification(self, msg):
        notification = json.loads(msg.data)
        self.get_logger().info(f"BI Agent {notification['building_id']} is out of service for {notification['duration']} seconds.")
        # Logic to handle OOS situation (e.g., wait or reroute)

def main(args=None):
    rclpy.init(args=args)
    ci_agent = CIAgent(agent_id=1)  # Example agent_id = 1
    rclpy.spin(ci_agent)
    ci_agent.destroy_node()
    rclpy.shutdown()
