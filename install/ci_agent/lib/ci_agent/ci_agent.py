import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class Visitor:
    def __init__(self, visitor_id, host_building_id):
        self.visitor_id = visitor_id
        self.host_building_id = host_building_id

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
        # Example campus map data (could be a graph structure)
        return {
            '1': {'connected_buildings': ['2', '3']},
            '2': {'connected_buildings': ['1', '3', '4']},
            '3': {'connected_buildings': ['1', '2', '4']},
            '4': {'connected_buildings': ['2', '3']}
        }

    def escort_visitor(self, visitor):
        self.current_visitor = visitor
        self.get_logger().info(f"Escorting visitor {visitor.visitor_id} to building {visitor.host_building_id}.")
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
        elif response['status'] == 'denied':
            self.get_logger().warn(f"Navigation denied: {response['reason']}. Trying to reroute.")
            self.handle_denied_navigation(response['building_id'])

    def handle_denied_navigation(self, building_id):
        # Implement logic to find an alternate route or another BI Agent
        self.get_logger().info(f"Attempting to find alternate routes for building {building_id}.")
        # Placeholder: Currently no alternate logic implemented

    def guide_to_host(self, path):
        # Logic to guide visitor to host using the path provided by BI Agent
        self.get_logger().info(f"Guiding visitor {self.current_visitor.visitor_id} using path: {path}")

    def handle_oos_notification(self, msg):
        notification = json.loads(msg.data)
        building_id = notification['building_id']
        duration = notification['duration']
        self.get_logger().info(f"BI Agent {building_id} is out of service for {duration} seconds.")
        
        if self.current_visitor and building_id == self.current_visitor.host_building_id:
            self.get_logger().warn(f"Visitor {self.current_visitor.visitor_id} cannot reach host building {building_id}. Retrying later...")
            # Retry navigation after the given duration
            self.timer = self.create_timer(duration, self.retry_navigation)

    def retry_navigation(self):
        self.get_logger().info(f"Retrying navigation for visitor {self.current_visitor.visitor_id}.")
        building_id = self.current_visitor.host_building_id
        self.request_navigation_path(building_id)
        self.timer.cancel()  # Stop the retry timer after one attempt

    def stop_escort(self):
        self.get_logger().info(f"Stopping escort for visitor {self.current_visitor.visitor_id}.")
        self.current_visitor = None

def main(args=None):
    rclpy.init(args=args)
    ci_agent = CIAgent(agent_id=1)  # Example agent_id = 1
    rclpy.spin(ci_agent)
    ci_agent.destroy_node()
    rclpy.shutdown()
