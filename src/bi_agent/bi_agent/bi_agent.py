import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class BIAgent(Node):
    def __init__(self, building_id):
        super().__init__(f'bi_agent_{building_id}')
        self.building_id = building_id
        self.building_map = self.load_building_map()
        self.oos_state = False
        
        # ROS Publishers and Subscribers
        self.navigation_request_sub = self.create_subscription(String, f'/bi_agent/{self.building_id}/navigation_request', self.handle_navigation_request, 10)
        self.navigation_response_pub = self.create_publisher(String, f'/bi_agent/{self.building_id}/navigation_response', 10)
        self.oos_notification_pub = self.create_publisher(String, f'/bi_agent/{self.building_id}/oos_notification', 10)

    def load_building_map(self):
        # Load building map data (could be a graph structure)
        return {}  # Placeholder

    def handle_navigation_request(self, msg):
        request = json.loads(msg.data)
        if self.oos_state or not self.check_authorization(request['visitor_id']):
            self.send_denial_message(request['ci_agent_id'])
        else:
            path = self.find_path(request['visitor_id'], request['building_id'])
            self.send_navigation_response(request['ci_agent_id'], path)

    def check_authorization(self, visitor_id):
        # Logic to check visitor's authorization
        return True  # Placeholder

    def find_path(self, visitor_id, host_id):
        # Logic to find path within the building from entrance to host's location
        return ["Entrance", "Hallway", "Host Office"]  # Placeholder

    def send_navigation_response(self, ci_agent_id, path):
        response_message = {
            'type': 'navigation_response',
            'status': 'success',
            'ci_agent_id': ci_agent_id,
            'building_id': self.building_id,
            'path': path
        }
        self.navigation_response_pub.publish(String(data=json.dumps(response_message)))

    def send_denial_message(self, ci_agent_id):
        denial_message = {
            'type': 'navigation_response',
            'status': 'denied',
            'ci_agent_id': ci_agent_id,
            'building_id': self.building_id,
            'reason': 'Unauthorized or BI Agent Out of Service'
        }
        self.navigation_response_pub.publish(String(data=json.dumps(denial_message)))

    def go_out_of_service(self, duration):
        self.oos_state = True
        oos_message = self.create_oos_notification(duration)
        self.oos_notification_pub.publish(String(data=json.dumps(oos_message)))
        time.sleep(duration)
        self.resume_service()

    def create_oos_notification(self, duration):
        return {
            'type': 'oos_notification',
            'building_id': self.building_id,
            'duration': duration
        }

    def resume_service(self):
        self.oos_state = False
        self.get_logger().info(f"BI Agent {self.building_id} is back in service.")

def main(args=None):
    rclpy.init(args=args)
    bi_agent = BIAgent(building_id=1)  # Example building_id = 1
    rclpy.spin(bi_agent)
    bi_agent.destroy_node()
    rclpy.shutdown()
