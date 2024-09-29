import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import networkx as nx
import matplotlib.pyplot as plt

class BIAgent(Node):
    def __init__(self, building_id):
        super().__init__(f'bi_agent_{building_id}')
        self.building_id = building_id
        self.building_map = self.load_building_map()
        self.oos_state = False
        
        # ROS Publishers and Subscribers
        self.navigation_request_sub = self.create_subscription(
            String, 
            f'/bi_agent/{self.building_id}/navigation_request', 
            self.handle_navigation_request, 
            10
        )
        self.navigation_response_pub = self.create_publisher(
            String, 
            f'/bi_agent/{self.building_id}/navigation_response', 
            10
        )
        self.oos_notification_pub = self.create_publisher(
            String, 
            f'/bi_agent/{self.building_id}/oos_notification', 
            10
        )
        self.visualize_building_map()  # Optional: Display the building map

    def load_building_map(self):
        building_map = nx.Graph()
        
        # Adding nodes (locations in the building)
        building_map.add_node("Entrance")
        building_map.add_node("Hallway")
        building_map.add_node("Host Office")
        building_map.add_node("Room1")
        building_map.add_node("Room2")
        
        # Adding edges (paths between locations)
        building_map.add_edge("Entrance", "Hallway", weight=1)
        building_map.add_edge("Hallway", "Host Office", weight=1)
        building_map.add_edge("Hallway", "Room1", weight=1)
        building_map.add_edge("Hallway", "Room2", weight=1)

        return building_map

    def visualize_building_map(self):
        pos = nx.spring_layout(self.building_map)
        nx.draw(self.building_map, pos, with_labels=True, node_size=3000, node_color='lightblue')
        plt.title(f"Building Map for BI Agent {self.building_id}")
        plt.show()

    def handle_navigation_request(self, msg):
        request = json.loads(msg.data)
        visitor_id = request['visitor_id']
        ci_agent_id = request['ci_agent_id']

        if self.oos_state or not self.check_authorization(visitor_id):
            self.send_denial_message(ci_agent_id)
        else:
            path = self.find_path(visitor_id, request['building_id'])
            self.send_navigation_response(ci_agent_id, path)

    def check_authorization(self, visitor_id):
        authorized_visitors = ["visitor_1", "visitor_2", "visitor_3"]  # Example authorized visitors
        return visitor_id in authorized_visitors

    def find_path(self, visitor_id, host_id):
        try:
            path = nx.shortest_path(self.building_map, source="Entrance", target="Host Office", weight='weight')
            self.get_logger().info(f"Path found for visitor {visitor_id}: {path}")
            return path
        except nx.NetworkXNoPath:
            self.get_logger().error(f"No path found for visitor {visitor_id} to {host_id}.")
            return []

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
        self.get_logger().info(f"BI Agent {self.building_id} is out of service for {duration} seconds.")
        time.sleep(duration)  # This is blocking, consider using a non-blocking approach
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
