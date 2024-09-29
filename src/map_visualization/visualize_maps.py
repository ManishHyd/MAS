# map_visualization/visualize_maps.py
import matplotlib.pyplot as plt
import networkx as nx

def show_campus_map(campus_map, visitor_location=None):
    """
    Visualizes the campus map with connections between buildings.
    Optionally highlights the current location of the visitor.
    """
    campus_graph = nx.Graph()
    
    # Adding nodes and edges from campus_map data
    for building_id, data in campus_map.items():
        campus_graph.add_node(building_id)
        for connected_building in data['connected_buildings']:
            campus_graph.add_edge(building_id, connected_building)
    
    pos = nx.spring_layout(campus_graph)
    nx.draw(campus_graph, pos, with_labels=True, node_size=2000, node_color='lightgreen')
    
    # Highlight visitor location if provided
    if visitor_location:
        nx.draw_networkx_nodes(campus_graph, pos, nodelist=[visitor_location], node_color='red')
        plt.annotate("Visitor", pos[visitor_location], textcoords="offset points", xytext=(10, 10), ha='center')
    
    plt.title("Campus Map")
    plt.show()

def show_building_map(building_map, visitor_location=None):
    """
    Visualizes the building map with rooms and connections between them.
    Optionally highlights the current location of the visitor.
    """
    pos = nx.spring_layout(building_map)
    nx.draw(building_map, pos, with_labels=True, node_size=3000, node_color='lightblue')
    
    # Highlight visitor location if provided
    if visitor_location:
        nx.draw_networkx_nodes(building_map, pos, nodelist=[visitor_location], node_color='red')
        plt.annotate("Visitor", pos[visitor_location], textcoords="offset points", xytext=(10, 10), ha='center')
    
    plt.title("Building Map")
    plt.show()
