import heapq
import tkinter as tk
from tkinter import messagebox


class Graph:
    def __init__(self, edges_list=None):
        self.nodes = []
        self.edges = []
        if edges_list is not None:
            self.add_edges(edges_list)

    def add_node(self, v):
        self.nodes.append(v)

    def add_edge(self, ab, w, layover_time):
        a, b = ab
        if a not in self.nodes:
            self.nodes.append(a)
        if b not in self.nodes:
            self.nodes.append(b)
        my_new_edge = Edge(ab, w, layover_time)
        self.edges.append(my_new_edge)

    def add_edges(self, e_list):
        for ab, w, layover_time in e_list:
            self.add_edge(ab, w, layover_time)

    def __str__(self):
        return f"{self.nodes}\n{self.edges}"

    def dijkstra(self, start_node):
        # Create a priority queue to store (distance, node) tuples
        queue = []
        heapq.heappush(queue, (0, start_node))

        # Create a dictionary to store the shortest path to each node
        # makes a map of every node (key) and distance to that node from starting point (value)
        distances = {node: float('infinity') for node in self.nodes}
        distances[start_node] = 0

        # Create a dictionary to store the best previous node to each node
        # makes a map of nodes (key) and best previous node (value)
        previous_nodes = {node: None for node in self.nodes}

        while queue:
            # node we are currently on
            current_distance, current_node = heapq.heappop(queue)

            # If we found a shorter path before, skip processing this node
            if current_distance > distances[current_node]:
                continue

            # Look at each neighbor of the current node
            for edge in self.edges:
                if edge._from == current_node:
                    distance = current_distance + edge._weight + edge._layover_time

                    # If a shorter path is found
                    if distance < distances[edge._to]:
                        distances[edge._to] = distance
                        previous_nodes[edge._to] = current_node
                        heapq.heappush(queue, (distance, edge._to))

        return distances, previous_nodes

    def get_edge_distance(self, from_node, to_node):
        for edge in self.edges:
            if edge._from == from_node and edge._to == to_node:
                return edge._weight
        return None

    def get_layover(self, from_node, to_node):
        for edge in self.edges:
            if edge._from == from_node and edge._to == to_node:
                return edge._layover_time
        return None

    def get_shortest_path(self, previous_nodes, start_node, end_node):
        shortest_path = []
        current_node = end_node
        while current_node is not None:
            shortest_path.append(current_node)
            current_node = previous_nodes[current_node]
        shortest_path.reverse()
        if shortest_path[0] != start_node:
            return None, None
        total_distance = 0
        distances_with_path = []
        for i in range(len(shortest_path) - 2):
            from_node = shortest_path[i]
            to_node = shortest_path[i + 1]
            edge_distance = self.get_edge_distance(from_node, to_node)
            edge_layover = self.get_layover(from_node, to_node)
            if edge_distance is not None and edge_layover is not None:
                distance_hours = int(edge_distance)
                distance_minutes = (edge_distance * 60) % 60
                layover_hours = int(edge_layover)
                layover_minutes = (edge_layover * 60) % 60
                distances_with_path.append(
                    f"{from_node} →({distance_hours}hr{int(distance_minutes)}m) {to_node} ({layover_hours}hr{int(layover_minutes)}m layover)")
                total_distance += edge_distance
                total_distance += edge_layover
        from_node = shortest_path[len(shortest_path) - 2]
        to_node = shortest_path[len(shortest_path) - 1]
        edge_distance = self.get_edge_distance(from_node, to_node)
        if edge_distance is not None:
            distance_hours = int(edge_distance)
            distance_minutes = (edge_distance * 60) % 60
            distances_with_path.append(f"{from_node} →({distance_hours}hr{int(distance_minutes)}m) {to_node}")
            total_distance += edge_distance
        else:
            distances_with_path.append(f"No flight path between {from_node} and {to_node}")

        total_hours = int(total_distance)
        total_minutes = (total_distance * 60) % 60
        distances_with_path.append(f"Total time: {total_hours}hr{int(total_minutes)}m")
        return shortest_path, distances_with_path


class Edge:
    def __init__(self, ab, w, layover_time):
        a, b = ab
        self._from = a
        self._to = b
        self._weight = w
        self._layover_time = layover_time

    def __repr__(self):
        return f"from {self._from} to {self._to} weighted {self._weight} with layover {self._layover_time}"

    def get_start(self):
        return self._from

    def get_destination(self):
        return self._to

    def get_names(self):
        return self._from, self._to


edgelist = [(('Anchorage', 'Honolulu'), 6.1, 2),
            (('Anchorage', 'Juneau'), 1.6, 2),
            (('Anchorage', 'San Francisco'), 4.5, 2),
            (('Anchorage', 'Seattle'), 3.4, 2),
            (('Atlanta', 'Chicago'), 2.1, 2),
            (('Atlanta', 'Houston'), 2.2, 2),
            (('Atlanta', 'Miami'), 1.9, 2),
            (('Atlanta', 'Newark'), 2.3, 2),
            (('Atlanta', 'San Diego'), 4.4, 2),
            (('Austin', 'Houston'), 1.0, 2),
            (('Boise', 'Portland'), 1.3, 2),
            (('Boston', 'Denver'), 4.7, 2),
            (('Boston', 'Newark'), 1.5, 2),
            (('Boston', 'Reykjavik'), 5.3, 2),
            (('Boston', 'Washington, D.C'), 1.8, 2),
            (('Chicago', 'Atlanta'), 1.9, 2),
            (('Chicago', 'Denver'), 2.7, 2),
            (('Chicago', 'Honolulu'), 9.0, 2),
            (('Chicago', 'Miami'), 3.2, 2),
            (('Chicago', 'Newark'), 2.3, 2),
            (('Chicago', 'San Diego'), 4.3, 2),
            (('Chicago', 'San Francisco'), 4.7, 2),
            (('Chicago', 'Seattle'), 4.6, 2),
            (('Chicago', 'Washington, D.C'), 1.9, 2),
            (('Denver', 'Atlanta'), 3.0, 2),
            (('Denver', 'Boston'), 4.0, 2),
            (('Denver', 'Chicago'), 2.5, 2),
            (('Denver', 'Honolulu'), 7.2, 2),
            (('Denver', 'Houston'), 2.4, 2),
            (('Denver', 'Newark'), 3.8, 2),
            (('Denver', 'Sacramento'), 2.5, 2),
            (('Denver', 'San Diego'), 2.3, 2),
            (('Denver', 'San Francisco'), 2.7, 2),
            (('Denver', 'Seattle'), 2.9, 2),
            (('Hilo', 'Honolulu'), 0.9, 2),
            (('Honolulu', 'Anchorage'), 6.1, 2),
            (('Honolulu', 'Chicago'), 8.2, 2),
            (('Honolulu', 'Denver'), 6.8, 2),
            (('Honolulu', 'Hilo'), 0.9, 2),
            (('Honolulu', 'San Diego'), 5.5, 2),
            (('Honolulu', 'San Francisco'), 5.1, 2),
            (('Honolulu', 'Sydney'), 10.5, 2),
            (('Houston', 'Atlanta'), 2.1, 2),
            (('Houston', 'Austin'), 1.0, 2),
            (('Houston', 'Denver'), 2.5, 2),
            (('Houston', 'San Diego'), 3.3, 2),
            (('Juneau', 'Anchorage'), 1.8, 2),
            (('London', 'Newark'), 8.0, 2),
            (('Miami', 'Atlanta'), 2.0, 2),
            (('Miami', 'Newark'), 3.0, 2),
            (('Miami', 'Chicago'), 3.3, 2),
            (('Newark', 'Atlanta'), 2.3, 2),
            (('Newark', 'Boston'), 1.3, 2),
            (('Newark', 'Chicago'), 2.6, 2),
            (('Newark', 'Denver'), 4.3, 2),
            (('Newark', 'London'), 7.2, 2),
            (('Newark', 'Miami'), 3.1, 2),
            (('Newark', 'Philadelphia'), 1.2, 2),
            (('Newark', 'Sacramento'), 6.0, 2),
            (('Newark', 'San Diego'), 5.8, 2),
            (('Newark', 'San Francisco'), 6.2, 2),
            (('Newark', 'Washington, D.C'), 1.4, 2),
            (('Portland', 'Boise'), 1.2, 2),
            (('Portland', 'San Francisco'), 1.8, 2),
            (('Portland', ' Seattle'), 1.1, 2),
            (('Reykjavik', 'Boston'), 5.8, 2),
            (('Sacramento', 'Denver'), 2.3, 2),
            (('Sacramento', 'Newark'), 5.2, 2),
            (('Sacramento', 'San Diego'), 1.5, 2),
            (('Sacramento', 'San Francisco'), 1.1, 2),
            (('San Diego', 'Atlanta'), 4.3, 2),
            (('San Diego', 'Chicago'), 4.2, 2),
            (('San Diego', 'Denver'), 2.4, 2),
            (('San Diego', 'Honolulu'), 6.1, 2),
            (('San Diego', 'Houston'), 3.3, 2),
            (('San Diego', 'Newark'), 5.4, 2),
            (('San Diego', 'Sacramento'), 1.6, 2),
            (('San Diego', 'San Francisco'), 1.6, 2),
            (('San Diego', 'Seattle'), 3.0, 2),
            (('San Francisco', 'Anchorage'), 4.8, 2),
            (('San Francisco', 'Denver'), 2.6, 2),
            (('San Francisco', 'Honolulu'), 5.5, 2),
            (('San Francisco', 'Newark'), 5.6, 2),
            (('San Francisco', 'Portland'), 1.8, 2),
            (('San Francisco', 'Sacramento'), 0.9, 2),
            (('San Francisco', 'San Diego'), 1.6, 2),
            (('Seattle', 'Anchorage'), 3.7, 2),
            (('Seattle', 'Chicago'), 4.1, 2),
            (('Seattle', 'Denver'), 2.7, 2),
            (('Seattle', 'Portland'), 1.0, 2),
            (('Seattle', 'San Diego'), 2.8, 2),
            (('Seattle', 'San Francisco'), 2.3, 2),
            (('Washington, D.C', 'Boston'), 1.6, 2),
            (('Washington, D.C', 'Chicago'), 2.1, 2),
            (('Washington, D.C', 'Newark'), 1.4, 2),

            ]

g = Graph(edgelist)

def create_ui():
    root = tk.Tk()
    root.title("Flight Path Finder")
    tk.Label(root, text="Starting City:").grid(row=0, column=0, padx=10, pady=10)
    tk.Label(root, text="Destination City:").grid(row=1, column=0, padx=10, pady=10)
    start_city_var = tk.StringVar()
    target_city_var = tk.StringVar()
    start_city_entry = tk.Entry(root, textvariable=start_city_var)
    start_city_entry.grid(row=0, column=1, padx=10, pady=10)
    target_city_entry = tk.Entry(root, textvariable=target_city_var)
    target_city_entry.grid(row=1, column=1, padx=10, pady=10)
    output_label = tk.Label(root, text="", wraplength=400, justify="left")
    output_label.grid(row=3, column=0, padx=10, pady=10)

    def findpath():
        start_city = start_city_var.get()
        target_city = target_city_var.get()
        if not start_city or not target_city:
            messagebox.showerror("Input error", "both fields are required")
            return
        distances, previous_nodes = g.dijkstra(start_city)
        path, distances_with_path = g.get_shortest_path(previous_nodes, start_city, target_city)
        if path:
            output_label.config(text="\n".join(distances_with_path))
        else:
            output_label.config(text="No valid path")

    findpath_button = tk.Button(root, text="Find Shortest Path", command=findpath)
    findpath_button.grid(row=2, column=2, columnspan=2, padx=10, pady=10)
    root.mainloop()

create_ui()
