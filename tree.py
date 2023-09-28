graph = {
    'S': ['A', 'B'],
    'A': ['B', 'C'],
    'B': ['C'],
    'C': ['D', 'G'],
    'D': ['G'],
    'G': []
    
}

graph = {
    'S': {'A':3, 'B':1},
    'A': {'B':2, 'C':2},
    'B': {'C':3},
    'C': {'D':4, 'G':4},
    'D': {'G':1},
    'G': {}
    
}

# Heuristic function for A* Search (you can replace this with your own heuristic)
heuristic = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}

print('\t\t\nFOR TREE SEARCH;')

#Code for depth first search
def dfs_tree_search(graph, start):
    visited = set()
    stack = [start]
    while stack:
        current_node = stack.pop()
        if current_node not in visited:
            print(current_node, end=" ")
            visited.add(current_node)
            stack.extend(neighbor for neighbor in graph[current_node] if neighbor not in visited)

# Driver code
start_node = 'S'
print("\nThe following is the Depth-First Search;")
dfs_tree_search(graph, start_node)


#Code for BREADTH first search
def bfs_tree_search(graph, start):
    visited = set()
    queue = [start]
    while queue:
        current_node = queue.pop(0)
        if current_node not in visited:
            print(current_node, end=" ")
            visited.add(current_node)
            queue.extend(neighbor for neighbor in graph[current_node] if neighbor not in visited)

# Driver code
start_node = 'S'
print("\n\nThe following is the Breadth-First Search;")
bfs_tree_search(graph, start_node)


#Code for Uniform cost search
import queue
def ucs_tree_search(my_graph, start, goal):
    q = queue.PriorityQueue()
    q.put((0, [start]))
    while not q.empty():
        current_tuple = q.get()
        if current_tuple[1][-1] == goal:  # Check if the current node is the goal
            return current_tuple[1], current_tuple[0]  # Return both the path and the cost
        current_vertex = current_tuple[1][-1]
        children = list(my_graph[current_vertex].items())
        for child, child_cost in children:
            cost = current_tuple[0] + child_cost
            path = current_tuple[1].copy()
            path.append(child)
            node = (cost, path)  # Include both cost and path in the tuple
            q.put(node)
    
    return None  # Return None if the goal is not found

if __name__ == '__main__':
    result, cost = ucs_tree_search(graph, 'S', 'G')
    if result:
        print(f"\n\nUCS Tree Search: Path from 'S' to 'G' is {result}")
        print(f"UCS Tree Search: Total cost is {cost}")
    else:
        print("UCS Tree Search: No path found from 'S' to 'G'")
  

#Code for Greedy search  
import queue
def greedy_search(graph, start, goal, heuristic):
    q = queue.PriorityQueue()
    q.put((heuristic[start], [start]))  # Include the path in the queue
    while not q.empty():
        _, current_path = q.get()
        current_vertex = current_path[-1]  # Get the current node from the path
        if current_vertex == goal:
            return current_path
        for neighbor, _ in graph[current_vertex].items():
            if neighbor not in current_path:  # Avoid loops
                new_path = current_path + [neighbor]  # Extend the path
                q.put((heuristic[neighbor], new_path))
    return None

# Driver code
start_node = 'S'
goal_node = 'G'
result = greedy_search(graph, start_node, goal_node, heuristic)
if result:
    print(f"\nThe following is the Greedy Search;\n{result}")
else:
    print(f"Greedy Search: No path found from {start_node} to {goal_node}")


#Code for A* search
import queue
def a_star_search(graph, start, goal, heuristic):
    q = queue.PriorityQueue()
    q.put((0, start, []))
    while not q.empty():
        cost, current_vertex, path = q.get()
        if current_vertex == goal:
            return path + [current_vertex]
        for neighbor, edge_cost in graph[current_vertex].items():
            new_cost = cost + edge_cost
            new_path = path + [current_vertex]
            q.put((new_cost + heuristic[neighbor], neighbor, new_path))
    return None

# Driver code
start_node = 'S'
goal_node = 'G'
result = a_star_search(graph, start_node, goal_node, heuristic)
if result:
    print(f"\nThe following is the A* Search;\n{result}")
else:
    print(f"A* Search: No path found from {start_node} to {goal_node}")
