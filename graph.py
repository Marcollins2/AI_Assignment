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
print('\t\t \n FOR GRAPH SEARCH;')

#Code for depth first search
visited = set() # Set to keep track of visited nodes of graph.

def dfs(visited, graph, node):  #function for dfs 
    if node not in visited:
        print (node)
        visited.add(node)
        for neighbour in graph[node]:
            dfs(visited, graph, neighbour)

# Driver Code
print("The following is the Depth-First Search;")
dfs(visited, graph, 'S')


#Code for breadth first search
visited = [] # List for visited nodes.
queue = []     #Initialize a queue

def bfs(visited, graph, node): #function for BFS
  visited.append(node)
  queue.append(node)

  while queue:          # Creating loop to visit each node
    m = queue.pop(0) 
    print (m, end = " ") 

    for neighbour in graph[m]:
      if neighbour not in visited:
        visited.append(neighbour)
        queue.append(neighbour)

# Driver Code
print("\nThe following is the Breadth-First Search;")
bfs(visited, graph, 'S')    # function calling


#Code for Uniform Cost search
import queue
def ucs_graph_search(my_graph, start, goal):
    q = queue.PriorityQueue()
    visited = set()  # Keep track of visited nodes
    q.put((0, [start]))
    while not q.empty():
        current_tuple = q.get()
        current_vertex = current_tuple[1][-1]
        if current_vertex == goal:  # Check if the current node is the goal
            return current_tuple[1], current_tuple[0]  # Return both the path and the cost
        visited.add(current_vertex)  # Mark the current node as visited
        children = list(my_graph[current_vertex].items())
        for child, child_cost in children:
            if child not in visited:  # Only expand unvisited nodes
                cost = current_tuple[0] + child_cost
                path = current_tuple[1].copy()
                path.append(child)
                node = (cost, path)  # Include both cost and path in the tuple
                q.put(node)
    
    return None  # Return None if the goal is not found

if __name__ == '__main__':
    result, cost = ucs_graph_search(graph, 'S', 'G')
    if result:
        print(f"\n\nUCS Graph Search: Path from 'S' to 'G' is {result}")
        print(f"UCS Graph Search: Total cost is {cost}")
    else:
        print("UCS Graph Search: No path found from 'S' to 'G'")
    
   
#Code for Greedy  search   
import queue
def greedy_search_graph(graph, start, goal, heuristic):
    q = queue.PriorityQueue()
    visited = set()  # Track visited nodes
    q.put((heuristic[start], start, [start]))  # Include the path in the queue
    while not q.empty():
        _, current_vertex, path = q.get()
        if current_vertex == goal:
            return path
        visited.add(current_vertex)  # Mark the current node as visited
        for neighbor, _ in graph[current_vertex].items():
            if neighbor not in visited:
                new_path = path + [neighbor]  # Extend the path
                q.put((heuristic[neighbor], neighbor, new_path))
    return None

# Driver code
start_node = 'S'
goal_node = 'G'
result = greedy_search_graph(graph, start_node, goal_node, heuristic)
if result:
    print(f"\nThe following is the Greedy search; \n {result}")
else:
    print(f"Greedy Search: No path found from {start_node} to {goal_node}")



#Code for A* search
def a_star_search(graph, start, target, heuristics):
    priority_queue = [(heuristics[start], 0, start, [start])]
    while priority_queue:
        (_, cost, node, path) = priority_queue.pop(0)
        if node == target:
            return path
        neighbors = graph[node]
        for neighbor, neighbor_cost in sorted(neighbors.items(), key=lambda x: heuristics[x[0]]):
            if neighbor not in path:
                new_cost = cost + neighbor_cost
                priority_queue.append((new_cost + heuristics[neighbor], new_cost, neighbor, path + [neighbor]))
                priority_queue.sort(key=lambda x: x[0])
start_node = 'S'
target_node = 'G'

a_star_path = a_star_search(graph, start_node, target_node, heuristic)
print("The following is the A* Search Path:", a_star_path)
