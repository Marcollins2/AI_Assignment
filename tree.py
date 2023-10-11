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
heuristics = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}

print('\t\t\nFOR TREE SEARCH;')

#Code for depth first search
def depth_first_tree_search(graph, startNode, goalNode):
    stack = [(startNode, [startNode])] 
    
    while stack:
        node, path = stack.pop()
        if node == goalNode:
            return path  
        
        nodeNeighbours = graph.get(node, {})
        
        for neighbour in reversed(sorted(list(nodeNeighbours.keys()))):
            new_path = path + [neighbour] 
            stack.append((neighbour, new_path))  
                   
    return None

# Calling the function
startNode = 'S'
goalNode = 'G'
tree_path = depth_first_tree_search(graph, startNode, goalNode)

if tree_path:
    print("\nOrder of DFS-tree states expansion: ( ", " -> ".join(tree_path), " )")

    explored_nodes = list(dict.fromkeys(tree_path)) 
    print("Explored nodes: [", " , ".join(explored_nodes), "] ")
    
    unexplored_nodes = list(set(graph.keys()) - set(tree_path))
    if unexplored_nodes:
        print("Unexplored nodes: [", " , ".join(unexplored_nodes), "] ")
    else:
        print("Unexplored nodes: \"All nodes have been explored\" ")

else:
    print("\nNo path found.")


#Code for BREADTH first search
def breadth_first_tree_search(graph, startNode, goalNode):
    queue = [(startNode, [startNode])] 
    
    while queue:
        node, path = queue.pop(0)

        if node == goalNode:
            return path 
        
        nodeNeighbours = graph.get(node, {})
        count = len(nodeNeighbours.keys())
        
        for neighbour in sorted(list(nodeNeighbours.keys())):
            path.append(neighbour)
        
            if neighbour == goalNode:
                return path

        i=0
        while i < count:
            new_node = path[-count+i]
            if new_node != startNode:
                queue.append((new_node,path))
                # print('current path is : ', path)
                # print('"new node to visit"', path[-count+i])
                i+=1
    return None

# calling the function
startNode = 'S'
goalNode = 'G'
tree_path = breadth_first_tree_search(graph, startNode, goalNode)


if tree_path:
    print("\nOrder of BFS tree Search Path expansion: ( ", " -> ".join(tree_path) , " )")

    explored_nodes = list(dict.fromkeys(tree_path)) 
    print("Explored nodes: [", " , ".join(explored_nodes), "] ")

    unexplored_nodes = list(set(graph.keys()) - set(tree_path))
    if unexplored_nodes:
        print("Unexplored nodes: [", " , ".join(unexplored_nodes), "] ")
    else:
        print("Unexplored nodes: \"All nodes have been explored\" ")
    
else:
    print("\nNo path found.")


#Code for Uniform cost search
import heapq
def uniform_cost_search(graph, start, goal):
    priority_queue = [(0, start)]
    cost_so_far = {node: float('inf') for node in graph}
    cost_so_far[start] = 0
    parent = {}

    while priority_queue:
        current_cost, current_node = heapq.heappop(priority_queue)

        if current_node == goal:
            path = []
            while current_node:
                path.insert(0, current_node)
                current_node = parent.get(current_node)
            return cost_so_far[goal], path

        for neighbour, edge_cost in graph[current_node].items():
            new_cost = cost_so_far[current_node] + edge_cost

            if new_cost < cost_so_far[neighbour]:
                cost_so_far[neighbour] = new_cost
                heapq.heappush(priority_queue, (new_cost, neighbour))
                parent[neighbour] = current_node

    return None, None

start_node = 'S'
goal_node = 'G'
tree_path, path = uniform_cost_search(graph, start_node, goal_node)

if tree_path is not None:
    print(f"\nUniform Cost Search: The cost from node \"{start_node}\" to node \"{goal_node}\" is:  {tree_path}")
    print(f"Order of Uniform cost state expansion: ( {' -> '.join(path)} )")

    explored_nodes = list(dict.fromkeys(path)) 
    print("Explored nodes: [", " , ".join(explored_nodes), "] ")

    unexplored_nodes = list(set(graph.keys()) - set(path))
    if unexplored_nodes:
        print("Unexplored nodes: [", " , ".join(unexplored_nodes), "] ")
    else:
        print("Unexplored nodes: \"All nodes have been explored\" ")
else:
    print(f"Uniform Cost Search: There is no path from {start_node} to {goal_node}")


#Code for Greedy search  
import heapq
def greedy_search(graph, heuristics, start, goal):
    path = []
    
    if startNode not in graph:
            error = "\n\"ERROR!!!\": The Start Node must be in the graph"
            return error
        
    if goalNode not in graph: 
        error = "\n\"ERROR!!!\": The Goal Node must be in the graph"
        return error
    
    path.append(start)
    
    while path:
        current_node = path[-1]
        
        if current_node == goal:
            return path
        
        neighbors = graph[current_node]
        
        if not neighbors:
            path.pop() # If there are no neighbors, backtrack
        else:
            min_neighbor = min(neighbors, key=lambda neighbor: heuristics[neighbor])
            path.append(min_neighbor)

startNode = 'S'
goalNode = 'G'
path = greedy_search(graph, heuristics, startNode, goalNode)

if path:
    print(f"\nOrder of Greedy Search State expansion from \"{startNode}\" to \"{goalNode}\" :  ( {' -> '.join(path)} )")

    explored_nodes = list(dict.fromkeys(path)) 
    print("Explored nodes: [", " , ".join(explored_nodes), "] ")

    unexplored_nodes = list(set(graph.keys()) - set(path))
    if unexplored_nodes:
        print("Unexplored nodes: [", " , ".join(unexplored_nodes), "] ")
    else:
        print("Unexplored nodes: \"All nodes have been explored\" ")

else:
    print("Path not found")


#Code for A* search
import heapq
def astar_search(graph, heuristics, start, goal):
    open_nodes = [(0, start)]
    
    g_cost = {node: float('inf') for node in graph}
    g_cost[start] = 0
    parent_nodes = {}
    
    while open_nodes:
        f_cost, current_node = heapq.heappop(open_nodes) #lowest priority
        
        if current_node == goal:
            path = []
            while current_node in parent_nodes:
                path.insert(0, current_node)
                current_node = parent_nodes[current_node]
            path.insert(0, start)
            return path
        
        for neighbor, cost in graph[current_node].items():
            tentative_g_cost = g_cost[current_node] + cost
            
            if tentative_g_cost < g_cost[neighbor]:
                g_cost[neighbor] = tentative_g_cost
                
                f_cost = tentative_g_cost + heuristics[neighbor]
                heapq.heappush(open_nodes, (f_cost, neighbor))
                
                parent_nodes[neighbor] = current_node
    
    return None

startNode = 'S'
goalNode = 'G'
path = astar_search(graph, heuristics, startNode, goalNode)

if path:
    print(f"\nOrder of A* search State expansion from \"{startNode}\" to \"{goalNode}\" :  ( {' -> '.join(path)} )")

    explored_nodes = list(dict.fromkeys(path)) 
    print("Explored nodes: [", " , ".join(explored_nodes), "] ")

    unexplored_nodes = list(set(graph.keys()) - set(path))
    if unexplored_nodes:
        print("Unexplored nodes: [", " , ".join(unexplored_nodes), "] ")
    else:
        print("Unexplored nodes: \"All nodes have been explored\" ")

else:
    print("Path not found")
