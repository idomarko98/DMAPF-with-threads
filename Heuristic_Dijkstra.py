import math

'''
    A function that calculates, using Dijkstra's algorithm, the distance from each point in the map to the given
    goal. This will be used as a perfect heuristic. It receives as input the goal position and returns a dictionary
    mapping each coordinate to the distance from it to the goal.
'''
def create_Heuristic_map(map_rows, map_cols, map, goal):
    graph = create_Graph(map_rows, map_cols, map)
    heuristic_map= dijkstra(graph, goal)
    return heuristic_map


#the function gets 2dim map and transforms it into a dictionary graph of vertex and neighbors
def create_Graph(map_rows, map_cols, map):
    graph={}
    for i in range(map_rows):
        for j in range(map_cols):
            neighbors = find_neighbors(map_rows, map_cols, map, i, j)
            graph[(i,j)] = neighbors
    return graph

#the function gets a position in the map and return a list with his neighbors
def find_neighbors(map_rows, map_cols, map, i, j):
    neighbors={}
    if i < map_rows - 1 and map[i + 1][j] != 1: #check legal position on map and check position is not a wall
        neighbors[(i + 1, j)]=1

    if i > 0 and map[i - 1][j] != 1:
        neighbors[(i - 1, j)] = 1

    if j < map_cols - 1 and map[i][j + 1] != 1:
        neighbors[(i, j + 1)] = 1

    if j > 0 and map[i][j - 1] != 1:
        neighbors[(i, j - 1)] = 1
    return neighbors

#Calculates the shortest distance between a goal node and the rest of the vertices on the graph, return Heuristic_map
def dijkstra(graph, goal):
    shortest_distances = {}
    predecessor = {}
    unseenNodes = graph
    infinity = math.inf
    path = []
    for node in unseenNodes:
        shortest_distances[node] = infinity
    shortest_distances[goal] = 0

    while unseenNodes:
        minNode = None
        for node in unseenNodes:
            if minNode is None:
                minNode = node
            elif shortest_distances[node] < shortest_distances[minNode]:
                minNode = node

        for childNode, weight in graph[minNode].items():
            if weight + shortest_distances[minNode] < shortest_distances[childNode]:
                shortest_distances[childNode] = weight + shortest_distances[minNode]
                predecessor[childNode] = minNode
        unseenNodes.pop(minNode)
    return shortest_distances



