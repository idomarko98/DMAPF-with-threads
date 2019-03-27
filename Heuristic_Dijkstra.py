import pdb
import math

#the function transforms a two-dimensional map into a dictionary graph of vertex and neighbors
def create_Graph(map_rows, map_cols, map):
    graph={}
    for i in range(map_rows):
        for j in range(map_cols):
            neighbors = find_neighbors(map_rows, map_cols, map, i, j)
            graph[(i,j)] = neighbors
    return graph

def find_neighbors(map_rows, map_cols, map, i, j):
    neighbors={}
    if i < map_rows - 1 and map[i + 1][j] != 1:
        neighbors[(i + 1, j)]=1

    if i > 0 and map[i - 1][j] != 1:
        neighbors[(i - 1, j)] = 1

    if j < map_cols - 1 and map[i][j + 1] != 1:
        neighbors[(i, j + 1)] = 1

    if j > 0 and map[i][j - 1] != 1:
        neighbors[(i, j - 1)] = 1
    return neighbors


def dijkstra(graph, start):
    shortest_distance = {}
    predecessor = {}
    unseenNodes = graph
    infinity = math.inf
    path = []
    for node in unseenNodes:
        shortest_distance[node] = infinity
    shortest_distance[start] = 0

    while unseenNodes:
        minNode = None
        for node in unseenNodes:
            if minNode is None:
                minNode = node
            elif shortest_distance[node] < shortest_distance[minNode]:
                minNode = node

        for childNode, weight in graph[minNode].items():
            if weight + shortest_distance[minNode] < shortest_distance[childNode]:
                shortest_distance[childNode] = weight + shortest_distance[minNode]
                predecessor[childNode] = minNode
        unseenNodes.pop(minNode)
    return shortest_distance

def create_Heuristic_map(map_rows, map_cols, map, goal):
    graph = create_Graph(map_rows, map_cols, map)
    heuristic_map= dijkstra(graph, goal)
    #print(heuristic_map)
    return heuristic_map


