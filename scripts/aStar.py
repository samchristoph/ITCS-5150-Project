import pygame
import heapq
import numpy as np

class AStar:
    def __init__(self, agent_radius=10, graph="digital"):
        self.graph_type = graph
        pygame.init()
        self.clock = pygame.time.Clock()
        self.running = False
        self.agent_radius = agent_radius
        self.objs = []
        self.forbiden_areas = []
        self.graph_resolution = 0
        self.display = pygame.display.set_mode((800, 600))

    def place_objects(self, objs):
        self.objs = objs

    def generate_path(self, start, dest):
        print("generate_path")
        start_tile = [int(start[0] // self.graph_resolution), int(start[1] // self.graph_resolution)]
        dest_tile = [dest[0] // self.graph_resolution, dest[1] // self.graph_resolution]
        frontier = []
        seen = set()
        cur_node = Node(start_tile[0], start_tile[1], 0, None)
        heapq.heappush(frontier, (0, cur_node)) 
        while len(frontier) != 0:
            _, cur_node = heapq.heappop(frontier)
            if cur_node.row == dest_tile[0] and cur_node.col == dest_tile[1]:
                return cur_node.retracePathFixed()
            self.expand_frontier(frontier, seen, cur_node, dest_tile)
        return None

    def expand_frontier(self, front, seen, node, dest_tile):
        for i in range(-1,2):
            for j in range(-1,2):
                if i+node.row < 0 or j+node.col < 0:
                    continue
                if i+node.row >= len(self.graph) or j+node.col >= len(self.graph[0]):
                    continue
                if (node.row+i, node.col+j) in seen:
                    continue
                if self.graph[i+node.row][j+node.col] == 1:
                    continue
                h = abs(node.row - dest_tile[0]) + abs(node.col - dest_tile[1])
                g = node.cost + (1 if i == 0 or j == 0 else 1.41)
                f = g + h
                next_node = Node(node.row + i, node.col + j, g, node, heuristic=h)
                seen.add((node.row + i, node.col + j))
                heapq.heappush(front, (f, next_node))

    def generate_digital_graph(self, width, height, resolution, start, goal):
        self.display_width = int(width*resolution)
        self.display_height = int(height*resolution)
        self.display = pygame.Surface((self.display_width, self.display_height))
        self.forbiden_areas = []
        self.graph_resolution = resolution
        for obj in self.objs:
            f_area = [obj[0]-self.agent_radius, obj[1]-self.agent_radius, obj[2]+2*self.agent_radius, obj[3]+2*self.agent_radius]
            self.forbiden_areas.append(f_area)
        self.graph = [[0 for _ in range(self.display_height // resolution)] for _ in range(self.display_width // resolution)]

        if goal[0] > len(self.graph):
            padding = int(len(self.graph) - goal[0]*100)
            buff = np.zeros([padding, len(self.graph[0])])
            self.graph = np.hstack((self.graph, buff))
        if goal[1] > len(self.graph):
            padding = int(len(self.graph) - goal[1]*100)
            buff = np.zeros([padding, len(self.graph)])
            self.graph = np.vstack((buff, self.graph))
        if start[0] < 0:
            padding = int(abs(start[0])*100)
            buff = np.zeros([len(self.graph), padding])
            self.graph = np.hstack((buff, self.graph))
            start[0] = 0
            for f_area in self.forbiden_areas:
                f_area[0] += padding
        if start[1] < 0:
            padding = int(abs(start[1])*100)
            buff = np.zeros([padding, len(self.graph[0])])
            self.graph = np.vstack((self.graph,buff))
            start[1] = 0
            for f_area in self.forbiden_areas:
                f_area[1] += padding

        for row in range(len(self.graph)):
            for col in range(len(self.graph[0])):
                tile = pygame.Rect(row*resolution, col*resolution, resolution, resolution)
                if self.inForbidenArea(tile):
                    self.graph[row][col] = 1
        return start

    def inForbidenArea(self, rect):
        for f_area in self.forbiden_areas:
            f_area_rect = pygame.Rect(f_area[0], f_area[1], f_area[2], f_area[3])
            if pygame.Rect.colliderect(f_area_rect, rect):
                return True
        return False
    
    def draw(self, path=[]):
        self.display.fill((255, 255, 255))
        for f_area in self.forbiden_areas:
            pygame.draw.rect(self.display, (255, 0, 0), pygame.Rect(f_area[0], f_area[1], f_area[2], f_area[3]))
        for node in path:
            pygame.draw.circle(self.display, (0, 0, 255), (node[0]*self.graph_resolution, node[1]*self.graph_resolution), self.agent_radius)
        pygame.display.update()

class Node:
    def __init__(self, r, c, cost, prev_node, heuristic=0):
        self.row = r
        self.col = c
        self.cost = cost
        self.prev_node = prev_node
        self.heuristic = heuristic

    def retracePath(self):
        if self.prev_node != None:
            path = self.prev_node.retracePath()
            path.append([self.row, self.col])
            return path
        else:
            return []

    def retracePathFixed(self):
        path = []
        current = self
        while current is not None:
            path.append([current.row, current.col])
            current = current.prev_node
        return path[::-1]

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)


if __name__ == "__main__":
    seen = set()
    seen.add((1,2))
    print((1,2) in seen)
    start = [0,0]
    finish = [900,900]
    aStar = AStar()
    # objs = [[20,20,100,300],[300,50,100,200],[60,700,200,50],[1000,500,300,50],[400,300,320,234],[400,320,654,23],[303,540,430,320],[674,875,44,465]]
    objs = []
    aStar.place_objects(objs)
    start = aStar.generate_digital_graph(1000,1000,1,start,finish)
    path = aStar.generate_path(start, finish)
    if path != None:
        print(path)
    else:
        print("Path not found!")
