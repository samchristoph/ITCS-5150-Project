import pygame
import heapq

class AStar:
    def __init__(self, graph="digital"):
        self.graph_type = graph
        pygame.init()
        self.display = pygame.display.set_mode((1200, 900))
        self.display_width = 1200
        self.display_height = 900 
        self.clock = pygame.time.Clock()
        self.running = False
        self.agent_radius = 10
        self.objs = []
        self.forbiden_areas = []
        self.graph_resoultion = 0

    def placeObjects(self, objs):
        self.objs = objs

    def generatePath(self, start, dest):
        start_tile = [start[0] // self.graph_resoultion, start[1] // self.graph_resoultion]
        print("start_tile: ", start_tile)
        dest_tile = [dest[0] // self.graph_resoultion, dest[1] // self.graph_resoultion]
        print("dest_tile: ", dest_tile)
        frontier = []
        seen = set()
        cur_node = Node(start_tile[0], start_tile[1], 0, None)
        heapq.heappush(frontier, (0, cur_node)) 
        while len(frontier) != 0:
            _, cur_node = heapq.heappop(frontier)
            print((cur_node.row, cur_node.col))
            if cur_node.row == dest_tile[0] and cur_node.col == dest_tile[1]:
                return cur_node.retracePath()
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
                    print("hit")
                    continue
                if self.graph[i+node.row][j+node.col] == 1:
                    continue
                h = abs(node.row - dest_tile[0]) + abs(node.col - dest_tile[1])
                if i == 0 or j == 0:
                    g = node.cost + 1
                else:
                    g = node.cost + 1.41
                f = g + h
                next_node = ((node.row+i, node.col+j))
                seen.add(next_node)
                heapq.heappush(front, (f, Node(node.row+i, node.col+j, g, node))) 

    def generateDigitalGraph(self, resolution=20):
        self.forbiden_areas = []
        self.graph_resoultion = resolution
        for obj in self.objs:
            f_area = [obj[0]-self.agent_radius, obj[1]-self.agent_radius, obj[2]+2*self.agent_radius, obj[3]+2*self.agent_radius]
            self.forbiden_areas.append(f_area)
        self.graph = [[0 for _ in range(self.display_height // resolution)] for _ in range(self.display_width // resolution)]
        for row in range(len(self.graph)):
            for col in range(len(self.graph[0])):
                tile = pygame.Rect(row*resolution, col*resolution, resolution, resolution)
                if self.inForbidenArea(tile):
                    self.graph[row][col] = 1

    def inForbidenArea(self, rect):
        for f_area in self.forbiden_areas:
            f_area_rect = pygame.Rect(f_area[0], f_area[1], f_area[2], f_area[3])
            if pygame.Rect.colliderect(f_area_rect, rect):
                return True
        return False

    def run(self):
        self.running = True
        start = [20, 800]
        dest = [1100, 700]
        path = self.generatePath(start, dest)
        print(path)
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
            self.display.fill("purple")
            for row in range(len(self.graph)):
                for col in range(len(self.graph[0])):
                    if self.graph[row][col] == 1:
                        tile_rect = pygame.Rect(row*self.graph_resoultion, col*self.graph_resoultion, self.graph_resoultion, self.graph_resoultion)
                        pygame.draw.rect(self.display, (0,200,0), tile_rect)
            for f_area in self.forbiden_areas:
                f_area_rect = pygame.Rect(f_area[0], f_area[1], f_area[2], f_area[3])
                pygame.draw.rect(self.display, (200,0,0), f_area_rect)
            for obj in self.objs:
                obj_rect = pygame.Rect(obj[0], obj[1], obj[2], obj[3])
                pygame.draw.rect(self.display, (0,0,0), obj_rect)
            if path != None:
                for tile in path:
                    tile_rect = pygame.Rect(tile[0]*self.graph_resoultion, tile[1]*self.graph_resoultion, self.graph_resoultion, self.graph_resoultion)
                    pygame.draw.rect(self.display, (0,0,200), tile_rect)
            for r in range(0, self.display_height, self.graph_resoultion):
                pygame.draw.line(self.display, (50,50,50), (0,r), (self.display_width,r))
            for c in range(0, self.display_width, self.graph_resoultion):
                pygame.draw.line(self.display, (50,50,50), (c,0), (c,self.display_height))
            start_rect = pygame.Rect(start[0], start[1],self.graph_resoultion, self.graph_resoultion)
            dest_rect = pygame.Rect(dest[0], dest[1],self.graph_resoultion, self.graph_resoultion)
            pygame.draw.rect(self.display, (200,200,0), start_rect)
            pygame.draw.rect(self.display, (0,200,200), dest_rect)
            pygame.display.flip()
            self.clock.tick(60)
        pygame.quit()


class Node:
    def __init__(self, r, c, cost, prev_node):
        self.row = r
        self.col = c
        self.cost = cost
        self.prev_node = prev_node

    def retracePath(self):
        if self.prev_node == None:
            return []
        path = self.prev_node.retracePath()
        path.append([self.row, self.col])
        return path

    def __lt__(self, other):
        return True


if __name__ == "__main__":
    seen = set()
    seen.add((1,2))
    print((1,2) in seen)
    aStar = AStar()
    objs = [[20,20,100,300],[300,50,100,200],[60,700,200,50],[1000,500,300,50],[400,300,320,234],[400,320,654,23],[303,540,430,320],[674,875,44,465]]
    aStar.placeObjects(objs)
    aStar.generateDigitalGraph()
    aStar.run()
