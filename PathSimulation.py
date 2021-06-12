import pygame
import math
import random
import time
import sys
import requests
from pygame.locals import *

def event():
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()
            sys.exit()

def telegram_bot_sendtext(bot_message):
    
    bot_token = '1756001708:AAEQffbEfE40sbJ9QrEJb1TuxeNbbeV1t30'
    bot_chatID = '976968327'
    send_text = 'https://api.telegram.org/bot' + bot_token + '/sendMessage?chat_id=' + bot_chatID + '&parse_mode=Markdown&text=' + bot_message

    response = requests.get(send_text)

    return response.json()

def drawMap(environment, start, goal):
    pygame.draw.circle(environment.map, (0, 255, 0), start, 2 + 5, 0)
    pygame.draw.circle(environment.map, (0, 255, 0), goal, 2 + 20, 1)

def dist(n1, n2):
    (x1, y1) = (n1[0], n1[1])
    (x2, y2) = (n2[0], n2[1])
    px = (float(x1) - float(x2)) ** 2
    py = (float(y1) - float(y2)) ** 2
    return (px + py) ** (0.5)

def routePath(environment, graph, goal, iteration, t1):
    while (not graph.path_to_goal()):

        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goal)
            pygame.draw.circle(environment.map, environment.grey, (X[-1], Y[-1]), environment.nodeRad*2, 0)
            pygame.draw.line(environment.map, environment.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                                environment.edgeThickness)

        else:
            X, Y, Parent = graph.expand()
            pygame.draw.circle(environment.map, environment.grey, (X[-1], Y[-1]), environment.nodeRad*2, 0)
            pygame.draw.line(environment.map, environment.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                                environment.edgeThickness)

        if iteration % 5 == 0:
            pygame.display.update()
        iteration += 1

    environment.drawPath(graph.waypoints2path())
    # path = graph.getPathCoords()
    path = graph.waypoints2path()
    path.pop(-1)
    return path

def routeFollow(environment, graph, start, goal, path, img):
    poslist = []
    while True:
        event()
        pygame.display.update()
        environment.map.blit(img, (0, 0))
        drawMap(environment, start, goal)
        environment.drawPath(graph.waypoints2path())
        environment.follow_path(path)
        environment.map.blit(environment.rotated, environment.rect)
        pos = (int(environment.xc), int(environment.yc))
        poslist.append(pos)
        if len(poslist)>1:
            if poslist[0] == poslist[1]:
                print('Reached')
                break
            else:
                poslist.pop(0)
                
class Envir:
    def __init__(self, start, goal, dimentions, robotimg, width = 0.01 * 3779.52):
        # RRTBase
        self.start = start
        self.goal = goal
        self.x, self.y = start
        self.xc, self.yc = start
        self.pmx, self.pmy = start
        # self.waypoint = 5

        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        # colors
        self.black = (0, 0, 0)
        self.grey = (70, 70, 70)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.blue = (0, 0, 255)
        self.yel = (255, 255, 0)

        # map dimenions
        self.height = dimentions[0]
        self.width = dimentions[1]

        # window settings
        pygame.display.set_caption('Car Run')
        self.map = pygame.display.set_mode((self.width, self.height))

        # robot
        self.m2p = 3779.52 # meters to pixels
        
        self.theta = 0.0
        self.w = width
        self.a = 20

        # graphics
        self.img = pygame.image.load(robotimg)
        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(-self.theta), 1)
        self.rect = self.rotated.get_rect(center = (self.xc, self.yc))

    def drawRobo(self, map):
        map.blit(self.rotated, self.rect)

    def drawPath(self, path):
        for node in path:
            pygame.draw.circle(self.map, self.red, node, 3, 0)

    def follow_path(self, path):
        self.path = path
        if len(path) > 0:
            # print(len(path))
            self.reach(self.path[-1])

    def reach(self, waypoint, speed = 1):
        # print(waypoint)
        flag = True
        distance = 0
        while flag:
            if distance == 0:
                # print('flag=true')
                pmx, pmy = self.pmx, self.pmy
            
                tx, ty = waypoint

                radians = math.atan2(ty - pmy, tx - pmx)
                distance = math.hypot(tx - pmx, ty - pmy) / speed
                distance = int(distance)
                self.theta = radians

                dx = math.cos(radians) * speed 
                dy = math.sin(radians) * speed 

                pmx, pmy = tx, ty

            if distance:
                # print(distance)
                distance -= 1
                self.xc += dx
                self.yc += dy
                if distance == 0:
                    flag = False
    
            self.rotated = pygame.transform.rotozoom(self.img, math.degrees(-self.theta), 1)
            self.rect = self.rotated.get_rect(center = (self.xc, self.yc))
       
        # environment.map.fill(0)
        self.map.blit(self.rotated, self.rect)
        pygame.time.Clock().tick(60)
        self.path.pop(-1)
        self.pmx, self.pmy = pmx, pmy
        
class RRTGraph:
    def __init__(self, start, goal, MapDimensions, obsdim = 0, obsnum = 0):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.maph, self.mapw = MapDimensions
        self.x = []
        self.y = []
        self.parent = []
        # initialize the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        # the obstacles
        self.obstacles = []
        self.obsDim = obsdim
        self.obsNum = obsnum
        # path
        self.goalstate = None
        self.path = []

    def add_node(self, n, x, y):
        self.x.insert(n, x)
        self.y.append(y)

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)

    def number_of_nodes(self):
        return len(self.x)

    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        return (px + py) ** (0.5)

    def sample_envir(self):
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return x, y

    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def isFree(self):
        n = self.number_of_nodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            if rectang.collidepoint(x, y):
                self.remove_node(n)
                return False
        return True

    def connect(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        self.add_edge(n1, n2)
        return True

    def step(self, nnear, nrand, dmax=35):
        d = self.distance(nnear, nrand)
        if d > dmax:
            u = dmax / d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + dmax * math.cos(theta)),
                      int(ynear + dmax * math.sin(theta)))
            self.remove_node(nrand)
            if abs(x - self.goal[0]) <= dmax and abs(y - self.goal[1]) <= dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)

    def bias(self, ngoal):
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_nodes()
        x, y = self.sample_envir()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.x, self.y, self.parent

    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
        return pathCoords

    def cost(self, n):
        ninit = 0
        n = n
        parent = self.parent[n]
        c = 0
        while n is not ninit:
            c = c + self.distance(n, parent)
            n = parent
            if n is not ninit:
                parent = self.parent[n]
        return c

    def waypoints2path(self):
        oldpath = self.getPathCoords()
        path = []
        for i in range(0, len(self.path) - 1):
            # print(i)
            if i >= len(self.path):
                break
            x1, y1 = oldpath[i]
            x2, y2 = oldpath[i + 1]
            # print('---------')
            # print((x1, y1), (x2, y2))
            for i in range(0, 5):
                u = i / 5
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                path.append((x, y))
                # print((x, y))

        # print(path[0])
        return path
    
def main():
    # initialisation
    dims = (1080,1920) #y, x
    clock = pygame.time.Clock()
    start=(500,1050) #x, y
    goals = [(1500,600), (254, 667), (267, 115), (1058, 110), (500,1050)] #x, y
    iteration=0
    t1=0
    rob = (r"D:\\VIT\\Sem6\EDI\\Code\\Pics\\green.png")
    bg = (r"D:\\VIT\\Sem6\\EDI\\Code\\Pics\\background.jpg")
    
    floor = pygame.image.load(bg)
    environment = Envir(start,goals[0],dims, rob)
    environment.map.blit(floor, (0, 0))
    environment.drawRobo(environment.map)

    # delta time
    lasttime = pygame.time.get_ticks()
    t1=time.time()
    for goal in goals:
        graph=RRTGraph(start,goal,dims)
        drawMap(environment, start, goal)
        path = routePath(environment, graph, goal, iteration, t1)
        routeFollow(environment, graph, start, goal, path, floor)
        time.sleep(1)
        clock.tick(30)
        start = goal
    
    test = telegram_bot_sendtext("Garbage Full Reached Final Destination!!")
    print(test)

if __name__ == '__main__':
    main()