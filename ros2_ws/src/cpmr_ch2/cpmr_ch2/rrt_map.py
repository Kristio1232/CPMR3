import cv2
import numpy as np
import math
import random
import subprocess
import time
from scipy.spatial import KDTree


numberOfNodes = 1500
fileName = "big_house.png"
map = cv2.imread(fileName)

class point:
    def __init__(self, x, y,next = None, prev = None,radius = 1):
        self.x = x
        self.y = y
        self.next = next
        self.prev = prev
        self.radius = radius
        self.color = (127,127,0)



def point_obst_overlap(map, p):
    def is_not_free(x, y):
        overlap = False
        if all(map[y, x] == [0, 0, 0]):
            overlap = True
        return overlap

    mapx, mapy = p.x, p.y

    if is_not_free(mapx, mapy):
        return True    
    return False

def line_color_intersection(map, v1, v2):
    def is_not_white(x, y):
        lineOverlap = False
        if all(map[y, x] == [0, 0, 0]):
            lineOverlap = True
        return lineOverlap 
    x1, y1, x2, y2 = v1.x, v1.y, v2.x, v2.y
    dx, dy = abs(x2 - x1), abs(y2 - y1)
    x, y = x1, y1
    sx = -1 if x1 > x2 else 1
    sy = -1 if y1 > y2 else 1
    err = dx - dy
    while True:
        if x == x2 and y == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
        if is_not_white(x, y):
            return True
    return False

def findDistace (p1, p2):
    distance = math.sqrt((p1.x - p2.x)**2 + (p2.y - p1.y)**2)
    return distance

def drawLine(v1,v2,color =(128,0,128), thickness=2):
    cv2.line(map, (v1.x,v1.y), (v2.x,v2.y), color, thickness)


def bigBrainAlgo(exploredVertexList,listOfVertix):
    newConnection = 0 # This will be two different vertexes we will be returning
    smallestDistance = float('inf')
    for exploredV in exploredVertexList:
        for unexploredVertix in listOfVertix:
            calculateDistance = findDistace(exploredV,unexploredVertix)
            if calculateDistance < smallestDistance and line_color_intersection(map, exploredV, unexploredVertix) == False:
                smallestDistance = calculateDistance
                newConnection = (exploredV,unexploredVertix)

    return newConnection

def findClosestNodeToGraph(exploredVertexList, listOfVertix):
    unexploredPoints = [(v.x, v.y) for v in listOfVertix]

    tree = KDTree(unexploredPoints)

    smallestDistance = float('inf')
    newConnection = None

    for exploredV in exploredVertexList:
        distance, index = tree.query((exploredV.x, exploredV.y))

        if distance < smallestDistance and line_color_intersection(map, exploredV, listOfVertix[index]) == False:

            smallestDistance = distance
            newConnection = (exploredV, listOfVertix[index])
            print('NEW CONNECTION MADE')
        
    
        if distance == 0:
            break
    if newConnection == None:
        newConnection = bigBrainAlgo(exploredVertexList,listOfVertix)
    return newConnection

def buildMap(map):
    
    randomPoints = [(np.random.randint(10, map.shape[1]-10), np.random.randint(10, map.shape[0]-10)) for _ in range(numberOfNodes)]
    listOfVertix = []
    print(f"nodes: {numberOfNodes}")
    for i in range(0,numberOfNodes):
        v = point(x = randomPoints[i][0], y = randomPoints[i][1])
        if point_obst_overlap(map,v):
            v.color = (0, 255, 255) 
        else: 
            listOfVertix.append(v)
        cv2.circle(map, (v.x,v.y), v.radius, v.color, thickness=-1)

    cv2.imwrite("_PrelineMap"+ fileName ,map)
    return listOfVertix

def mainRead(map, start_x = 250, start_y = 250, finish_x = 15, finish_y = 15):
    
    listOfVertix = buildMap(map)

    exploredVertexList = []

    rrt = []
    startVertex = point(start_x, start_y)
    
    random_index = random.randint(0, len(listOfVertix))

    finishPoint = point(finish_x, finish_y)
    finishPoint.color=(255, 255, 0)
    
    cv2.circle(map, (startVertex.x,startVertex.y), 6, (0,255,0), thickness=-1)
    cv2.circle(map, (finishPoint.x,finishPoint.y), 6, (255,255,0), thickness=-1)

    listOfVertix.insert(random_index, finishPoint)
    exploredVertexList.append(startVertex)

    while(len(listOfVertix) > 0):

        graphNode, newNode = findClosestNodeToGraph(exploredVertexList, listOfVertix)

        graphNode.next = newNode
        newNode.prev = graphNode
        drawLine(graphNode, newNode) 
        exploredVertexList.append(newNode)
        listOfVertix.remove(newNode)


        print('ELEMENTS IN LIST OF VERTIX: ', len(listOfVertix))
        print('GRAPH NODE: ', graphNode.x, graphNode.y)
        print('NEW NODE: ', newNode.x, newNode.y)
        print('*******NODE ADDED TO RRT*******')

        # check if we have reached the goal            
        if newNode.x == finishPoint.x and newNode.y == finishPoint.y:
            print('FINISH POINT REACHED. BREAK OUT OF LOOP')
            break
        
        print('\n')  
    while(newNode.prev != None):
        pointInX = (newNode.x/10.0)
        pointInY = 63.5- (newNode.y/10.0)
        rrt.append((pointInX,pointInY))
        print(f"Location: {newNode.x} , {newNode.y}")
        drawLine(newNode, newNode.prev,(0, 0, 255), 4) 
        newNode = newNode.prev

    cv2.imwrite("_output_image"+fileName,map)
    return rrt


def movingInRos(listOfNodes):

    command = "" 
    listOfNodes.reverse()
    
    with open('wayPoints.txt', 'w') as file:
        for r in listOfNodes:
            file.write(f'{r}\n')

    print("\n\n\nWORKING ON MOVEMENT \n\n\n")
    for i in listOfNodes:
        print(f"{i[0]}, {i[1]}")
        command = f"ros2 param set /drive_to_goal newGoal \"{i[0]} & {i[1]}\"" 
        result = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print("Errors:", result.stderr)
        print("Return code:", result.returncode)
        print("Output:", result.stdout)
        time.sleep(1.0)



listOfNodes = mainRead(map, start_x=130,start_y=580,finish_x=900,finish_y=300)
x = input("____________________\n PRESS TO CONINTUE \n________________\n")
movingInRos(listOfNodes)
