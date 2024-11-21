import cv2
import numpy as np
import math
import random
import subprocess
import time
from scipy.spatial import KDTree

NODE_COUNT = 1500
IMAGE_FILE = "big_house.png"
IMAGE = cv2.imread(IMAGE_FILE)

class Node:
    def __init__(self, x, y, next=None, prev=None, radius=1):
        self.x = x
        self.y = y
        self.next = next
        self.prev = prev
        self.radius = radius
        self.color = (127, 127, 0)

def check_collision(image, node):
    def is_obstacle(x, y):
        return all(image[y, x] == [0, 0, 0])
    return is_obstacle(node.x, node.y)

def check_path_collision(image, n1, n2):
    def is_obstacle(x, y):
        return all(image[y, x] == [0, 0, 0])
    
    x1, y1, x2, y2 = n1.x, n1.y, n2.x, n2.y
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
        if is_obstacle(x, y):
            return True
    return False

def calculate_distance(n1, n2):
    return math.sqrt((n1.x - n2.x)**2 + (n2.y - n1.y)**2)

def draw_path(n1, n2, color=(128, 0, 128), thickness=2):
    cv2.line(IMAGE, (n1.x, n1.y), (n2.x, n2.y), color, thickness)

def find_nearest_connection(explored, unexplored):
    nearest = None
    min_distance = float('inf')
    for e in explored:
        for u in unexplored:
            dist = calculate_distance(e, u)
            if dist < min_distance and not check_path_collision(IMAGE, e, u):
                min_distance = dist
                nearest = (e, u)
    return nearest

def find_nearest_node(explored, unexplored):
    unexplored_coords = [(v.x, v.y) for v in unexplored]
    tree = KDTree(unexplored_coords)
    
    nearest = None
    min_distance = float('inf')
    
    for e in explored:
        distance, index = tree.query((e.x, e.y))
        if distance < min_distance and not check_path_collision(IMAGE, e, unexplored[index]):
            min_distance = distance
            nearest = (e, unexplored[index])
        if distance == 0:
            break
    
    return nearest or find_nearest_connection(explored, unexplored)

def generate_nodes(image):
    nodes = []
    for _ in range(NODE_COUNT):
        x = np.random.randint(10, image.shape[1] - 10)
        y = np.random.randint(10, image.shape[0] - 10)
        node = Node(x=x, y=y)
        if not check_collision(image, node):
            nodes.append(node)
        else:
            node.color = (0, 255, 255)
        cv2.circle(image, (node.x, node.y), node.radius, node.color, thickness=-1)
    
    cv2.imwrite("_PrelineMap" + IMAGE_FILE, image)
    return nodes

def create_rrt(image, start_x=250, start_y=250, goal_x=15, goal_y=15):
    nodes = generate_nodes(image)
    explored = []
    path = []
    
    start = Node(start_x, start_y)
    goal = Node(goal_x, goal_y)
    goal.color = (255, 255, 0)
    
    cv2.circle(image, (start.x, start.y), 6, (0, 255, 0), thickness=-1)
    cv2.circle(image, (goal.x, goal.y), 6, (255, 255, 0), thickness=-1)
    
    nodes.insert(random.randint(0, len(nodes)), goal)
    explored.append(start)
    
    while nodes:
        parent, child = find_nearest_node(explored, nodes)
        parent.next = child
        child.prev = parent
        draw_path(parent, child)
        explored.append(child)
        nodes.remove(child)
        
        if child.x == goal.x and child.y == goal.y:
            break
    
    current = child
    while current.prev:
        x = current.x / 10.0
        y = 63.5 - (current.y / 10.0)
        path.append((x, y))
        draw_path(current, current.prev, (0, 0, 255), 4)
        current = current.prev
    
    cv2.imwrite("_output_image" + IMAGE_FILE, image)
    return path

def execute_path(path):
    path.reverse()
    with open('wayPoints.txt', 'w') as file:
        for point in path:
            file.write(f'{point}\n')
    
    for x, y in path:
        command = f"ros2 param set /drive_to_goal newGoal \"{x} & {y}\""
        subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        time.sleep(1.0)

path = create_rrt(IMAGE, start_x=130, start_y=580, goal_x=900, goal_y=300)
input("Press Enter to continue...")
execute_path(path)