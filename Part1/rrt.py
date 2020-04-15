import time
from time import sleep
from cmap import *
from gui import *
from utils import *
import random

MAX_NODES = 20000

def angle_between(node0, node1):
    return np.arctan2(node1.y - node0.y, node1.x - node0.x)

def step_from_to(node0, node1, limit=75):
    if get_dist(node0, node1) < limit:
        return node1
    theta = angle_between(node0, node1)
    return Node((node0.x + limit * np.cos(theta), node0.y + limit * np.sin(theta)))
    ############################################################################

def node_generator(cmap):
    w,h = cmap.get_size()
    rand_node = Node((random.randint(0, w-1), random.randint(0, h-1)))
    while not cmap.is_inbound(rand_node) or cmap.is_inside_obstacles(rand_node):
        rand_node = Node((random.randint(0, w-1), random.randint(0, h-1)))
    return rand_node

def RRT(cmap, start):
    cmap.add_node(start)
    while (cmap.get_num_nodes() < MAX_NODES):
        rand_node = cmap.get_random_valid_node()
        nearest_node = None
        best_dist = sys.maxsize
        for node in cmap.get_nodes():
            td = get_dist(rand_node, node)
            if td < best_dist:
                nearest_node = node
                best_dist = td
        rand_node = step_from_to(nearest_node, rand_node)
        time.sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break
    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
    else:
        print("Please try again :-(")

################################################################################
#                     DO NOT MODIFY CODE BELOW                                 #
################################################################################

class RRTThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global grid, stopevent
    stopevent = threading.Event()
    cmap = CozMap("maps/map1.json", node_generator)
    visualizer = Visualizer(cmap)
    robot = RRTThread()
    robot.start()
    visualizer.start()
    stopevent.set()
