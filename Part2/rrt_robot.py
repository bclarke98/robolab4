import sys
import time
import cozmo
import random

from cmap import *
from gui import *
from utils import *

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

G_OFFSETX = 50
G_OFFSETY = 35

def follow_parents_as_list(node, l):
    if not node:
        return
    l.append(node)
    follow_parents_as_list(node.parent, l)

async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent
    marked = {}
    mw, mh = cmap.get_size()
    sx, sy = G_OFFSETX, G_OFFSETY
    iters = 0
    path = []
    TURN_ANG = 0
    TARGET = None
    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    while True:
        iters += 1
        goalp, rlcmap = await detect_cube(robot, marked, robot_pose_as_node(robot))
        if TURN_ANG:
            await robot.turn_in_place(cozmo.util.Angle(radians=-TURN_ANG)).wait_for_completed()
            TURN_ANG = 0
        if goalp:
            TARGET = goalp
        if rlcmap:
            cmap.reset()
        if not cmap.is_solved():
            if goalp is None and len(cmap.get_goals()) == 0:
                await go_to_node(robot, Node((mw/2, mh/2)))
                await robot.turn_in_place(cozmo.util.Angle(degrees=90)).wait_for_completed()
                cmap.set_start(robot_pose_as_node(robot))
                continue
            elif len(cmap.get_goals()) > 0:
                cmap.set_start(robot_pose_as_node(robot))
                RRT(cmap, cmap.get_start())
                if cmap.is_solved():
                    path = []
                    follow_parents_as_list(TARGET, path)
        if len(path) > 0:
            targ = path.pop()
            TURN_ANG = await go_to_node(robot, targ)
            cmap.set_start(robot_pose_as_node(robot))

async def go_to_node(robot, node):
    rn = robot_pose_as_node(robot)
    ang = angle_between(rn, node)
    dist = get_dist(rn, node)
    await robot.turn_in_place(cozmo.util.Angle(radians=ang)).wait_for_completed()
    await robot.drive_straight(cozmo.util.Distance(distance_mm=dist), cozmo.util.Speed(75)).wait_for_completed()
    return ang

def robot_pose_as_node(robot):
    return Node((G_OFFSETX + robot.pose.position.x, G_OFFSETY + robot.pose.position.y))

def local_to_global(la, lp, rp):
    rcos = np.cos(la)
    rsin = np.sin(la)
    nx, ny = rp.x * rcos + rp.y * -rsin, rp.x * rsin + rp.y * rcos
    return Node((lp.x + nx, lp.y + ny))

async def detect_cube(robot, marked, curpose):
    global cmap

    rpad, cpad = 120, 70
    update_cmap = False
    goalc = None

    time.sleep(1)
    for obj in robot.world.visible_objects:
        if obj.object_id in marked:
            continue
        dx = obj.pose.position.x - robot.pose.position.x
        dy = obj.pose.position.y - robot.pose.position.y
        objp = Node((curpose.x + dx, curpose.y + dy))
        obja = obj.pose.rotation.angle_z.radians

        if robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id == obj.object_id:
            relp = Node((0, -rpad))
            # translate local pose to global pose for goal
            goalp = local_to_global(obja, objp, relp)
            if cmap.is_inside_obstacles(goalp) or not cmap.is_inbound(goalp):
                print('Goal cube is not in a valid position. Please move it.')
            else:
                cmap.clear_goals()
                cmap.add_goal(goalp)
                goalc = goalp
        obst = []
        obst.append(local_to_global(obja, objp, Node((cpad, cpad))))
        obst.append(local_to_global(obja, objp, Node((cpad, -cpad))))
        obst.append(local_to_global(obja, objp, Node((-cpad, -cpad))))
        obst.append(local_to_global(obja, objp, Node((-cpad, cpad))))
        cmap.add_obstacle(obst)
        marked[obj.object_id] = obj
        update_cmap = True
    return goalc, update_cmap

################################################################################
#                     DO NOT MODIFY CODE BELOW                                 #
################################################################################

class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            time.sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    cmap = CozMap("maps/emptygrid.json", node_generator)
    robot_thread = RobotThread()
    robot_thread.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
