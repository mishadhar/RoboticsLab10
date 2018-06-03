
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps, radians
import time
import asyncio


class node:
    """Helper object to represent nodes as we search

        Constructor inputs:
        cell -- cell where the node is located
        parent -- parent node
    """
    def __init__(self, cell, parent):
        self.cell = cell
        self.parent = parent

    def __lt__(self, other):
        return self.cell < other.cell


def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """

    #define a priority queue and a holder priority queue that we'll use to build the path
    pq = PriorityQueue()
    pqholder = {}

    #define the end point
    end = grid.getGoals()[0]

    #define the initial node based on the starting point, and with no parent
    initialNode = node(((grid.getStart()[0], grid.getStart()[1]), 0), None)

    #define the initial heuristic based on the initial node and the end point
    initialHeuristic = heuristic(initialNode.cell[0], end)

    #initialize the queue
    pqholder[initialNode.cell[0]] = initialHeuristic
    pq.put((initialHeuristic, 0, initialNode))

    #go through all items in the queue
    while pq.empty() == False:
        item = pq.get()

        #get the current goal and the current node
        currentGoal = -1 * item[1]
        currentNode = item[2]

        #mark that we've been to the current cell
        grid.addVisited(currentNode.cell[0])
        pqholder.pop(currentNode.cell[0], None)

        #if we realize that we're at the endpoint...
        if currentNode.cell[0] == end:
            #...travel up through the tree to determine the path back from the endpoint to the startpoint...
            path = []
            while currentNode != None:
                path.insert(0, currentNode.cell[0])
                currentNode = currentNode.parent

            #...and we're done!
            grid.setPath(path)
            return path

        #otherwise, look around the current node and try to figure out the options are
        neighbors = grid.getNeighbors(currentNode.cell[0])

        #go through all neighboring cells
        for n in neighbors:
            #if the cell hasn't been visited yet (only go to unvisited cells to avoid going in circles)
            if n[0] not in grid.getVisited():

                #define the heuristic and goal for this cell
                h = heuristic(n[0], end)
                g = n[1] + currentGoal
                if (n[0] not in pqholder) or (pqholder[n[0]] > h + g):
                    pq.put((h + g, -1 * g, node(n, currentNode)))
                    pqholder[n[0]] = h + g


def heuristic(current, goal):
    """Heuristic function for A* algorithm based on euclidean distance

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
    return math.sqrt(math.pow(current[0] - goal[0], 2) + math.pow(current[1] - goal[1], 2))


def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """

    global grid, stopevent

    #set/import initial grid conditions, initialize variables.  set a default initial goal to be the center of the map.
    botPoint = (3, 2)
    grid.setStart(botPoint)
    grid.addGoal((13, 9))
    cubes = set()
    goalCubePos = None
    goalScalar = -1
    cubeFootprint = [-2, -1, 0, 1, 2]
    scale = 25
    width = 26
    height = 18
    repath = False;

    #reset the robot for its journey
    robot.image_stream_enabled = True
    robot.set_head_angle(degrees(0)).wait_for_completed()
    robot.move_lift(-3)
    speed = 27
    previousHeading = 0

    while not stopevent.is_set():
        #if repath == True:
        grid.clearStart()
        grid.setStart(botPoint)
         #   repath = False;

        #run astar search
        astar(grid, heuristic)
        path = grid.getPath()

        #where are we now?
        if len(path) - path.index(botPoint) == 1:
            endPoint = botPoint
        else:
            endPoint = path[path.index(botPoint) + 1]

        print("cur: {0}".format(botPoint))
        print("end: {0}".format(endPoint))

        #find the cube(s)
        for cube in robot.world.visible_objects:
            cubeID = cube.object_id
            #if this is a 'new' cube...
            if cubeID not in cubes:
                cubex = cube.pose.position.x
                cubey = cube.pose.position.y

                #normalize the position to the size of the grid and make sure that any cubes are located INSIDE the grid
                cubex = min(math.ceil((cubex) / scale) + 2, width)
                cubey = min(math.ceil((cubey) / scale) + 2, height)

                #mark grid squares as obstacles, based on the normalized cube position and the footprint of the cube
                for xi in cubeFootprint:
                    for yi in cubeFootprint:
                        curx = cubex + xi
                        cury = cubey + yi
                        #only mark a grid square as containing an obstacle if the square is actually inside the grid
                        if curx >= 0 and curx <= width and cury >= 0 and cury <= height:
                            grid.addObstacle((curx, cury))
                grid.clearVisited()
                grid.setStart(endPoint)

                print("Found a cube: {0}".format(cubeID))
                #add this cube to the master set of cubes
                cubes.add(cubeID)

                #special casing if we realize that this is the goal cube
                #if robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id == cubeID:
                if cubeID == 1:
                    goalCubePos = (cubex, cubey)
                    #adjust the bot's final position based on the rotational position of the cube so that the bot can approach it properly
                    theta = cube.pose.rotation.angle_z.radians + math.pi
                    goalx = cubex + (round(math.cos(theta)) * math.floor((len(cubeFootprint) / 2) + 1))
                    goaly = cubey + (round(math.sin(theta)) * math.floor((len(cubeFootprint) / 2) + 1))

                    #get rid of any stale goals (e.g. center of the grid) and add the goal position
                    print("Found Goal Cube at: "+ str(goalx) + ", " + str(goaly))
                    grid.clearGoals()
                    grid.addGoal((goalx, goaly))
                    print(grid.getGoals())
                    repath = True

                    grid.clearStart()
                    grid.setStart(botPoint)
                    grid.clearVisited()

                    # run astar search
                    astar(grid, heuristic)

                    path = grid.getPath()

                    # where are we now?
                    if len(path) - path.index(botPoint) == 1:
                        endPoint = botPoint
                    else:
                        endPoint = path[path.index(botPoint) + 1]

                    print("bot: {0}".format(botPoint))
                    print("end: {0}".format(endPoint))

        #if repath == False:
        #get the vector of displacement between the endpoint and the bot's current point
        dispVector = (endPoint[0] - botPoint[0], endPoint[1] - botPoint[1])
        #get a scalar measure of how far the endpoint is from the bot's current point
        dispScalar = math.sqrt(math.pow(dispVector[0], 2) + math.pow(dispVector[1], 2))
        #atan2(y,x) gives the absolute angle of a vector, so we can use it to get the best heading
        heading = math.atan2(dispVector[1], dispVector[0])
        #we turn based on the difference between the desired heading and the bot's previous heading
        turn = heading - previousHeading
        previousHeading = heading

        if dispScalar != 0:
            #turn and move the bot
            robot.turn_in_place(radians(turn)).wait_for_completed()
            robot.drive_wheels(speed, speed, duration = 2*dispScalar)

        if goalCubePos is not None:
            goalVector = (goalCubePos[0] - botPoint[0], goalCubePos[1] - botPoint[1])
            goalScalar = math.sqrt(math.pow(goalVector[0], 2) + math.pow(goalVector[1], 2))

        #if we've reached the center of the grid and still haven't found the goal cube, slowly turn in place (until we find it)
        if dispScalar == 0 and goalCubePos is None:
            robot.turn_in_place(degrees(15)).wait_for_completed()
            print("Turning in place til we find the cube")
        #if we HAVE found the goal cube, and we've reached the endpoint...
        elif dispScalar == 0 and goalCubePos is not None and goalScalar == 0:
            #get the heading and turn to face the cube
            heading = math.atan2(goalCubePos[1] - endPoint[1], goalCubePos[0] - endPoint[0])
            turn = heading - previousHeading
            robot.turn_in_place(radians(turn)).wait_for_completed()
            print("Reached goal cube")
            #and we're done!
            robot.set_all_backpack_lights(cozmo.lights.blue_light)
            robot.turn_in_place(degrees(720))
            robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)
            time.sleep(5)
            break

        #update the bot's point to be the endpoint it just navigated to
        botPoint = endPoint
        #grid.setStart(botPoint)


######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

