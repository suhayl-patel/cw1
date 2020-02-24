# -*- coding: utf-8 -*-

from occupancy_grid import OccupancyGrid
from search_grid import SearchGrid
from planner_base import PlannerBase
from collections import deque
from cell import *
from planned_path import PlannedPath
from math import *
from bresenham import bresenham
import rospy
class GeneralForwardSearchAlgorithm(PlannerBase):

    # This class implements the basic framework for LaValle's general
    # template for forward search. It includes a lot of methods for
    # managing the graphical output as well.

    # Define some class variables. As the popCounter and pushCounter will only be used
    # when the 'while' loop of LaVelle's forward search algorithm is implemented, the
    # starting value for the QueueSize and MaxQueueSize is set to 1, as there is already
    # one cell in the queue.  
    NumberOfPushes = 0
    NumberOfPops = 0
    QueueSize = 1
    MaxQueueSize = 1

    def __init__(self, title, occupancyGrid):
        PlannerBase.__init__(self, title, occupancyGrid)

        # Flag to store if the last plan was successful
        self.goalReached = None

    # These methods manage the queue of cells to be visied.
    def pushCellOntoQueue(self, cell):
        raise NotImplementedError()

    # This method will be used to keep track of the number of pushes
    def pushCounter(self):
	GeneralForwardSearchAlgorithm.NumberOfPushes += 1

    # This method returns a boolean - true if the queue is empty,
    # false if it still has some cells on it.
    def isQueueEmpty(self):
        raise NotImplementedError()

    # This method finds the first cell (at the head of the queue),
    # removes it from the queue, and returns it.
    def popCellFromQueue(self):
        raise NotImplementedError()

    # This method determines if the goal has been reached.
    def hasGoalBeenReached(self, cell):
        raise NotImplementedError()

    # This method gets the list of cells which could be visited next.
    def getNextSetOfCellsToBeVisited(self, cell):
        raise NotImplementedError()

    # This method determines whether a cell has been visited already.
    def hasCellBeenVisitedAlready(self, cell):
        raise NotImplementedError()

    def markCellAsVisitedAndRecordParent(self, cell, parentCell):
        cell.label = CellLabel.ALIVE
        cell.parent = parentCell

    # Mark that a cell is dead. A dead cell is one in which all of its
    # immediate neighbours have been visited.
    def markCellAsDead(self, cell):
        cell.label = CellLabel.DEAD

    # Handle the case that a cell has been visited already.
    def resolveDuplicate(self, nextCell, cell):
        raise NotImplementedError()

    def popCounter(self):
	GeneralForwardSearchAlgorithm.NumberOfPops += 1

    # This method is used to calculate the new QueueSize.
    # This method will be used in the 'while' loop of LaVelle's
    # algorithm, after each pop and push. The pop and push counter are
    # reset to 0 after each pop and push. Then the new QueuSize can be 
    # set to the MaxQueueSize if it is greater than the previous value.
    def setMaxQueueSize(self):
	GeneralForwardSearchAlgorithm.QueueSize = GeneralForwardSearchAlgorithm.QueueSize + (GeneralForwardSearchAlgorithm.NumberOfPushes -  GeneralForwardSearchAlgorithm.NumberOfPops)
	
	GeneralForwardSearchAlgorithm.NumberOfPushes = 0
	GeneralForwardSearchAlgorithm.NumberOfPops = 0

	if (GeneralForwardSearchAlgorithm.QueueSize > GeneralForwardSearchAlgorithm.MaxQueueSize):
	    GeneralForwardSearchAlgorithm.MaxQueueSize = GeneralForwardSearchAlgorithm.QueueSize


    def computeAngle(self, pathEndCell, cell, parentParentCell):

        x_vector_coordinate = pathEndCell.coords[0] - cell.coords[0]
        y_vector_coordinate = pathEndCell.coords[1] - cell.coords[1]

        x_vector_coordinate_parent = cell.coords[0] - parentParentCell.coords[0]
        y_vector_coordinate_parent = cell.coords[1] - parentParentCell.coords[1]

        # print('The Pathend cell coord in x is :' + str(pathEndCell.coords[0]) + 'and in y is: ' + str(pathEndCell.coords[1]) )
        # print('The cell coord in x is :' + str(cell.coords[0]) + 'and in y is: ' + str(cell.coords[1]))
        # print('The parent parent cell coord in x is :' + str(parentParentCell.coords[0]) + 'and in y is: ' + str(parentParentCell.coords[1]))
        vector_a = x_vector_coordinate * x_vector_coordinate_parent
        vector_b = y_vector_coordinate * y_vector_coordinate_parent

        sqrt_1 = sqrt((x_vector_coordinate * x_vector_coordinate) + (y_vector_coordinate * y_vector_coordinate))
        sqrt_2 = sqrt( (x_vector_coordinate_parent *  x_vector_coordinate_parent) + (y_vector_coordinate_parent * y_vector_coordinate_parent))

        self.angle_rad = acos(( vector_a + vector_b)  / (sqrt_1 * sqrt_2))
        self.angle_deg = degrees(self.angle_rad)

        return self.angle_deg

    def computeHeuristicConstant(self):
        constant = 10
        return constant

    def computeHeuristicManhattan(self, nextCell):
        bressenham_1 = list(bresenham(nextCell.coords[0], nextCell.coords[1], nextCell.coords[0], self.goal.coords[1]))
        L_1 = 0
        previous_cell = nextCell
        for cells in bressenham_1:
            theCellIs = self.searchGrid.getCellFromCoords(cells)
            dx = abs(theCellIs.coords[0] - previous_cell.coords[0])
            dy = abs(theCellIs.coords[1] - previous_cell.coords[1])
            cost = min(1 + (0.2 / ((1.75 - theCellIs.terrainCost) ** 2)) ** 2, 1000)
            L_1 += cost * (dx  + dy)
            previous_cell = theCellIs

        bressenham_2 = list(bresenham(nextCell.coords[0], self.goal.coords[1], self.goal.coords[0], self.goal.coords[1]))
        L_2 = 0
        previous_cell = nextCell
        for cells in bressenham_2:
            theCellIs = self.searchGrid.getCellFromCoords(cells)
            dx = abs(theCellIs.coords[0] - previous_cell.coords[0])
            dy = abs(theCellIs.coords[1] - previous_cell.coords[1])
            cost = min(1 + (0.2 / ((1.75 - theCellIs.terrainCost) ** 2)) ** 2, 1000)
            L_2 += cost * (dx + dy)
            previous_cell = theCellIs

        # dx1 = nextCell.coords[0] - self.goal.coords[0]
        # dy1 = nextCell.coords[1] - self.goal.coords[1]
        # dx2 = self.start.coords[0] - self.goal.coords[0]
        # dy2 = self.start.coords[1] - self.goal.coords[1]
        # cross = abs(dx1 * dy2 - dx2 * dy1)

        return (L_1 + L_2)

    def computeHeuristicEuclidian(self, nextCell):
        bressenham = list(bresenham(nextCell.coords[0], nextCell.coords[1], self.goal.coords[0], self.goal.coords[1]))
        L_total = 0
        previous_cell = nextCell
        for cells in bressenham:
            theCellIs = self.searchGrid.getCellFromCoords(cells)
            dx = abs(theCellIs.coords[0] - previous_cell.coords[0])
            dy = abs(theCellIs.coords[1] - previous_cell.coords[1])
            cost = min(1+(0.2/((1.75-theCellIs.terrainCost)**2))**2, 1000)
            L_total += cost * (sqrt(dx * dx + dy * dy))
            previous_cell = theCellIs

        # dx = abs(nextCell.coords[0] - self.goal.coords[0])
        # dy = abs(nextCell.coords[1] - self.goal.coords[1])
        # L = sqrt(dx *dx + dy * dy)
        #
        # return L
        # dx1 = nextCell.coords[0] - self.goal.coords[0]
        # dy1 = nextCell.coords[1] - self.goal.coords[1]
        # dx2 = self.start.coords[0] - self.goal.coords[0]
        # dy2 = self.start.coords[1] - self.goal.coords[1]
        # cross = abs(dx1 * dy2 - dx2 * dy1)
        return L_total

    def computeHeuristicOctile(self, nextCell):
        dx = abs(nextCell.coords[0] - self.goal.coords[0])
        dy = abs(nextCell.coords[1] - self.goal.coords[1])

        # dx1 = nextCell.coords[0] - self.goal.coords[0]
        # dy1 = nextCell.coords[1] - self.goal.coords[1]
        # dx2 = self.start.coords[0] - self.goal.coords[0]
        # dy2 = self.start.coords[1] - self.goal.coords[1]
        # cross = abs(dx1 * dy2 - dx2 * dy1)
        return ((max(dx, dy)) + (sqrt(2) - 1) * min(dx, dy))

    # Compute the additive cost of performing a step from the parent to the
    # current cell. This calculation is carried out the same way no matter
    # what heuristics, etc. are used. The cost computed here takes account
    # of the terrain traversability cost using an equation a bit like that
    # presented in the lectures.
    def computeEuclidianCost(self, nextCell, goalCell):
        if (nextCell is None):
            return

            # Travel cost is Cartesian distance
        dX = goalCell.coords[0] - nextCell.coords[0]
        dY = goalCell.coords[1] - nextCell.coords[1]

        L = sqrt(dX * dX + dY * dY)
        return L

    def computeLStageAdditiveCost(self, parentCell, cell):
        # If the parent is empty, this is the start of the path and the
        # cost is 0.
        if (parentCell is None):
            return

        # Travel cost is Cartesian distance
        dX = cell.coords[0] - parentCell.coords[0]
        dY = cell.coords[1] - parentCell.coords[1]
        # Terrain cost
        #  Run this in matlab to visualise ro check the image
        # However, basically it builds up extremely quickly
        # x=[1:0.01:2];
        # c=min(1+(.2./((1.7-x).^2)).^2,1000);
        cost = min(1+(0.2/((1.75-cell.terrainCost)**2))**2, 1000)
        L = sqrt(dX * dX + dY * dY)*cost# Multiplied by the terrain cost of the cell

        return L
        
    # The main search routine. The routine searches for a path between a given
    # set of coordinates. These are then converted into start and destination
    # cells in the search grid and the search algorithm is then run.
    def search(self, startCoords, goalCoords):

        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.isQueueEmpty() == False):
            self.popCellFromQueue()

        # Create or update the search grid from the occupancy grid and seed
        # unvisited and occupied cells.
        if (self.searchGrid is None):
            self.searchGrid = SearchGrid.fromOccupancyGrid(self.occupancyGrid)
        else:
            self.searchGrid.updateFromOccupancyGrid()

        # Get the start cell object and label it as such. Also set its
        # path cost to 0.
        self.start = self.searchGrid.getCellFromCoords(startCoords)
        self.start.label = CellLabel.START
        self.start.pathCost = 0

        # Get the goal cell object and label it.
        self.goal = self.searchGrid.getCellFromCoords(goalCoords)
        self.goal.label = CellLabel.GOAL

        # If the node is being shut down, bail out here.
        if rospy.is_shutdown():
            return False

        # Draw the initial state
        self.resetGraphics()

        # Insert the start on the queue to start the process going.
        self.markCellAsVisitedAndRecordParent(self.start, None)
        self.pushCellOntoQueue(self.start)

        # Reset the count
        self.numberOfCellsVisited = 0

        # Indicates if we reached the goal or not
        self.goalReached = False


        # Iterate until we have run out of live cells to try or we reached the goal.
        # This is the main computational loop and is the implementation of
        # LaValle's pseudocode
        while (self.isQueueEmpty() == False):

            # Check if ROS is shutting down; if so, abort. This stops the
            # planner from hanging.
            if rospy.is_shutdown():
                return False


            
            cell = self.popCellFromQueue()
            # print(cell)

	    # Increase the popCounter by 1
	    self.popCounter()
	    # Calculate the current Queuesize, then reset the popCounter and pushCounter to 0
	    # and set the QueueSize to the maxQueueSize if it is bigger than the previous.
	    self.setMaxQueueSize()


            if (self.hasGoalBeenReached(cell) == True):
                self.goalReached = True
                break

            cells = self.getNextSetOfCellsToBeVisited(cell)
            for nextCell in cells:
                if (self.hasCellBeenVisitedAlready(nextCell) == False):
                    self.markCellAsVisitedAndRecordParent(nextCell, cell)

                    #nextCell.pathCost = cell.pathCost + self.computeLStageAdditiveCost(nextCell,cell) # DIJKSTRA #

                    #nextCell.pathCost = self.computeEuclidianCost(nextCell, self.goal) # GREEDY #

                    nextCell.pathCost = cell.pathCost + self.computeLStageAdditiveCost(nextCell,cell) +  self.computeHeuristicManhattan(nextCell) # Manhattan #

                    #nextCell.pathCost = cell.pathCost + self.computeLStageAdditiveCost(nextCell, cell) + self.computeHeuristicEuclidian(nextCell)  # Euclidian #

                    #nextCell.pathCost = cell.pathCost + self.computeLStageAdditiveCost(nextCell,cell) + self.computeHeuristicOctile(nextCell) # Octile #

                    #nextCell.pathCost = cell.pathCost + self.computeLStageAdditiveCost(nextCell, cell) + self.computeHeuristicConstant() # Constant #


                    self.pushCellOntoQueue(nextCell) #push the cell information ( cell name + distance to goal in the priority queue)

		    # Increase the pushCounter by 1
		    self.pushCounter()
		    # Calculate the current Queuesize, reset the pop and push counter to 0
	            # and set the QueueSize to the maxQueueSize if it is bigger than the previous.

		    self.setMaxQueueSize()


                    self.numberOfCellsVisited = self.numberOfCellsVisited + 1

                elif (nextCell.label == CellLabel.ALIVE):
                    self.resolveDuplicate(nextCell, cell)


            # Now that we've checked all the actions for this cell,
            # mark it as dead
            self.markCellAsDead(cell)

            # Draw the update if required
            self.drawCurrentState()

        # Do a final draw to make sure that the graphics are shown, even at the end state
        self.drawCurrentState()
        
        print "numberOfCellsVisited = " + str(self.numberOfCellsVisited)
        print "MaxQueueSize = " + str(GeneralForwardSearchAlgorithm.MaxQueueSize)

        
        if self.goalReached:
            print "Goal reached"
        else:
            print "Goal not reached"

        return self.goalReached

    # This method extracts a path from the pathEndCell to the start
    # cell. The path is a list actually sorted in the order:
    # cell(x_1), cell(x_2), ... , cell(x_K), cell(x_G). You can use
    # this method to try to find the path from any end cell. However,
    # depending upon the planner used, the results might not be
    # valid. In this case, the path will probably not terminate at the
    # start cell.
    def extractPathEndingAtCell(self, pathEndCell, colour):

        # Construct the path object and mark if the goal was reached
        path = PlannedPath()

        path.goalReached = self.goalReached

        # Initial condition - the goal cell
        path.waypoints.append(pathEndCell)

        # Start at the goal and find the parent. Find the cost associated with the parent
        previousCell = pathEndCell
        cell = pathEndCell.parent
        parentParentCell = cell.parent
        path.travelCost = self.computeLStageAdditiveCost(pathEndCell.parent, pathEndCell)
        pathAngle = self.computeAngle(pathEndCell, cell, parentParentCell)
        print(pathAngle)
        print(cell.coords[0])

        # Iterate back through and extract each parent in turn and add
        # it to the path. To work out the travel length along the
        # path, you'll also have to add self at self stage.
        while (cell is not None):
            if (cell.parent == None):
                break

            path.waypoints.appendleft(cell)
            path.travelCost = path.travelCost + self.computeLStageAdditiveCost(cell.parent, cell)

            pathAngle += self.computeAngle(previousCell, cell, cell.parent)

            cell = cell.parent
            previousCell = previousCell.parent

        # Update the stats on the size of the path
        path.numberOfWaypoints = len(path.waypoints)

        # Note that if we failed to reach the goal, the above mechanism computes a path length of 0.
        # Therefore, if we didn't reach the goal, change it to infinity
        if path.goalReached is False:
            path.travelCost = float("inf")

        print("The total path Angle is {} degrees".format(pathAngle))

        print "Path travel cost = " + str(path.travelCost)
        print "Path cardinality = " + str(path.numberOfWaypoints)

        # Draw the path if requested
        if (self.showGraphics == True):
            self.plannerDrawer.update()
            self.plannerDrawer.drawPathGraphicsWithCustomColour(path, colour)
            self.plannerDrawer.waitForKeyPress()

        # Return the path
        return path

    # Extract the path from a specified end cell to the start. This is not
    # necessarily the full path. Rather, it lets us illustrate parts of the
    # path.
    def extractPathEndingAtCoord(self, endCellCoord):
        endCell = self.searchGrid.getCellFromCoords(endCellCoord)
        self.extractPathEndingAtCell(endCell, 'red')

    # Extract the path between the start and goal.
    def extractPathToGoal(self):
        path = self.extractPathEndingAtCell(self.goal, 'yellow')

        return path


