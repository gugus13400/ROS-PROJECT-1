# -*- coding: utf-8 -*-
from queue import PriorityQueue

from cell_based_forward_search import CellBasedForwardSearch, CellLabel
from collections import deque

# This class implements the FIFO - or breadth first search - planning
# algorithm. It works by using a double ended queue: cells are pushed
# onto the back of the queue, and are popped from the front of the
# queue.
from search_grid import SearchGrid


class DIJKSTRAPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.dijkstraQueue = PriorityQueue()

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        self.dijkstraQueue.put((cell.pathCost, cell))

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.dijkstraQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.dijkstraQueue.get()
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass

    def search(self, startCoords, goalCoords):

        #NEED TO SET THE COST TO GO OF EVERY CELL TO INFINITY

        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work. 22
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

        self.start.pathCost = 0   #------> NEED TO BE SET ( C in class example)

        # Get the goal cell object and label it.
        self.goal = self.searchGrid.getCellFromCoords(goalCoords)
        self.goal.label = CellLabel.GOAL

        # If the node is being shut down, bail out here.
        if rospy.is_shutdown():
            return False

        # Draw the initial state
        self.resetGraphics()

        # Insert the start on the queue to start the process going.

        #we want general case, not only start as we will be expanding from nextCell. NOT SURE
        self.markCellAsVisitedAndRecordParent(self.start, None)
        self.pushCellOntoQueue(self.start)
        # USE PRIORITY QUEUE

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

            cell = self.popCellFromQueue() #should be equal to the cell with smallest pathCost or startcell and alive
            if cell.label != CellLabel.ALIVE:
                continue


            if (self.hasGoalBeenReached(cell) == True):
                self.goalReached = True
                break

            cells = self.getNextSetOfCellsToBeVisited(cell)
            for nextCell in cells:
                # if (self.hasCellBeenVisitedAlready(nextCell) == False):

                    if nextCell.pathCost < cell.pathCost + self.computeLStageAdditiveCost(nextCell, cell):
                        nextCell.pathCost = cell.pathCost + self.computeLStageAdditiveCost(nextCell, cell) #compute the distance between the current 'cell' and the 'nextcell'.
                        nextCell.parent = cell
                        cell.updateQueue(nextCell)

                        # self.numberOfCellsVisited = self.numberOfCellsVisited + 1
                # else:
                #     self.resolveDuplicate(nextCell, cell)
                #     if nextCell.costToGo < nextCellcurrentPathCost:
                #         nextCellcurrentPathCost = nextCeell.costToGo
                #         nextCell.parent = cell
                #         update priority queue with nextCellcurrentPathCost
                #put the nextCell.costToGo in the priority Queue, do that for every next cell around to order them from lowest to highest


            # Now that we've checked all the actions for this cell,
            # mark it as dead
            self.markCellAsDead(cell)

            #GO TO NEXT CLOSEST CELL

            # Draw the update if required
            self.drawCurrentState()

        # Do a final draw to make sure that the graphics are shown, even at the end state
        self.drawCurrentState()

        print
        "numberOfCellsVisited = " + str(self.numberOfCellsVisited)

        if self.goalReached:
            print
            "Goal reached"
        else:
            print
            "Goal not reached"

        return self.goalReached
