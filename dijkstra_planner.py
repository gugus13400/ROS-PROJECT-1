# -*- coding: utf-8 -*-
from Queue import PriorityQueue

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque

# This class implements the FIFO - or breadth first search - planning
# algorithm. It works by using a double ended queue: cells are pushed
# onto the back of the queue, and are popped from the front of the
# queue.
from search_grid import SearchGrid


class DIJKSTRAPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        # CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        # self.dijkstraQueue = PriorityQueue()
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.fifoQueue = deque()

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        # self.dijkstraQueue.put((cell.pathCost, cell))
        self.fifoQueue.append(cell)

    # Check the queue size is zero
    def isQueueEmpty(self):
        # return not self.dijkstraQueue
        return not self.fifoQueue

    # Simply pull from the front of the list1
    def popCellFromQueue(self):
        # cell = self.dijkstraQueue.get()
        # return cell
        cell = self.fifoQueue.popleft()
        return cell

    def resolveDuplicate(self, nextCell, cell):
        if nextCell.pathCost < cell.pathCost + self.computeLStageAdditiveCost(nextCell, cell):
            self.popCellFromQueue()
            nextCell.pathCost = cell.pathCost + self.computeLStageAdditiveCost(nextCell,cell)  # compute the distance between the current 'cell' and the 'nextcell'.
            nextCell.parent = cell
            self.numberOfCellsVisited = self.numberOfCellsVisited + 1
            self.pushCellOntoQueue(nextCell)
