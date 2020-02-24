# -*- coding: utf-8 -*-
from Queue import PriorityQueue

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque


# This class implements the FIFO - or breadth first search - planning
# algorithm. It works by using a double ended queue: cells are pushed
# onto the back of the queue, and are popped from the front of the
# queue.


class ASTARPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.aStarQueue = PriorityQueue()
        self.temporaryQueue = PriorityQueue()
        # CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        # self.fifoQueue = deque()

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        self.aStarQueue.put((cell.pathCost, cell))
        # self.fifoQueue.append(cell)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.aStarQueue.empty()
        # return not self.fifoQueue

    # Simply pull from the front of the list1
    def popCellFromQueue(self):
        cell = self.aStarQueue.get()
        return cell[1]
        # cell = self.fifoQueue.popleft()
        # return cell

    def resolveDuplicate(self, nextCell, cell):
        if nextCell.pathCost > cell.pathCost + self.computeLStageAdditiveCost(nextCell, cell) + self.computeHeuristicEuclidian(nextCell):


            #nextCell.pathCost = cell.pathCost + self.computeLStageAdditiveCost(nextCell,cell) + self.computeHeuristicManhattan(nextCell) # Manhattan #

            nextCell.pathCost = cell.pathCost + self.computeLStageAdditiveCost(nextCell, cell) + self.computeHeuristicEuclidian(nextCell)  # Euclidian #

            #nextCell.pathCost = cell.pathCost + self.computeLStageAdditiveCost(nextCell, cell) + self.computeHeuristicOctile(nextCell) # Octile #

            # nextCell.pathCost = cell.pathCost + self.computeLStageAdditiveCost(nextCell, cell) + self.computeHeuristicConstant() # Constant #

            self.markCellAsVisitedAndRecordParent(nextCell, cell)
            self.numberOfCellsVisited = self.numberOfCellsVisited + 1

            while (self.aStarQueue.empty() == False):
                store = self.aStarQueue.get()
                if store[1].coords[0] == nextCell.coords[0] and store[1].coords[1] == nextCell.coords[1]:
                    self.temporaryQueue.put((nextCell.pathCost, nextCell))
                else:
                    self.temporaryQueue.put((store[0], store[1]))


            while (self.temporaryQueue.empty() == False):
                tempo = self.temporaryQueue.get()
                self.aStarQueue.put((tempo[0], tempo[1]))