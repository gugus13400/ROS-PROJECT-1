# -*- coding: utf-8 -*-
from Queue import PriorityQueue

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque


# This class implements the FIFO - or breadth first search - planning
# algorithm. It works by using a double ended queue: cells are pushed
# onto the back of the queue, and are popped from the front of the
# queue.


class GREEDYPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.greedyQueue = PriorityQueue()
        # CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        # self.fifoQueue = deque()

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        self.greedyQueue.put((cell.pathCost, cell))
        # self.fifoQueue.append(cell)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.greedyQueue.empty()
        # return not self.fifoQueue

    # Simply pull from the front of the list1
    def popCellFromQueue(self):
        cell = self.greedyQueue.get()
        return cell[1]
        # cell = self.fifoQueue.popleft()
        # return cell

    def resolveDuplicate(self, nextCell, cell):
        if nextCell.pathCost > cell.pathCost + self.computeLStageAdditiveCost(nextCell, cell):
            # print 'NextCell path cost is '
            # print(nextCell.pathCost)
            # print 'cellpath cost + computeLadditive cost is '
            # print(cell.pathCost + self.computeLStageAdditiveCost(nextCell, cell))
            # self.popCellFromQueue()
            # print'pop cell from queue is'
            # print(self.popCellFromQueue())
            nextCell.pathCost = self.computeLStageAdditiveCost(nextCell, self.goal)
            nextCell.parent = cell
            self.numberOfCellsVisited = self.numberOfCellsVisited + 1
            # print(self.numberOfCellsVisited)
            # print ' push cell onto queue is :'
            # print(nextCell)
            # print(self.pushCellOntoQueue(nextCell))
            self.pushCellOntoQueue(nextCell)