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
        self.temporaryQueue = PriorityQueue()
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
            self.markCellAsVisitedAndRecordParent(nextCell, cell)
            self.numberOfCellsVisited = self.numberOfCellsVisited + 1

            while (self.greedyQueue.empty() == False):
                store = self.greedyQueue.get()
                # print('The store [0] element is : ' + str(store[0]) + ' and the store[1] element is ' + str(store[1]) )
                # print('The nextCell pathcost element is : ' + str(nextCell.pathCost) + ' and the nextcell element is ' + str(nextCell))
                if store[1].coords[0] == nextCell.coords[0] and store[1].coords[1] == nextCell.coords[1]:
                    self.temporaryQueue.put((nextCell.pathCost, nextCell))
                else:
                    self.temporaryQueue.put((store[0], store[1]))


            while (self.temporaryQueue.empty() == False):
                tempo = self.temporaryQueue.get()
                self.greedyQueue.put((tempo[0], tempo[1]))

            # print(self.numberOfCellsVisited)
            # print ' push cell onto queue is :'
            # print(nextCell)
            # print(self.pushCellOntoQueue(nextCell))

            # self.pushCellOntoQueue(nextCell)
