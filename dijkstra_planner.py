# -*- coding: utf-8 -*-
from Queue import PriorityQueue

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque


# This class implements the FIFO - or breadth first search - planning
# algorithm. It works by using a double ended queue: cells are pushed
# onto the back of the queue, and are popped from the front of the
# queue.


class DIJKSTRAPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.dijkstraQueue = PriorityQueue()
        self.temporaryQueue = PriorityQueue()
        # CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        # self.fifoQueue = deque()

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        self.dijkstraQueue.put((cell.pathCost, cell))
        # self.fifoQueue.append(cell)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.dijkstraQueue.empty()
        # return not self.fifoQueue

    # Simply pull from the front of the list1
    def popCellFromQueue(self):
        cell = self.dijkstraQueue.get()
        return cell[1]
        # cell = self.fifoQueue.popleft()
        # return cell

    def resolveDuplicate(self, nextCell, cell):
        if nextCell.pathCost > cell.pathCost + self.computeLStageAdditiveCost(nextCell, cell):

            nextCell.pathCost = cell.pathCost + self.computeLStageAdditiveCost(nextCell, cell)  #compute the distance between the current 'cell' and the 'nextcell'.
            self.markCellAsVisitedAndRecordParent(nextCell, cell)
            # nextCell.parent = cell
            self.numberOfCellsVisited = self.numberOfCellsVisited + 1

            while (self.dijkstraQueue.empty() == False):
                store = self.dijkstraQueue.get()
                # print('The store [0] element is : ' + str(store[0]) + ' and the store[1] element is ' + str(store[1]) )
                # print('The nextCell pathcost element is : ' + str(nextCell.pathCost) + ' and the nextcell element is ' + str(nextCell))
                if store[1].coords[0] != nextCell.coords[0]:
                   self.temporaryQueue.put((store[0], store[1]))
                elif store[1].coords[0] == nextCell.coords[0] and store[1].coords[1] == nextCell.coords[1]:
                    self.temporaryQueue.put((nextCell.pathCost, nextCell))

            while (self.temporaryQueue.empty() == False):
                tempo = self.temporaryQueue.get()
                self.dijkstraQueue.put((tempo[0], tempo[1]))

            # print(' push cell onto queue is :')
            # print(nextCell)
            # print(self.pushCellOntoQueue(nextCell))
            # self.pushCellOntoQueue(nextCell)
