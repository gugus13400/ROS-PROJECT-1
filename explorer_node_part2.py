import rospy
from Queue import PriorityQueue
from explorer_node_base import ExplorerNodeBase
from bresenham import bresenham
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose2D
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi, degrees
import math
from comp0037_reactive_planner_controller.controller_base import ControllerBase
# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)

        self.blackList = []
	self.checkForDuplicateFrontiers = []
	self.listOfFrontiers = PriorityQueue()
	self.weightPriorityQueue = PriorityQueue()
	
	self.transferQueue = PriorityQueue()
	
	self.numberOfWaypoints = 0
	
	self.currentOdometrySubscriber = rospy.Subscriber('/robot0/odom', Odometry, self.odometryCallback)
	self.currentRobotPose = Pose2D()

	
    def updateFrontiers(self):
	pass

    def popFrontierFromQueue(self):
	frontier = self.listOfFrontiers.get()
	return frontier[1]

    def popWeightedFrontierFromQueue(self):
	weightedFrontier = self.weightPriorityQueue.get()
	return weightedFrontier[1]
    
    def pushFrontierOnToQueue(self, frontierLength, frontierList):
    	self.listOfFrontiers.put((frontierLength * (-1), frontierList))

    def pushWeightedFrontierOnToQueue(self, Weight, frontierList):
    	self.weightPriorityQueue.put((Weight, frontierList))
    
    def pushTransfer(self, frontierLength, frontierList):
    	self.transferQueue.put((frontierLength * (-1), frontierList))

    def popTransfer(self):
	frontier = self.transferQueue.get()
	return frontier[1]
    
    def findingMiddleCellOfAFrontier(self, frontierList):
	print('The initial plan was to go to that cell:')
	print(frontierList[0])
	xTotal = 0
	yTotal = 0
	averageX = 0
	averageY = 0
	
	thresholdPythagoras = float('inf')

	totalNumberOfElementInFrontierList = len(frontierList)
	for cells in frontierList:
		xTotal += cells[0]
		yTotal += cells[1]
	averageX = xTotal / totalNumberOfElementInFrontierList
	averageY = yTotal/ totalNumberOfElementInFrontierList
	
	for cellInList in frontierList:
		if sqrt((cellInList[0] - averageX)**2 + (cellInList[1] - averageY)**2) <  thresholdPythagoras:
			theNewCellIs = (cellInList[0], cellInList[1])
			thresholdPythagoras = sqrt((cellInList[0] - averageX)**2 + (cellInList[1] - averageY)**2)
	print('Now it is choosing that one: ')
	print(theNewCellIs)
	return theNewCellIs
	
 		
    def angleToTurnWeight(self, robotAngle, robotX, robotY, cellToGo):
	deltaX = cellToGo[0] - robotX
	deltaY = cellToGo[1] - robotY
	angleToCellToGo = atan2(deltaY, deltaX)
	
	if (deltaX < 0 and deltaY > 0) or (deltaX < 0 and deltaY < 0):
		angleToCellToGo += math.pi
	
	delta = angleToCellToGo - robotAngle
	if delta < -math.pi:
		delta += 2.0 * math.pi
	elif delta > math.pi:
		delta -= 2.0 * math.pi

	print('THEDIFFERENCE IN ANGLE IS')
	print(math.degrees(delta))
	#dx1 = currentCell[0] - cellToGo[0]
	#dy1 = currentCell[0] - cellToGo[0]
	#dx2 = previousCell[0] - currentCell[0]
	#dy2 = previousCell[0] - currentCell[0]
	#angle = abs(dx1 * dx2 - dx2 * dy1)
	return math.degrees(delta)

    def distanceToFrontierWeight(self, robotPose, cellToGo):
	dX = cellToGo[0] - robotPose[0]
	dY = cellToGo[1] - robotPose[1]
	distance = sqrt(dX * dX + dY * dY)
	print('THE DISTANCE IS')
	print(distance)
	return distance

    def updateTheCheckFrontierList(self, deletedFrontier):
	#print('THE DELETED FRONTIER IS:')
	#print(deletedFrontier)
	#print('the alreadyCheckedFrontierlist is :')
	#print(self.checkForDuplicateFrontiers)
	#for alreadyCheckedFrontierCells in self.checkForDuplicateFrontiers :
	#	print('the alreadyCheckedFrontierCells is :')
	#	print(alreadyCheckedFrontierCells)
		#if deletedFrontier == alreadyCheckedFrontierCells:
			#	self.checkForDuplicateFrontiers.remove(alreadyCheckedFrontierCells)
	#			print('the alreadyCheckedFrontierlist is noow:')
	#			print(self.checkForDuplicateFrontiers)
	pass

    def checkForDuplicate(self, frontierToCheck):
	elem_to_find = frontierToCheck
	res1 = any(elem_to_find in sublist for sublist in self.checkForDuplicateFrontiers)
	return res1
   
    def odometryCallback(self, odometry):
        odometryPose = odometry.pose.pose

        pose = Pose2D()

        position = odometryPose.position
        orientation = odometryPose.orientation
        
        pose.x = position.x
        pose.y = position.y
        pose.theta = 2 * atan2(orientation.z, orientation.w)
	self.currentRobotPose = pose


    def updateFrontiersCell(self, nextCandidate):
	self.neighbouringFrontierCellsList = []
	self.alreadyCheckedFrontiersList = []
	self.neighbouringFrontierCellsList.append(nextCandidate)
	
	while (len(self.neighbouringFrontierCellsList) != 0):
		nextFrontierCandidate = self.neighbouringFrontierCellsList[0]
		#print('NeighbouringFrontierCellsList before pop')
		#print(self.neighbouringFrontierCellsList)
		self.neighbouringFrontierCellsList.pop(0)
		#print('NeighbouringFrontierCellsList after pop')
		#print(self.neighbouringFrontierCellsList)
		#print('There is next candidate')
		#print(nextFrontierCandidate)
		self.alreadyCheckedFrontiersList.append(nextFrontierCandidate)
		#print('the already checked list is after appending nextfrontiercandidate')
		#print(self.alreadyCheckedFrontiersList)
		
		for cells in self.getNextSetOfCellsToBeVisited(nextFrontierCandidate):
			if self.isFrontierCell(cells[0], cells[1]) == True and cells not in self.alreadyCheckedFrontiersList and cells not in self.neighbouringFrontierCellsList:
				#print('the cell appending is')
				#print(cells)
				self.neighbouringFrontierCellsList.append(cells)

	if self.checkForDuplicate(self.alreadyCheckedFrontiersList[0]) == False:
		self.pushFrontierOnToQueue(len(self.alreadyCheckedFrontiersList), self.alreadyCheckedFrontiersList)

		self.checkForDuplicateFrontiers.append(self.alreadyCheckedFrontiersList)	
	
	
    def chooseNewDestination(self):
	self.temporaryTransfer = []
	

	robotX = self.currentRobotPose.x
	robotY = self.currentRobotPose.y
	robotAngle = self.currentRobotPose.theta
	robotPose = (robotX, robotY)
	
        candidateGood = False
        destination = None
      	
        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                candidate = (x, y)
		
		if self.occupancyGrid.getCell(candidate[0], candidate[1]) == 1 and candidate not in self.blackList:
			self.blackList.append(candidate)
		if self.isFrontierCell(x, y) == True:
			candidateGood = True
			for k in range(0, len(self.blackList)):
				if self.blackList[k] == candidate:
					candidateGood = False
					break
			
			if candidateGood is True:
				self.updateFrontiersCell(candidate)

	while self.listOfFrontiers.empty() == False:
		take = self.popFrontierFromQueue() 
		self.temporaryTransfer.append(take)
		self.pushTransfer(len(take), take)
	
	print('The Initial List of Frontiers is :')
	print(self.temporaryTransfer)
	print(len(self.temporaryTransfer))
	self.temporaryTransfer = []
	
	while self.transferQueue.empty() == False:
		take = self.popTransfer()
		self.pushFrontierOnToQueue(len(take), take)
	

	while self.listOfFrontiers.empty() == False:
		candidateFrontier = self.popFrontierFromQueue() 
		nextCellToGo = self.findingMiddleCellOfAFrontier(candidateFrontier)
		length = len(candidateFrontier)
		angle = self.angleToTurnWeight(robotAngle, robotX, robotY, nextCellToGo)
		distance = self.distanceToFrontierWeight(robotPose, nextCellToGo)
		Weight = -1.5 * angle + 3 * distance + 4 * length
		self.pushWeightedFrontierOnToQueue(Weight * (-1), candidateFrontier)
	i = 0

	while self.weightPriorityQueue.empty() == False:
		isItTheHighestWeightedFrontier = self.popWeightedFrontierFromQueue()
		if i == 0:
			candidateGood = True
			destination = self.findingMiddleCellOfAFrontier(isItTheHighestWeightedFrontier)
			#self.updateTheCheckFrontierList(isItTheHighestWeightedFrontier)
			i += 1
		else:
			self.pushFrontierOnToQueue(len(isItTheHighestWeightedFrontier), isItTheHighestWeightedFrontier)
		
	while self.listOfFrontiers.empty() == False:
		take = self.popFrontierFromQueue() 
		self.temporaryTransfer.append(take)
		self.pushTransfer(len(take), take)
	
	print('The Final list of Frontiers is :')
	print(self.temporaryTransfer)
	print(len(self.temporaryTransfer))
	
	while self.transferQueue.empty() == False:
		take = self.popTransfer()
		self.pushFrontierOnToQueue(len(take), take)
		
      #lalalla

	print('The number of waypoints visited is:')
	print(self.numberOfWaypoints)
	
	self.checkForDuplicateFrontiers = []
	#for element in self.checkForDuplicateFrontiers:
		#del element
	while self.listOfFrontiers.empty() == False:
		caca = self.popFrontierFromQueue()

        return candidateGood, destination

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)

    def getNextSetOfCellsToBeVisited(self, cell):

        # self stores the set of valid actions / cells
        cells = list();

        # Go through all the neighbours and add the cells if they
        # don't fall outside the grid and they aren't the cell we
        # started with. The order has been manually written down to
        # create a spiral.
        self.pushBackCandidateCellIfValid(cell, cells, 0, -1)
        self.pushBackCandidateCellIfValid(cell, cells, 1, -1)
        self.pushBackCandidateCellIfValid(cell, cells, 1, 0)
        self.pushBackCandidateCellIfValid(cell, cells, 1, 1)
        self.pushBackCandidateCellIfValid(cell, cells, 0, 1)
        self.pushBackCandidateCellIfValid(cell, cells, -1, 1)
        self.pushBackCandidateCellIfValid(cell, cells, -1, 0)
        self.pushBackCandidateCellIfValid(cell, cells, -1, -1)

        return cells

    # This helper method checks if the robot, at cell.coords, can move
    # to cell.coords+(offsetX, offsetY). Reasons why it can't do this
    # include falling off the edge of the map or running into an
    # obstacle.
    def pushBackCandidateCellIfValid(self, cell, cells, offsetX, offsetY):
        newX = cell[0] + offsetX
        newY = cell[1] + offsetY
        extent = self.occupancyGrid.getExtentInCells()
        if ((newX >= 0) & (newX < extent[0]) \
            & (newY >= 0) & (newY < extent[1])):
            newCell = (newX, newY)
            cells.append(newCell)

 	#DO THIS FOR EVERY FRONTIER

	#candidateFrontier1 = self.popFrontierFromQueue() 
	#candidateFrontier2 = self.popFrontierFromQueue()
	
	#nextCellToGo1 = self.fidingMiddleCellOfAFrontier(candidateFrontier1)
	#nextCellToGo2 = self.fidingMiddleCellOfAFrontier(candidateFrontier2)

	#angle1 = self.angleToTurnWeight(robotAngle, robotX, robotY, nextCellToGo1)
	#angle2 = self.angleToTurnWeight(robotAngle, robotX, robotY, nextCellToGo2)

	#distance1 = self.distanceToFrontierWeight(robotPose, nextCellToGo1)
	#distance2 = self.distanceToFrontierWeight(robotPose, nextCellToGo2)

	#Weight1 = -1 * angle1 + 100 * distance1 
	#Weight2 = -1 * angle2 + 100 * distance2
	
	#if Weight1 > Weight2:
		#candidateGood = True
		#destination = nextCellToGo1
		#destination = self.fidingMiddleCellOfAFrontier(candidateFrontier1)
		#self.pushFrontierOnToQueue(len(candidateFrontier2), candidateFrontier2) #maybe compute updateFrontiersCell with candidate = to nextCellToGo1
		#print('I CHOOSE THE FIRST FRONTIER')
		#self.numberOfWaypoints += 1

	#elif len(candidateFrontier1) == 0 and len(candidateFrontier2) == 0 :
		#print('We have explored all the map')
		#return False
	#else:
		#candidateGood = True
		#destination = nextCellToGo2
		#destination = self.fidingMiddleCellOfAFrontier(candidateFrontier2)
		#self.pushFrontierOnToQueue(len(candidateFrontier1), candidateFrontier1)
		#print('I CHOOSE THE SECOND FRONTIER')
		#self.numberOfWaypoints += 1

	
	
	#if len(candidateFrontier) == 0:
	#	print('We have explored all the map')
	#	return False
	#else:
	#	candidateGood = True
	#	destination = candidateFrontier[0]
	#	self.numberOfWaypoints += 1

        # If we got a good candidate, use it
            
