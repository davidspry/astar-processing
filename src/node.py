'''
A* Path Finder: Node
====================
'''
class Node:
	def __init__(self, x, y):
		self.x, self.y = x, y
		self.parent = None
		self.visited = False
		self.obstacle = False
		self.localcost = 10**10
		self.globalcost = 10**10
		self.neighbours = []
