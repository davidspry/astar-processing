'''
A* path-finding algorithm visualisation for Processing.py
=========================================================
Note:		This program for Processing.py is based on an implementation of the A* algorithm by David Barr. 
		(https://youtu.be/icZj67PTFhc)

Author:		David Spry

Controls:	Mouse LMB -> Place an obstacle
		Mouse RMB -> Remove an obstacle
		Mouse LMB + ⌘ -> Nominate a destination node
		Mouse LMB + L-Shift -> Nominate a source node
			
Global variables:
		a: tuple, z: tuple -> The locations of the path's endpoints.
		nodes: list -> A 2D list of Nodes
		path: list -> An ordered list of Nodes constituting a path
		scale: int -> An positive integer used to determine the size and position of each node.
'''

import math
import node

a, z = (), ()
nodes, path = [], []
scale = 25

def grid_array(scale):
	'''
	Create a two-dimensional list to represent a grid. \
	The dimensions of the list are defined by the size of \
	screen, defined in size(), and the value of the parameter \
	'scale'.
	
	:parameter scale: The size of each location in the grid.
	'''
	rows = width // scale
	cols = height // scale
	array = [[None] * rows for col in range(cols)]
	return array

def get_node(index):
	'''
	Return a reference to the node/s represented in 'index', where \
	'index' is either a tuple signifying a single position (r, c), \
	or a list containing multiple tuples, each signifying a single \
	position (r, c).
	
	:parameter index: A tuple or a list of tuples, each signifying \
	a position (r, c).
	:returns: In the case that 'index' is a tuple, a single reference \
	to a node is returned. In the case that 'index' a list of tuples, \
	a list of nodes is returned.
	'''
	if type(index) == list:
		return [nodes[t[0]][t[1]] for t in index]
	elif type(index) == tuple:
		return nodes[index[0]][index[1]]
	else: return None

def reset_nodes():
	'''
	Reset each node to the default state.
	'''
	for row in nodes:
		for node in row:
			node.visited = False
			node.localcost = 10**10
			node.globalcost = 10**10
			node.parent = None
	
def initialise_nodes():
	'''
	Create a 2D list of Nodes. Initialise the global variables 'a' & 'z', \
	which store the locations of the path's endpoints.
	'''
	global nodes, a, z
	h, w, sz = height, width, scale * 0.5
	a = (h / scale // 2, 5)
	z = (h / scale // 2, w // scale - 5)
	nodes = grid_array(scale)
	for r, y in enumerate(range(0, h, scale)):
		for c, x in enumerate(range(0, w, scale)):
			nodes[r][c] = node.Node(x + sz, y + sz)

def draw_nodes():
	'''
	Draw each node to the screen as a square centred \
	at the point (node.x, node.y).
	'''
	stroke(235)
	strokeWeight(1)
	for y, row in enumerate(nodes):
		for x, node in enumerate(row):
			if node.obstacle: fill(225)	
			elif (y,x) == a: fill(85, 185, 65)	
			elif (y,x) == z: fill(225, 95, 85)	
			elif node.visited: fill(25, 115, 185)
			else: fill(25, 65, 125, 55)
			rect(node.x, node.y, scale, scale)

def in_range(r, c, rm, cm):
	'''
	Determine whether the position (r+rm, c+cm) is valid or not.
	'''
	rows, cols = len(nodes), len(nodes[0])
	is_neighbour = r * c != r + c
	r_in_range = r + rm > -1 and r + rm < rows
	c_in_range = c + cm > -1 and c + cm < cols
	return is_neighbour and r_in_range and c_in_range

def find_neighbours():
	'''
	For each node in the list of nodes, find the neighbouring \
	nodes and append references to each neighbour to the list of \
	neighbours that's associated with each node.
	'''
	for r in range(len(nodes)):
		for c in range(len(nodes[r])):
			append_neighbours(r, c)	
	
def append_neighbours(row, col):
	'''
	Search the adjacent locations in the N, S, E, & W directions \
	from the given position (row, col). For each node that is found, \
	append a reference to the node to the list of neighbours associated \
	with the source node, whose location is (row, col).
	
	In order that paths can move diagonally, neighbours in the directions \
	of NW, NE, SE, & SW must be available to the source node. To achieve \
	this, remove the condition: abs(row) != abs(col) below.
	
	:parameter row: The row of the source node.
	:parameter col: The column of the source node.
	'''
	for r in range(-1, 2):
		for c in range(-1, 2):
			if abs(r) != abs(c) and in_range(r, c, row, col):
				nodes[row][col].neighbours += [nodes[row+r][col+c]]

def assess_neighbours(node):
	'''
	For each neighbour in a node's neighbours list, reassess the cost \
	and add the node referenced in the parameter 'node' as the parent \
	if the cost is updated.
	
	:parameter node: A reference to the node whose neighbours should be updated.
	:returns: A list of neighbour nodes with updated costs and parents.
	'''
	neighbours = []
	for neighbour in node.neighbours:
		if not neighbour.obstacle and not neighbour.visited:
			neighbours += [neighbour]
			new_cost = node.localcost + euclidean_distance(node, neighbour)
			if new_cost < neighbour.localcost: update_cost(neighbour, node, new_cost)
	return neighbours

def update_cost(node, parent, cost):
	'''
	Update the parent, local cost, and global cost of the node referenced in 'node'.
	
	Note: The global variable 'z', which contains the position of the path's \
	destination endpoint as a tuple, is used as a means to glean a reference to \
	the path's endpoint node from get_node().
	
	:parameter node: The node to update.
	:parameter parent: The parent node of 'node'.
	:parameter cost: The new local cost.
	'''
	global z
	node.parent = parent
	node.localcost = cost
	node.globalcost = node.localcost + euclidean_distance(node, get_node(z))

def euclidean_distance(a, b):
	'''
	Compute the difference between two nodes, 'a' and 'b'.
	
	:parameter a: The source node.
	:parameter b: The destination node.
	'''
	d = (a.x - b.x) ** 2 + (a.y - b.y) ** 2
	return math.sqrt(d)

def modify_path(r, c, make, key):
	'''
	Modify the path in order to add or remove an obstacle, or move the endpoints of the path.
	
	Note: The constant 'keyPressed' is provided by the Processing.py API. It contains a \
	Boolean value indicating whether a keyboard key is currently being pressed.
	
	:parameter r: The row component of the selected node.
	:parameter c: The column component of the selected node.
	:parameter make: An int representing whether the user's mouse's LMB or RMB has been pressed.
	:parameter key: An int representing the most recently pressed keyboard key.
	'''
	global nodes, a, z
	if keyPressed and not get_node((c, r)).obstacle:
		if key == 16: a = (c, r)
		elif key == 157: z = (c, r)
	else: 
		if (c, r) not in [a, z] and in_range(c, r, 0, 0):
			nodes[c][r].obstacle = make == 37
	find_path()
	
def find_path():
	'''
	Build a path between the nominated path endpoints.
	
	1. Reset all nodes to the default state, create references to the
	endpoint nodes 'start' and 'end', and update the cost of 'start'.
	Construct a list of nodes, 'boundary', beginning with 'start'.
	
	2. While there are unvisited nodes in 'boundary':
		> Sort 'boundary' by its nodes' global cost in descending order.
		> Remove any visited elements from the end of the list.
		> Select the unvisited node with the smallest global cost by popping.
		> Set the selected node to visited;
		> Update the selected node's neighbours' costs and parents in assess_neighbours();
		> Add the updated neighbours to 'boundary'.

	3. Build a path from 'end'.
	'''
	reset_nodes()

	start, end = get_node([a,z])
	start.localcost = 0.0
	start.globalcost = euclidean_distance(start, end)
	
	current = start
	boundary = [current]
	
	while boundary:
		boundary.sort(key = lambda n: n.globalcost, reverse=True)
		while boundary and boundary[-1].visited: boundary.pop()
		if len(boundary) == 0: break
		
		current = boundary.pop()
		current.visited = True
		boundary += assess_neighbours(current)

	build_path(end)

def build_path(node):
	'''
	Beginning from the node referenced in 'node', build a list of nodes \
	that follows the sequence of node.parent connections until a node \
	without a parent is found.
	
	:parameter node: The source node, from which the path is built.
	'''
	global path
	path = []
	while node:
		path += [node]
		node = node.parent
	
def draw_path():
	'''
	For all but the last node in the path, draw a line from the \
	node's position (node.x, node.y) to its subsequent neighbour.
	'''
	n = len(path)
	stroke(255, 215, 25)
	strokeWeight(3)
	for x in range(n-1):
		a, b = path[x], path[x+1]
		line(a.x, a.y, b.x, b.y)

def setup():
	size(800,800,P3D)
	rectMode(CENTER)
	smooth(8)
	
	frame.setTitle("A* Path Finder")
	initialise_nodes()
	find_neighbours()
	
def draw():
	background(0)
	draw_nodes()
	draw_path()

def mousePressed():
	'''
	Determine which node the cursor is pointing to by \
	dividing the components of the cursor's screen coordinates \
	by the global variable 'scale'. Send the most recently \
	pressed mouse button and keyboard key codes with the computed \
	node location to modify_path().
	'''
	x = mouseX // scale
	y = mouseY // scale
	modify_path(x, y, mouseButton, keyCode)
	
def mouseDragged():
	'''
	Determine which node the cursor is pointing to by \
	dividing the components of the cursor's screen coordinates \
	by the global variable 'scale'. Send the most recently \
	pressed mouse button and keyboard key codes with the computed \
	node location to modify_path().
	'''
	x = mouseX // scale
	y = mouseY // scale
	modify_path(x, y, mouseButton, keyCode)
