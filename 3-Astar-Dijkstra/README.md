Project 03: Discrete Planning

Code submitted by Mohammad Anas Imam Khan (C17566828) and Ashit Mohanty (C13582787)

1. astar.py:Code for A* Algorithm. 
RESULTS:

cost = [1,1,1], Giving equal weightage to right, front and left actions respectively.
['-', '-', '-', '-', '-', '-']
['-', '-', '-', '-', '-', '-']
['*', 'F', 'F', 'L', '-', '-']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

Expanded Nodes
(4,3,0)
(3,3,0)
(2,3,0)
(2,2,1)
(2,1,1)
(2,0,1)

cost = [1, 1, 10], Penalizing left(then forward) action

['-', '-', '-', 'R', 'F', 'R']
['-', '-', '-', 'F', '-', 'F']
['*', 'F', 'F', 'F', 'F', 'R']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

Expanded Nodes
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)
(2, 5, 3)
(0, 3, 0)
(0, 4, 3)
(0, 5, 3)
(1, 5, 2)
(2, 5, 2)
(2, 4, 1)
(2, 3, 1)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

cost = [10, 1, 1]: Penalizing right(then forward) action of the car
['-', '-', '-', '-', '-', '-']
['-', '-', '-', '-', '-', '-']
['*', 'F', 'F', 'L', '-', '-']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

Expanded Nodes
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)


2. dijkstra.py: Code using the Dijkstra's Algorithm. The difference here is that the heuristic array will be filled with zeroes the code was run. Due to this change, dijkstra expands more nodes than A* to reach the goal (2,0,1).
 
RESULTS:

cost = [1, 1, 1]:
['-', '-', '-', '-', '-', '-']
['-', '-', '-', '-', '-', '-']
['*', 'F', 'F', 'L', '-', '-']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

Expanded Nodes
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)
(2, 2, 1)
(2, 5, 3)
(0, 3, 0)
(2, 1, 1)
(1, 5, 0)
(0, 4, 3)
(2, 0, 1)

cost = [1, 1, 10]:
['-', '-', '-', 'R', 'F', 'R']
['-', '-', '-', 'F', '-', 'F']
['*', 'F', 'F', 'F', 'F', 'R']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

Expanded Nodes
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)
(2, 5, 3)
(0, 3, 0)
(0, 4, 3)
(0, 5, 3)
(1, 5, 2)
(2, 5, 2)
(2, 4, 1)
(2, 3, 1)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

cost = [10, 1, 1]:
['-', '-', '-', '-', '-', '-']
['-', '-', '-', '-', '-', '-']
['*', 'F', 'F', 'L', '-', '-']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

Expanded Nodes
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(1, 3, 0)
(2, 2, 1)
(0, 3, 0)
(2, 1, 1)
(2, 0, 1)