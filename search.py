# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).
  
  You do not need to change anything in this class, ever.
  """
  
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first [p 74].
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.18].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"

  # list of visited nodes
  visited = []

  # list of path to goal
  p = []

  # append to stack
  fringe = util.Stack()

  # inserts starting node and path (p) into fringe stack
  fringe.push((problem.getStartState(), p))

  # check if starting state is the goal state
  if problem.isGoalState(problem.getStartState()):
      fringe.pop()

  # Implementation of GRAPH-SEARCH:
  while not fringe.isEmpty():

      # remove node from stack to explore neighbours, update path
      current, p = fringe.pop()

      # marks the position as visited
      visited.append(current)

      # check if the goal state is acheived
      if problem.isGoalState(current):
          return p

      # looping through possible successors of a given current state
      for childNode in problem.getSuccessors(current):

          # checks if neighboring states are explored
          if childNode[0] not in visited:

              # push in the successor returned from method
              fringe.push((childNode[0], p + [childNode[1]]))

  # if while loop ends without return value then no path exists
  return []


def breadthFirstSearch(problem):
  "Search the shallowest nodes in the search tree first. [p 74]"
  "*** YOUR CODE HERE ***"

  visited = []
  p = []
  fringe = util.Queue()
  fringe.push((problem.getStartState(), p))

  if problem.isGoalState(problem.getStartState()):
      fringe.pop()

  while not fringe.isEmpty():
    current, p = fringe.pop()

    if current not in visited:
      visited.append(current)

      if problem.isGoalState(current):
          return p

      for childNode in problem.getSuccessors(current):
        if childNode[0] not in visited:
          if childNode[0] not in (item[0] for item in fringe.list):
            fringe.push((childNode[0], p + [childNode[1]]))

  return []
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"

  visited = []
  p = []
  fringe = util.PriorityQueue()
  # insert starting node into fringe along with the path
  fringe.push((problem.getStartState(), p), problem.getCostOfActions(p))

  if problem.isGoalState(problem.getStartState()):
      fringe.pop()

  while not fringe.isEmpty():
    current, p = fringe.pop()

    if current not in visited:
      visited.append(current)

      if problem.isGoalState(current):
          return p

      for childNode in problem.getSuccessors(current):
        if childNode[0] not in visited:
          fringe.push((childNode[0], p + [childNode[1]]), problem.getCostOfActions(p + [childNode[1]]))

  return []

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  
  frontier = util.PriorityQueue()
  return dataStructureSearch(problem, frontier, heuristic)

def dataStructureSearch(problem, frontier, heuristic=nullHeuristic):
  explored = []
  node = problem.getStartState()
  path = []
  cost = 0 + heuristic(node, problem)
  frontier.push((node, path), cost)
  if problem.isGoalState(node):
    return []
  while not frontier.isEmpty():
    node, path = frontier.pop()
    successors = problem.getSuccessors(node)
    for successor in successors:
      child = successor[0]
      childPath = []
      childPath.extend(path)
      childPath.append(successor[1])
      childCost = problem.getCostOfActions(path) + successor[2] + heuristic(child, problem)
      if child not in explored:
        if problem.isGoalState(child):
          return childPath
        frontier.push((child, childPath), childCost)
        explored.append(child)

  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch