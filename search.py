# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
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
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    # Implement the stack
    stack = util.Stack()
    visited = {}

    # Push the start state onto the stack
    start = problem.getStartState()
    stack.push((start, []))

    # While there are states still left to expand
    while not stack.isEmpty():

        # Pop the most recent state off the stack and visit it
        state, path = stack.pop()
        visited[state] = True

        # If the state is the goal return the path
        if problem.isGoalState(state):
            return path

        # Otherwise expand the state
        successors = problem.getSuccessors(state)

        # Add its neighbors onto the stack, while updating the path
        for s in successors:
            if s[0] not in visited:
                stack.push((s[0], path + [s[1]]))

    # No path found, return an empty path
    return []

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # Implement the queue
    queue = util.Queue()
    visited = []

    # Push the start state onto the queue
    start = problem.getStartState()
    queue.push((start, []))

    # While there are states still left to expand
    while not queue.isEmpty():

        # Pop the next state off the queue and visit it
        state, path = queue.pop()
        visited.append(state)

        # If the state is the goal return the path
        if problem.isGoalState(state):
            return path

        # Otherwise expand the state
        successors = problem.getSuccessors(state)

        # Add its neighors onto the queue, while updating the path if the node hasn't already been visited and its not in queue already.
        for s in successors:
            if s[0] not in visited and s[0] not in (state[0] for state in queue.list):
                queue.push((s[0], path + [s[1]]))

    # No path found, return an empty path
    return []



def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # Implement the priority queue
    pqueue = util.PriorityQueue()
    visited = {}

    # Store the path
    path = []

    # Push the start state onto the queue with priority of 0
    start = problem.getStartState()
    pqueue.push((start,[]), 0)

    # While there are states still left to expand
    while not pqueue.isEmpty():

        # Pop the next state off the queue and visit it
        state, path = pqueue.pop()
        visited[state] = True

        # If the state is the goal return the path
        if problem.isGoalState(state):
            return path
        
        # Otherwise expand the state
        successors = problem.getSuccessors(state)

        # Add its neighors onto the queue
        for s in successors:

            # If the state isn't visited and doesn't exist in the queue
            if s[0] not in visited and s[0] not in (state[2][0] for state in pqueue.heap):

                # Add it to the path, get the cost of the state, and push it onto the queue
                newPath = path + [s[1]]
                pri = problem.getCostOfActions(newPath)
                pqueue.push((s[0], newPath), pri)

            # Else if the state isn't visited but exists in the queue
            elif s[0] not in visited and s[0] in (state[2][0] for state in pqueue.heap):

                # Find the state in the queue
                for hstate in pqueue.heap:
                    if hstate[2][0] == s[0]:

                        # If the old cost is greater than the new cost push the state onto the queue with the new cost
                        oldPri = problem.getCostOfActions(hstate[2][1])
                        newPri = problem.getCostOfActions(path + [s[1]])
                        if oldPri > newPri:
                            pqueue.push((s[0], (path + [s[1]])), s[2])

    # No path found, return an empty path
    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    # Implement the priority queue
    pqueue = util.PriorityQueue()
    visited = []

    # Store the path here
    path = []

    # Push the start state onto the queue with priority 0 + the cost of the heuristic
    start = problem.getStartState()
    pqueue.push((start,[]), 0 + heuristic(start, problem))

    # While there are states still left to expand
    while not pqueue.isEmpty():

        # Pop the next state off the queue and visit it
        state, path = pqueue.pop()
        visited.append(state)

        # If the state is the goal return the path
        if problem.isGoalState(state):
            return path
        
        # Otherwise expand the state
        successors = problem.getSuccessors(state)

        # Add its neighors onto the queue
        for s in successors:

            # If the state isn't visited and doesn't exist in the queue
            if s[0] not in visited and s[0] not in (state[2][0] for state in pqueue.heap):
                newPath = path + [s[1]]
                pri = problem.getCostOfActions(newPath)
                pqueue.push((s[0], newPath), pri + heuristic(s[0], problem))

            # Else if the state isn't visited but exists in the queue
            elif s[0] not in visited and s[0] in (state[2][0] for state in pqueue.heap):
                for hstate in pqueue.heap:
                    if hstate[2][0] == s[0]:

                        # If the old cost is greater than the new cost push the state onto the queue with the new cost
                        oldPri = problem.getCostOfActions(hstate[2][1])
                        newPri = problem.getCostOfActions(path + [s[1]])
                        if oldPri > newPri:
                            pqueue.push((s[0], (path + [s[1]])), newPri + heuristic(s[0], problem))

    # No path found, return an empty path
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
