# berkeley-ai-searchEating
 Berkeley AI Project 1 : Search

 In this project I implement various search algorithms:

  - Depth First Search
  - Breadth First Search
  - Uniform Cost Search
  - A* Search

As well as utilize these search algorithms to solve various problems within the game of pacman.

  ## This project is separated into various questions.
 - [Overview](#overview)
  - [Q1: Depth First Search](#Q1:-Depth-First-Search)
  - [Q2: Breadth First Search](#Q2:-Breadth-First-Search)
  - [Q3: Uniform Cost Search](#Q3:-Uniform-Cost-Search)
  - [Q4: A* Search](#Q4:-A*-Search)
  - [Q5: Corners Problem: Representation](#Q5:-Corners-Problem:-Representation)
  - [Q6: Corners Problem: Heuristic](#Q6:-Corners-Problem:-Heuristic)
  - [Q7: Eating All The Dots: Heuristic](#Q7:-Eating-All-The-Dots:-Heuristic)
  - [Q8: Suboptimal Search](#Q8:-Suboptimal-Search)
- [My process](#my-process)
  - [Built with](#built-with)
  - [What I learned](#what-i-learned)
- [Author](#author)


## Overview

  ### Q1: Depth First Search

    Implement the depth-first search (DFS) algorithm in the depthFirstSearch function in search.py. To make your algorithm complete, write the graph search version of DFS, which avoids expanding any already visited states.

    In this problem I had to implement the DFS algorithm to solve pacman tinyMaze, mediumMaze, and bigMaze.

    ```python
    
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
    stack = util.Stack()
    visited = {}
    start = problem.getStartState()
    stack.push((start, []))

    while not stack.isEmpty():
        state, path = stack.pop()
        visited[state] = True

        if problem.isGoalState(state):
            return path

        successors = problem.getSuccessors(state)

        for s in successors:
            if s[0] not in visited:
                stack.push((s[0], path + [s[1]]))

    return []
    
    ```

  ### Q2: Breadth First Search
  ### Q3: Uniform Cost Search
  ### Q4: A* Search
  ### Q5: Corners Problem: Representation
  ### Q6: Corners Problem: Heuristic
  ### Q7: Eating All The Dots: Heuristic
  ### Q8: Suboptimal Search
