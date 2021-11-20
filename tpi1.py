from tree_search import *
from cidades import *

class MyNode(SearchNode):
    def __init__(self,state,parent,depth=0,cost=0, heuristic=0):
        super().__init__(state,parent)

        # Initialize custom properties
        self.depth = depth # Node depth
        self.cost = cost # Cost from root to this node
        self.heuristic = heuristic # Heuristic cost from this node to goal
        self.eval = round(cost + heuristic) # A* eval function 
        self.children = None # Node children

class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',seed=0): 
        super().__init__(problem,strategy,seed)

        # Overwrite to use correct class
        root = MyNode(problem.initial, None, 
        depth=0,
        cost=0, 
        heuristic=problem.domain.heuristic(problem.initial, problem.goal))

        self.all_nodes = [root]

        self.solution_tree = None # Soltuion tree for depth_rand strategy
        self.used_shortcuts = None # Shortcuts 

    def astar_add_to_open(self,lnewnodes):
        # Sort by the A* evaluation function
        self.open_nodes = sorted(self.open_nodes + lnewnodes, key = lambda node: self.all_nodes[node].heuristic + self.all_nodes[node].cost)

    def propagate_eval_upwards(self,node):
        # Source https://github.com/miguelmatos-ua/ia/blob/master/tpi/tpi1/tpi1.py

        if node.children:
            node.eval = sorted([self.all_nodes[n].eval for n in node.children])[0]
            if node.parent is not None:
                self.propagate_eval_upwards(self.all_nodes[node.parent])

    def search2(self,atmostonce=False):  

        closed = {} # Close/visited nodes

        while self.open_nodes != []:     

            nodeID = self.open_nodes.pop(0)
            node = self.all_nodes[nodeID]                 # node.children = sorted(lnewnodes, key= lambda node: self.all_nodes[node].eval, reverse=True)

            if self.problem.goal_test(node.state):
                self.solution = node
                self.terminals = len(self.open_nodes)+1
                return self.get_path(node) 

            if atmostonce:
                closed[node.state] = node.cost # Initialize closed dict

            lnewnodes = []
            self.non_terminals += 1 

            for a in self.problem.domain.actions(node.state):                
                newstate = self.problem.domain.result(node.state,a)

                if atmostonce:                    
                    if newstate in closed: # Check if newstate has already been visited
                        
                        # If it has been visited, check if current path has lower cost. If so, update
                        if node.cost + self.problem.domain.cost(node.state, a) < closed[newstate]:
                             closed[newstate] = node.cost + self.problem.domain.cost(node.state, a)
                        else:
                            continue 
                    else:
                        # Add newstate to closed dict
                        closed[newstate] = node.cost + self.problem.domain.cost(node.state, a) 

                if newstate not in self.get_path(node):

                    newnode = MyNode(newstate,
                    nodeID,
                    depth=node.depth + 1,
                    cost=node.cost + self.problem.domain.cost(node.state, a),
                    heuristic=self.problem.domain.heuristic(newstate, self.problem.goal))
 
                    self.all_nodes.append(newnode)
                    lnewnodes.append(len(self.all_nodes)-1)

            # "Save" children
            if lnewnodes != []:
                node.children = lnewnodes

            self.propagate_eval_upwards(node)
    
            self.add_to_open(lnewnodes)

        return None

    def repeated_random_depth(self,numattempts=3,atmostonce=False):

        best_cost = None # Store best(i.e., lower) cost

        for i in range(numattempts):
            t = MyTree(self.problem, self.strategy, seed=i)
            t.search2(atmostonce=atmostonce)

            if t.solution.cost < best_cost if best_cost is not None else True:
                best_cost = t.solution.cost # Update best cost
                self.solution_tree = t # Update solution tree

        return self.solution_tree.get_path(self.solution_tree.solution)

    def make_shortcuts(self):
        # Reference: https://stackoverflow.com/questions/40438676/combinations-of-two-non-consecutive-items
        
        # Get a list of all connections from domain (without distance)
        connections = [(conn[0], conn[1]) for conn in self.problem.domain.connections]

        # Get the path to the solution
        solution_path = self.get_path(self.solution)

        shortcuts = []
        self.used_shortcuts = []

        # Iterate over a set (S_i  S_j) , where j − i > 1
        for i, c1 in enumerate(solution_path):
            for c2 in reversed(solution_path[i+2:]):

                if (c1, c2) in connections or (c2, c1) in connections: # A shortcut was found. Save it.
                    shortcuts.append((c1,c2))
                    break

        # Apply the found shortcuts to the solution_path, removing the unnecessary cities
        for c1, c2 in shortcuts:
            if c1 in solution_path and c2 in solution_path:

                self.used_shortcuts.append((c1, c2))
                del solution_path[solution_path.index(c1)+1:solution_path.index(c2)]

        return solution_path

class MyCities(Cidades):

    @property
    def average_branching_factor(self):
        # Compute average branching factor (approximated by the average number of neighbors)
        
        # Condensate all occurrences of all cities (in connections) into a single list
        all_connections = [x[0] for x in self.connections] + [x[1] for x in self.connections]

        # Build a dict with the cities and respective count
        neighbors = {}
        for city in self.coordinates.keys():   
            neighbors[city] = all_connections.count(city)

        # Return the mean
        return sum(neighbors.values()) / len(neighbors.values())

    def maximum_tree_size(self,depth):   # assuming there is no loop prevention
        b = self.average_branching_factor
        return round((b ** (depth + 1) - 1)/(b - 1))

        

    


