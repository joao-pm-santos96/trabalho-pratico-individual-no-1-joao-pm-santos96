from tree_search import *
from cidades import *

class MyNode(SearchNode):
    def __init__(self,state,parent,depth=0,cost=0, heuristic=0):
        super().__init__(state,parent)

        self.depth = depth
        self.cost = cost
        self.heuristic = heuristic
        self.eval = round(cost + heuristic)
        self.children = None

class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',seed=0): 
        super().__init__(problem,strategy,seed)

        # Overwrite to use correct class
        root = MyNode(problem.initial, None, 
        depth=0,
        cost=0, 
        heuristic=problem.domain.heuristic(problem.initial, problem.goal))

        self.all_nodes = [root]
        self.closed_nodes = None

        self.solution_tree = None
        self.used_shortcuts = None

    def astar_add_to_open(self,lnewnodes):
        self.open_nodes = sorted(self.open_nodes + lnewnodes, key = lambda node: self.all_nodes[node].heuristic + self.all_nodes[node].cost)

    def propagate_eval_upwards(self,node):
        
        if node.children:
            node.eval = sorted([self.all_nodes[n].eval for n in node.children])[0]

            if node.parent is not None:
                self.propagate_eval_upwards(self.all_nodes[node.parent])


    def search2(self,atmostonce=False):  

        closed = {}   

        while self.open_nodes != []:     

            nodeID = self.open_nodes.pop(0)
            node = self.all_nodes[nodeID] 

            if self.problem.goal_test(node.state):
                self.solution = node
                self.terminals = len(self.open_nodes)+1
                return self.get_path(node) 

            lnewnodes = []
            self.non_terminals += 1 

            for a in self.problem.domain.actions(node.state):                
                newstate = self.problem.domain.result(node.state,a)

                if atmostonce:                    
                    if newstate in closed:
                        if node.cost + self.problem.domain.cost(node.state, a) < closed[newstate]:
                             closed[newstate] = node.cost + self.problem.domain.cost(node.state, a)
                        else:
                            continue 
                    else:
                        closed[newstate] = node.cost + self.problem.domain.cost(node.state, a) 

                if newstate not in self.get_path(node):

                    newnode = MyNode(newstate,
                    nodeID,
                    depth=node.depth + 1,
                    cost=node.cost + self.problem.domain.cost(node.state, a),
                    heuristic=self.problem.domain.heuristic(newstate, self.problem.goal))
 
                    self.all_nodes.append(newnode)
                    lnewnodes.append(len(self.all_nodes)-1)

            if lnewnodes != []:
                node.children = lnewnodes

            self.propagate_eval_upwards(node)

            self.add_to_open(lnewnodes)

        return None

    def repeated_random_depth(self,numattempts=3,atmostonce=False):

        best_cost = None

        for i in range(numattempts):
            t = MyTree(self.problem, self.strategy, seed=i)
            tree = t.search2(atmostonce=atmostonce)

            if t.solution.cost < best_cost if best_cost is not None else True:
                self.solution_tree = t
                best_cost = t.solution.cost

        return self.solution_tree.get_path(self.solution_tree.solution)

    def make_shortcuts(self):
        # Reference: https://stackoverflow.com/questions/40438676/combinations-of-two-non-consecutive-items
        
        connections = [(conn[0], conn[1]) for conn in self.problem.domain.connections]

        solution_path = self.get_path(self.solution)
        possibilities = [(a, b) for i, a in enumerate(solution_path) for b in reversed(solution_path[i+2:])]

        shortcuts = []
        self.used_shortcuts = []
        for i, c1 in enumerate(solution_path):
            for c2 in reversed(solution_path[i+2:]):

                if (c1, c2) in connections or (c2, c1) in connections:
                    shortcuts.append((c1,c2))
                    break


        for c1, c2 in shortcuts:
            if c1 in solution_path and c2 in solution_path:

                self.used_shortcuts.append((c1, c2))
                del solution_path[solution_path.index(c1)+1:solution_path.index(c2)]


        return solution_path

class MyCities(Cidades):

    @property
    def average_branching_factor(self):

        neighbors = {}
        all_connections = [x[0] for x in self.connections] + [x[1] for x in self.connections]

        for city in self.coordinates.keys():   
            neighbors[city] = all_connections.count(city)

        return sum(neighbors.values()) / len(neighbors.values())

    def maximum_tree_size(self,depth):   # assuming there is no loop prevention
        b = self.average_branching_factor
        return round((b ** (depth + 1) - 1)/(b - 1))

        

    


