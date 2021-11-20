from tree_search import *
from cidades import *

class MyNode(SearchNode):
    def __init__(self,state,parent,arg3=None,cost=0, heuristic=0):
        super().__init__(state,parent)

        self.cost = cost
        self.heuristic = heuristic
        self.eval = cost + heuristic
        self.children = None

class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',seed=0): 
        super().__init__(problem,strategy,seed)

        # Overwrite to use correct class
        root = MyNode(problem.initial, None, 
        cost=0, 
        heuristic=problem.domain.heuristic(problem.initial, problem.goal))

        self.all_nodes = [root]

    def astar_add_to_open(self,lnewnodes):
        
        self.open_nodes += lnewnodes

        self.open_nodes.sort(key=lambda  node: self.all_nodes[node].heuristic + self.all_nodes[node].cost)

    def propagate_eval_upwards(self,node):
        if node.children:
            node.eval = sorted([self.all_nodes[n].eval for n in node.children])[0]

            if node.parent:
                self.propagate_eval_upwards(self.all_nodes[node.parent])

    def search2(self,atmostonce=False):
        
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

                if newstate not in self.get_path(node):
                    newnode = MyNode(newstate,
                    nodeID,
                    cost=node.cost + self.problem.domain.cost(node.state, a),
                    heuristic=self.problem.domain.heuristic(newstate, self.problem.goal))
                    
                    self.all_nodes.append(newnode)
                    lnewnodes.append(len(self.all_nodes)-1)

            node.children = lnewnodes
            self.propagate_eval_upwards(node)

            self.add_to_open(lnewnodes)

        return None

    def repeated_random_depth(self,numattempts=3,atmostonce=False):
        #IMPLEMENT HERE
        pass

    def make_shortcuts(self):
        #IMPLEMENT HERE
        pass



class MyCities(Cidades):

    def maximum_tree_size(self,depth):   # assuming there is no loop prevention
        #IMPLEMENT HERE
        pass


