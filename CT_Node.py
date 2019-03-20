import copy
from Solution import *
from Conflict import *
from Low_Level import *
import pdb
'''
Attributes:
1.	Solutions – array with  solution for every agent 
2.	Total_cost – the cost of the solutions
3.	Conflicts – array of conflicts
4.	Parent – pointer to parent CTNode
Functions:
1.	Init CT_Node (Solutions, Total_cost, Conflicts, Parent)
2.	Find conflicts(CT_Node)
'''
class CT_Node:
    def __init__(self, solutions, totalCost, conflicts, parent):
        self.solutions = copy.deepcopy(solutions)
        self.totalCost = totalCost
        self.conflicts = copy.deepcopy(conflicts)
        if parent != None:
            self.parent = parent
        else: #root
            self.parent = None

    def print_solutions(self):
        for i in range(len(self.solutions)):
            self.solutions[i].print_Solutoin()

    def print_CT_Node(self):
        print("print CT Node:")
        self.print_solutions()
        print("total cost:{}, print conflicts:".format(self.totalCost,self.conflicts))
        if self.parent:
            print("\nprint parent node:")
            self.parent.print_CT_Node()
        else:
            print("CT_Node have no parent Node")

    def find_conflicts(self):
        for i in range(len(self.solutions)):
            for j in range(len(self.solutions)):
                if i!=j:
                    new_conflict = self.find_conflict_2path(self.solutions[i].path,self.solutions[j].path, self.solutions[i].agent_Id, self.solutions[j].agent_Id)
                    if new_conflict != None:
                        new_conflict = self.check_involved_agents(new_conflict, new_conflict.involved_Agents[0],new_conflict.involved_Agents[1])
                        return new_conflict
        print("no conflicts")
        return None

    def find_conflict_2path(self,path1,path2,agent1_id, agent2_id):
        minPathLen=min(len(path1),len(path2))
        for i in range(minPathLen):
            compare_States=self.compareStates(path1[i],path2[i])
            if compare_States:
                new_conflict=Conflict([agent1_id,agent2_id],path1[i][0],path1[i][1],path1[i][2])
                print('conflict found')
                return new_conflict
        return None

    def compareStates(self,state1,state2):
        if state1.i==state2.i and state1.j==state2.j and state1.time==state2.time:
            return True
        return False

    def check_involved_agents(self,new_conflict, agent1_id,agent2_id ):
        state = (new_conflict.i, new_conflict.j, new_conflict.time)
        for i in range(len(self.solutions)):
            if i != agent1_id and i != agent2_id:
                if state in self.solutions[i].path:
                    new_conflict.involved_Agents.append(self.solutions[i].agent_Id)
        return new_conflict


def test_CT_Node_find_conflicts():
    solutoin1 = Solutoin(0,[(1,2,0),(2,2,1),(1,2,0),(2,2,6)],1)
    solutoin2 = Solutoin(1,[(0,0,0)],2)
    solutoin3 = Solutoin(2, [(0,3,0),(2,2,1)], 2)

    CT_Node1=CT_Node([solutoin1,solutoin2,solutoin3],5,[],None)
    #CT_Node2=CT_Node([solutoin1,solutoin2],5,[],CT_Node1)
    CT_Node1.print_CT_Node()
    print('\n')
    new_conflict = CT_Node1.find_conflicts()
    if new_conflict!= None:
        new_conflict.print_conflict()

