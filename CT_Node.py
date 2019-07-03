from Low_Level import *

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
        else:  # root
            self.parent = None

    def print_solutions(self):
        for i in range(len(self.solutions)):
            self.solutions[i].print_Solution()
            print()

    def print_CT_Node(self):
        print("print CT Node:")
        self.print_solutions()
        print("total cost:{}, print conflicts:".format(self.totalCost, self.conflicts))
        if self.parent:
            print("\nprint parent node:")
            self.parent.print_CT_Node()
        else:
            print("CT_Node have no parent Node")

    def find_conflicts(self):
        for i in range(len(self.solutions)):
            for j in range(len(self.solutions)):
                if i != j:
                    new_conflict = self.find_conflict_2path(self.solutions[i].path, self.solutions[j].path,
                                                            self.solutions[i].agent_Id, self.solutions[j].agent_Id)
                    if new_conflict != None:
                        new_conflict = self.check_involved_agents(new_conflict, new_conflict.involved_Agents[0],
                                                                  new_conflict.involved_Agents[1])
                        return new_conflict
        # print("no conflicts")
        return None

    def find_Edges_conflicts(self):
        for i in range(len(self.solutions)):
            for j in range(len(self.solutions)):
                if i != j:
                    new_Edge_conflict = self.find_Edge_conflict_2path(self.solutions[i].path, self.solutions[j].path,
                                                                      self.solutions[i].agent_Id,
                                                                      self.solutions[j].agent_Id)
                    if new_Edge_conflict != None:
                        return new_Edge_conflict
        # print("no conflicts")
        return None

    def find_conflict_2path(self, path1, path2, agent1_id, agent2_id):
        minPathLen = min(len(path1), len(path2))
        maxPathLen = max(len(path1), len(path2))
        if len(path1) > len(path2):
            final_state_i, final_state_j, path_num = path2[len(path2) - 1].i, path2[len(path2) - 1].j, 1
        else:
            final_state_i, final_state_j, path_num = path1[len(path1) - 1].i, path1[len(path1) - 1].j, 2
        for i in range(maxPathLen):
            if i < minPathLen - 1:
                compare_States = self.compareStates(path1[i], path2[i])
                if compare_States:
                    new_conflict = Conflict([agent1_id, agent2_id], path1[i].i, path1[i].j, path1[i].time)
                    return new_conflict
            else:
                if path_num == 1:
                    if final_state_i == path1[i].i and final_state_j == path1[i].j:
                        new_conflict = Conflict([agent1_id, agent2_id], path1[i].i, path1[i].j, path1[i].time)
                        return new_conflict
                if path_num == 2:
                    if path_num == 1:
                        if final_state_i == path2[i].i and final_state_j == path2[i].j:
                            new_conflict = Conflict([agent1_id, agent2_id], path2[i].i, path2[i].j, path2[i].time)
                            return new_conflict
        return None

    def compareStates(self, state1, state2):
        if state1.i == state2.i and state1.j == state2.j and state1.time == state2.time:
            return True
        return False

    def compare_Edges(self, v1_t1, v1_t2, v2_t1, v2_t2):
        if v1_t1.i == v2_t2.i and v1_t1.j == v2_t2.j and v1_t2.i == v2_t1.i and v1_t2.j == v2_t1.j:
            return True
        return False

    def check_involved_agents(self, new_conflict, agent1_id, agent2_id):
        state = (new_conflict.i, new_conflict.j, new_conflict.time)
        for i in range(len(self.solutions)):
            if i != agent1_id and i != agent2_id:
                if state in self.solutions[i].path:
                    new_conflict.involved_Agents.append(self.solutions[i].agent_Id)
        return new_conflict

    def find_Edge_conflict_2path(self, path1, path2, agent1_id, agent2_id):
        minPathLen = min(len(path1), len(path2))
        for t in range(minPathLen - 1):
            compare_Edges = self.compare_Edges(path1[t], path1[t + 1], path2[t], path2[t + 1])
            if compare_Edges == True:
                v1 = Vertex(path1[t].i, path1[t].j)
                v2 = Vertex(path1[t + 1].i, path1[t + 1].j)
                new_Edge_conflict = Conflict_Edges([agent1_id, agent2_id], v1, v2, t, t + 1)
                return new_Edge_conflict
        return None
