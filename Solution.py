import copy
from Low_Level import *
'''
Attributes:
Agent id – identify number of the agent (of the thread)
Path – path to the goal
Cost – the cost of the path
'''

class Solutoin:
    def __init__(self, agent_Id, path, cost):
        self.agent_Id = agent_Id
        self.path =copy.deepcopy(path)
        self.cost = cost
    def print_Solutoin(self):
        msg= "Print solution: Agent_Id:{}, path cost: {}."  # 3 {} placeholders
        print(msg.format(self.agent_Id, self.cost))  # Pass 3 strings into method, separated by a comma
        for i in range(len(self.path)):
            self.path[i].print_State()
