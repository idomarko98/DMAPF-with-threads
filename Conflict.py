import copy

'''
Attributes:
1.	Involved_Agents –list of  the Agent Ids that involved in the conflict   (threads name)
2.	position –[i,j] position of the conflict in the map    
3.	time – time the conflict accrue 
Functions:
3.	Init function (Involved_Agents, position, time)
'''
class Conflict:
    def __init__(self, involved_Agents, i,j , time):
        self.involved_Agents = copy.deepcopy(involved_Agents)
        self.i = i
        self.j = j
        self.time = time
    def print_conflict(self):
        print('conflict: position:[{},{}], time:{}, between agents:{}'.format(self.i,self.j,self.time,self.involved_Agents))

