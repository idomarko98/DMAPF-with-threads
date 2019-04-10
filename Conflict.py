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

class Conflict_Edges:
    def __init__(self, involved_Agents, v1,v2, start_time,end_time):
        self.involved_Agents = copy.deepcopy(involved_Agents)
        self.v1 = v1
        self.v2 = v2
        self.start_time = start_time
        self.end_time = end_time
    def print_Edge_conflict(self):
        print('conflict: Edge:[v1:({},{}),v2:({},{})], time interval:{}-{}, between agents:{}'.format(self.v1.i,self.v1.j,self.v2.i,self.v2.j,self.end_time,self.start_time,self.involved_Agents))

class Vertex:
    def __init__(self, i,j):
        self.i = i
        self.j = j
    def print_conflict(self):
        print('Vertex position:[{},{}]'.format(self.i,self.j))

