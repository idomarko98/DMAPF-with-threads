import queue
import time
import math
from Msgs import *
import copy
from Solution import *
from CT_Node import *
import pdb
from Conflict import *
from Low_Level import *
from Conform_map import *
from Heuristic_Dijkstra import *

# global variables
# M- Number of agents
M = 2

q0= queue.PriorityQueue()
q1 = queue.PriorityQueue()
Counter_InitMsgs = 0
Counter_GoalMsgs= 0
Counter_NewNodeMsgs= 0
openListCounter = 0

class Agent:
    def __init__(self, agent_id, startpoint, goal ,map ,constrains,incumbentSolutionCost,incumbentSolution,map_cols,map_rows):
        # initiate  attributes
        self.agent_id=agent_id
        self.startpoint = startpoint
        self.goal= goal
        self.map=map
        self.constrains=constrains
        self.openList=queue.PriorityQueue()
        self.incumbentSolutionCost=incumbentSolutionCost
        self.incumbentSolution=incumbentSolution
        self.map_cols=map_cols
        self.map_rows=map_rows
        self.heuristicMap=create_Heuristic_map(map_rows,map_cols,map,(goal[0],goal[1]))
    # print Agent attributs for check
    def print_agent_attributs(self):
        print('agent id: {} start point:{} goal point:{} map:{} constrains:{} openList:{} incumbentSolutionCost:{} '
              'incumbentSolution:{}'
              .format(self.agent_id, self.startpoint, self.goal, self.map, self.constrains, self.openList,
                      self.incumbentSolutionCost, self.incumbentSolution))

'''
Initialization Step 1:
Find optimal path for yourself and send Init Msgs to all the agents
'''
def initialization_step_1(agent):
    print('\nagent{}: start Initialization step 1'.format(agent.agent_id))
    #-----------------   Initialization Step 1 ------------------
    #Find optimal path for yourself
    path, cost = find_optimal_path(agent.startpoint[0],agent.startpoint[1],
    agent.goal[0],agent.goal[1],agent.map,agent.heuristicMap,[],agent.map_cols, agent.map_rows)
    if path != []:
        print('cost: {} path:'.format(cost))
        print_path(path)
        #Create Init Msg and send it to all the agents
        create_Send_init_msgs(path,cost,agent.agent_id)
    print('agent{}: finished Initialization step 1'.format(agent.agent_id))

def print_path(path):
    for i in range(len(path)):
        path[i].print_State()
'''
Initialization Step 2:
check that agent received all the init msgs 
'''
def initialization_step_2(agent_id):
    print('agent{}: start Initialization step 2'.format(agent_id))
    q=int_to_queue(agent_id)
    if q.qsize() == M:
        print("agent{} received all the init Msgs".format(agent_id))
    else:
        print("Error: agent{} Not received all the init Msgs".format(agent_id))
    print('agent{}: finished Initialization step 2'.format(agent_id))


# Create CT.Root with the solution from the init_msgs  - return CT.root
def Create_CT_Root_for_agent_id(agent_id):
    solutions = []
    totalCost =0
    msgsQueue = int_to_queue(agent_id)
    while msgsQueue.empty() == False:
        incoming_Init_Msg = (msgsQueue.get())[2]
        # check if : Msg is init Msg with type = 1
        if incoming_Init_Msg.type == 1:
            newSolution = Solutoin(incoming_Init_Msg.source, incoming_Init_Msg.path, incoming_Init_Msg.cost)
            solutions.append(newSolution)
            totalCost = totalCost + incoming_Init_Msg.cost
        else:
            print("Error: unexpected message received")
            return -1
    CT_root = CT_Node(solutions, totalCost, [], None)
    return CT_root


#Create Init Msg and send it to all the agents
def create_Send_init_msgs(path,cost,agent_id):
    for i in range(M):
        new_init_Msg = Init_Msg(path, cost, agent_id, i)
        new_init_Msg.print_Msg()
        global Counter_InitMsgs
        if i == 0:
            Counter_InitMsgs=Counter_InitMsgs+1
            q0.put((1, Counter_InitMsgs, new_init_Msg))
            print('succsed to put')

        elif i == 1:
            Counter_InitMsgs = Counter_InitMsgs + 1
            q1.put((1, Counter_InitMsgs, new_init_Msg))
            print('succsed to put')


def print_Msg_queue(queueToprint,name):
    print('print {}'.format(name))
    for i in range(queueToprint.qsize()):
        (queueToprint.get()[2]).print_Msg()

def int_to_queue(agent_id):
    if agent_id == 0:
        return q0
    elif agent_id == 1:
        return q1

def checkMsgsQueues(q0,q1):
    if q0.empty() and q1.empty():
        print("No more msgs to handle...")
        return False
    else:
        print("there are more msgs to handle...")
        return True

def checkOpenLists(agents):
    for i in range(M):
        if agents[i].openList.empty() == False:
          print('more CT Nodes to handle in openList...')
          return True
    print("No more CT Nodes to handle in openList")
    return False

def handleNewMsg(newmsg,agent):
    if newmsg.type == 2:
        print('agent{}: start handle New Goal Msg'.format(agent.agent_id))
        handle_Goal_Msg(newmsg,agent)
        print('agent{}: finshed handle New Goal Msg'.format(agent.agent_id))
    elif newmsg.type == 3:
        print('agent{}: start handle NewCTNode_Msg'.format(agent.agent_id))
        handle_NewCTNode_Msg(newmsg, agent)
        print('agent{}: finished handle NewCTNode_Msg'.format(agent.agent_id))


def handle_Goal_Msg(goal_msg,agent):
    if goal_msg.solutionCost < agent.incumbentSolutionCost:
        # Update the best solution and the cost
        print('Update the new best solution and the cost:{}'.format(goal_msg.solutionCost))
        agent.incumbentSolutionCost = goal_msg.solutionCost
        agent.incumbentSolution = copy.deepcopy(goal_msg.CTNode_solution)
    else:
        print('The incumbent cost solution is better drop the goal message')


def handle_NewCTNode_Msg(newCTNode_msg,agent):
    constrains = copy.deepcopy(newCTNode_msg.CTNode.conflicts)
    constrains.append(newCTNode_msg.constrains)
    start_i=agent.startpoint[0]
    start_j=agent.startpoint[1]
    goal_i=agent.goal[0]
    goal_j=agent.goal[1]
    path, cost = find_optimal_path(start_i,start_j,goal_i,goal_j,agent.map,agent.heuristicMap
                                   ,constrains, agent.map_cols,agent.map_rows)
    print('print new path:')
    print_path(path)
    if path != None:
        # Calculate new solution cost:
        new_Total_cost = cost
        for i in range(M):
            if newCTNode_msg.CTNode.solutions[i].agent_Id != newCTNode_msg.destination:
                new_Total_cost =new_Total_cost + newCTNode_msg.CTNode.solutions[i].cost
        if new_Total_cost < agent.incumbentSolutionCost:
            #pdb.set_trace()
            new_Solution = copy.deepcopy(newCTNode_msg.CTNode.solutions)
            # To add: find index of solution of agent t.name
            new_Solution[agent.agent_id]=Solutoin(agent.agent_id, path, cost)
            #Create CTNodeChild
            CTNodeChild = CT_Node(new_Solution, new_Total_cost,constrains, newCTNode_msg.CTNode)
            global openListCounter
            openListCounter=openListCounter+1
            agent.openList.put((new_Total_cost,openListCounter, CTNodeChild))
            print('new CTNode child created and addedm to openlist')
        else:
            print('The incumbent solution cost is better than the new solution of NewCTNode_Msg, drop NewCTNode_Msg')
    print('Done')

def handleNewCT_Node(new_Node,agent):
    agent_id=agent.agent_id
    print('agent{}: start handle New CT_Node from openList'.format(agent_id))
    newConflict=new_Node.find_conflicts()
    if newConflict == None:
        #Create new Goal Msg - broadcast message to all the agents
        create_Send_goal_msgs(new_Node, agent_id)
        print('there is no conflict in new_Node - create Goal Msg and send to all the agents')
    else:
        print('there is new conflict:')
        newConflict.print_conflict()
        for i in range(len(newConflict.involved_Agents)):
            #Create NewCTNode_Msg
            CT_Node_msg=NewCTNode_Msg(new_Node,newConflict,agent_id,i)
            if i == agent_id:
                handle_NewCTNode_Msg(CT_Node_msg, agent)
            else:
                msg_q = int_to_queue(i)
                global Counter_NewNodeMsgs
                Counter_NewNodeMsgs=Counter_NewNodeMsgs+1
                msg_q.put((3,Counter_NewNodeMsgs,CT_Node_msg))

# Create goal Msg and send it to all the agents
def create_Send_goal_msgs(new_Node, agent_id):
    for i in range(M):
        new_goal_Msg = Goal_Msg(new_Node, new_Node.totalCost, agent_id, i)
        global Counter_GoalMsgs
        if i == 0:
            ounter_GoalMsgs = Counter_GoalMsgs + 1
            q0.put((1, Counter_GoalMsgs, new_goal_Msg))
            print('succsed to put')
        elif i == 1:
            Counter_GoalMsgs = Counter_GoalMsgs + 1
            q1.put((1, Counter_GoalMsgs, new_goal_Msg))
            print('succsed to put')
    print('agent{}: done handle New CT_Node'.format(agent_id))
    ########################test - insert NewCTNode_Msg to q0############

def test_inset_msg():
    goal_solutions = [Solutoin(0, [[0, 0], [1, 1]], 2), Solutoin(1, [[1, 0], [1, 0], [0, 1]], 3)]
    CT_goal = CT_Node(goal_solutions, 5, [], None)
    goal_Msg = Goal_Msg(CT_goal, 5, 1, 2)
    global Counter_GoalMsgs
    Counter_GoalMsgs = Counter_GoalMsgs + 1
    q0.put((2, Counter_GoalMsgs, goal_Msg))

    goal_solutions = [Solutoin(0, [[0, 0], [1, 1]], 2), Solutoin(1, [[1, 0], [1, 0], [0, 1]], 3)]
    CT_goal = CT_Node(goal_solutions, 5, [], None)
    goal_Msg = Goal_Msg(CT_goal, 5, 1, 2)
    Counter_GoalMsgs = Counter_GoalMsgs + 1
    q0.put((2, Counter_GoalMsgs, goal_Msg))

    CT_Node1 = CT_Node(goal_solutions, 5, [], None)
    constrains = Conflict([0, 1], 0, 0, 1)
    newCTNode_Msg = NewCTNode_Msg(CT_Node1, constrains, 1, 0)
    global Counter_NewNodeMsgs
    Counter_NewNodeMsgs = Counter_NewNodeMsgs + 1
    q0.put((3, Counter_NewNodeMsgs, newCTNode_Msg))


##########################Main############################

#Set the problem data
#map_rows,map_cols,map= txtfile_to_2D_map('map1_22X28.txt')
map4x4=[[1,0,0,1],
        [0,0,0,0],
        [0,0,0,0],
        [1,0,0,1]]

map3x4=[[0,1,1,0],
        [0,1,0,0],
        [0,0,0,0]]

map3x3=[[0,1,1],
        [0,0,1],
        [0,1,1]]
map_cols=4
map_rows=4
startpoints=[[1,0],[0,1]]
goals=[[2,3],[3,2]]
agents=[]

#initiate M agents
for i in range(M):
    new_agent=Agent(i,startpoints[i],goals[i],map4x4,[],math.inf,[],map_cols,map_rows)
    agents.append(new_agent)
    agents[i].print_agent_attributs()
'''
Initialization Step 1:
Find optimal path for yourself and send Init Msgs to all the agents
'''
for i in range(M):
    # create_Heuristic_map(map_rows, map_cols, map3X3,(goal_i,goal_j))
    initialization_step_1(agents[i])
'''
Initialization Step 2:
check that agent received all the init msgs 
'''
for i in range(M):
    initialization_step_2(agents[i].agent_id)

for i in range(M):
    CT_Root = Create_CT_Root_for_agent_id(agents[i].agent_id)
    print('\nPrint CT_root for agent{}'.format(i))
    CT_Root.print_CT_Node()
    openListCounter=openListCounter+1
    (agents[i].openList).put((CT_Root.totalCost,openListCounter,CT_Root))

# Main Process(Agent Ai)
msgsQueues=checkMsgsQueues(q0,q1)
openListsCT_nodes=checkOpenLists(agents)
while msgsQueues or openListsCT_nodes:
    '''Handle a new CTNode from OpenSet
    Handle Incoming Messages'''
    for i in range(M):
        print('the turn of agent{} is START'.format(agents[i].agent_id))
        #Handle Incoming Messages
        q=int_to_queue(agents[i].agent_id)
        if q.empty() == False:
            new_msg=q.get()
            handleNewMsg(new_msg[2],agents[i])

        #Handle a new CTNode from OpenList
        if (agents[i].openList).empty() == False:
            new_Node=(agents[i].openList).get()
            #openList is Priority Queue - it pops the lowest cost every time
            if new_Node[0] < agents[i].incumbentSolutionCost:
                handleNewCT_Node(new_Node[2],agents[i])
            else:
                print('all the Nodes in the open list are more expensive than incumbentSolutionCost - pop them all')
                while(agents[i].openList).empty() == False:
                    (agents[i].openList).get()

        print('the turn of agent{} is OVER'.format(agents[i].agent_id))

    msgsQueues = checkMsgsQueues(q0, q1)
    openListsCT_nodes = checkOpenLists(agents)
#pdb.set_trace()
for i in range(M):
    print('Agent{} final solution cost:{}'.format(i, agents[i].incumbentSolutionCost))
    agents[i].incumbentSolution.print_solutions()
