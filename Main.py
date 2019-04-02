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
from Tests import *
import sys

#-------------Global variables-------------#

# M- Number of agents
M = 0
# 0 - doesn't print comments 1- print comments
Print_flag=0
# q0= queue.PriorityQueue()
# q1 = queue.PriorityQueue()
#list of Priority Queues msgs
MsgsQueues=[]


Counter_InitMsgs = 0
Counter_GoalMsgs= 0
Counter_NewNodeMsgs= 0
openListCounter = 0
RoundRobin_Iterations=0
Counter_expand_Nodes=0

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
    if Print_flag==1:
        print('\nagent{}: start Initialization step 1'.format(agent.agent_id))
        #-----------------   Initialization Step 1 ------------------
        #Find optimal path for yourself
    path, cost = find_optimal_path(agent.startpoint[0],agent.startpoint[1],
    agent.goal[0],agent.goal[1],agent.map,agent.heuristicMap,[],agent.map_cols, agent.map_rows,Print_flag)
    if path != []:
        if Print_flag==1:
            print('cost: {} path:'.format(cost))
            print_path(path)
        #Create Init Msg and send it to all the agents
        create_Send_init_msgs(path,cost,agent.agent_id)
        if Print_flag==1:
            print('agent{}: finished Initialization step 1'.format(agent.agent_id))
        return 1
    else:
        return -1

def print_path(path):
    for i in range(len(path)):
        path[i].print_State()
'''
Initialization Step 2:
check that agent received all the init msgs 
'''
def initialization_step_2(agent_id):
    if Print_flag==1:
        print('agent{}: start Initialization step 2'.format(agent_id))
    q=int_to_queue(agent_id)
    if q.qsize() == M:
        if Print_flag==1:
            print("agent{} received all the init Msgs".format(agent_id))
    else:
        if Print_flag==1:
            print("Error: agent{} Not received all the init Msgs".format(agent_id))
    if Print_flag == 1:
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
        if Print_flag==1:
            new_init_Msg.print_Msg()
        global Counter_InitMsgs
        Counter_InitMsgs=Counter_InitMsgs+1
        MsgsQueues[i].put((1, Counter_InitMsgs, new_init_Msg))
        if Print_flag==1:
            print('succsed to put')



def print_Msg_queue(queueToprint,name):
    print('print {}'.format(name))
    for i in range(queueToprint.qsize()):
        (queueToprint.get()[2]).print_Msg()

def int_to_queue(agent_id):
    return MsgsQueues[agent_id]

#check if there are more msgs to handle
def checkMsgsQueues(MsgsQueues):
    for i in range(len(MsgsQueues)):
        if MsgsQueues[i].empty() == False:
            if Print_flag == 1:
                print("there are more msgs to handle...")
            return True
    if Print_flag == 1:
        print("No more msgs to handle...")
    return False

#check if there are more CT_Nodes to handle
def checkOpenLists(agents):
    for i in range(M):
        if agents[i].openList.empty() == False:
            if Print_flag == 1:
                print('more CT Nodes to handle in openList...')
            return True
    if Print_flag == 1:
        print("No more CT Nodes to handle in openList")
    return False

def handleNewMsg(newmsg,agent):
    if newmsg.type == 2:
        if Print_flag == 1:
            print('agent{}: start handle New Goal Msg'.format(agent.agent_id))
        handle_Goal_Msg(newmsg,agent)
        if Print_flag == 1:
            print('agent{}: finshed handle New Goal Msg'.format(agent.agent_id))
    elif newmsg.type == 3:
        if Print_flag == 1:
            print('agent{}: start handle NewCTNode_Msg'.format(agent.agent_id))
        handle_NewCTNode_Msg(newmsg, agent)
        if Print_flag == 1:
            print('agent{}: finished handle NewCTNode_Msg'.format(agent.agent_id))


def handle_Goal_Msg(goal_msg,agent):
    if goal_msg.solutionCost < agent.incumbentSolutionCost:
        # Update the best solution and the cost
        if Print_flag == 1:
            print('Update the new best solution and the cost:{}'.format(goal_msg.solutionCost))
        agent.incumbentSolutionCost = goal_msg.solutionCost
        agent.incumbentSolution = copy.deepcopy(goal_msg.CTNode_solution)
    else:
        if Print_flag == 1:
            print('The incumbent cost solution is better drop the goal message')


def handle_NewCTNode_Msg(newCTNode_msg,agent):
    constrains = copy.deepcopy(newCTNode_msg.CTNode.conflicts)
    constrains.append(newCTNode_msg.constrains)
    start_i=agent.startpoint[0]
    start_j=agent.startpoint[1]
    goal_i=agent.goal[0]
    goal_j=agent.goal[1]
    path, cost = find_optimal_path(start_i,start_j,goal_i,goal_j,agent.map,agent.heuristicMap
                                   ,constrains, agent.map_cols,agent.map_rows,Print_flag)
    if Print_flag == 1:
        print('print new path:')
        print_path(path)
    if path != []:
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
            global Counter_expand_Nodes
            Counter_expand_Nodes = Counter_expand_Nodes + 1
            if Print_flag == 1:
                print('new CTNode child created and addedm to openlist')
        else:
            if Print_flag == 1:
                print('The incumbent solution cost is better than the new solution of NewCTNode_Msg, drop NewCTNode_Msg')
    if Print_flag == 1:
        print('Done')

def handleNewCT_Node(new_Node,agent):
    agent_id=agent.agent_id
    if Print_flag == 1:
        print('agent{}: start handle New CT_Node from openList'.format(agent_id))
    newConflict=new_Node.find_conflicts()
    if newConflict == None:
        #Create new Goal Msg - broadcast message to all the agents
        create_Send_goal_msgs(new_Node, agent_id)
        if Print_flag == 1:
            print('there is no conflict in new_Node - create Goal Msg and send to all the agents')
    else:
        if Print_flag == 1:
            print('there is new conflict:')
            newConflict.print_conflict()
        for i in range(M):
            #Create NewCTNode_Msg
            CT_Node_msg=NewCTNode_Msg(new_Node,newConflict,agent_id,i)
            if i == agent_id:
                handle_NewCTNode_Msg(CT_Node_msg, agent)
            elif i in newConflict.involved_Agents:
                msg_q = int_to_queue(i)
                global Counter_NewNodeMsgs
                Counter_NewNodeMsgs=Counter_NewNodeMsgs+1
                msg_q.put((3,Counter_NewNodeMsgs,CT_Node_msg))

# Create goal Msg and send it to all the agents
def create_Send_goal_msgs(new_Node, agent_id):
    for i in range(M):
        new_goal_Msg = Goal_Msg(new_Node, new_Node.totalCost, agent_id, i)
        global Counter_GoalMsgs
        Counter_GoalMsgs = Counter_GoalMsgs + 1
        MsgsQueues[i].put((1, Counter_GoalMsgs, new_goal_Msg))
        if Print_flag == 1:
            print('succsed to put')
    if Print_flag == 1:
        print('agent{}: done handle New CT_Node'.format(agent_id))



#Set the problem data
agents=[]
test_map,map_cols, map_rows, M, startpoints, goals =test_radom32X32_10()
start = time.time()
for i in range(M):
    MsgsQueues.append(queue.PriorityQueue())
#initiate M agents
for i in range(M):
    new_agent=Agent(i,startpoints[i],goals[i],test_map,[],math.inf,[],map_cols,map_rows)
    agents.append(new_agent)
    if Print_flag == 1:
        agents[i].print_agent_attributs()

'''
Initialization Step 1:
Find optimal path for yourself and send Init Msgs to all the agents
'''
for i in range(M):
    check_path=initialization_step_1(agents[i])
    if check_path ==-1:
        print('exit')
        sys.exit("there is no solution")
'''
Initialization Step 2:
check that agent received all the init msgs 
'''
for i in range(M):
    initialization_step_2(agents[i].agent_id)


for i in range(M):
    CT_Root = Create_CT_Root_for_agent_id(agents[i].agent_id)
    if Print_flag == 1:
        print('\nPrint CT_root for agent{}'.format(i))
        CT_Root.print_CT_Node()
    openListCounter=openListCounter+1
    (agents[i].openList).put((CT_Root.totalCost,openListCounter,CT_Root))

# Main Process(Agent Ai)
msgsQueues=checkMsgsQueues(MsgsQueues)
openListsCT_nodes=checkOpenLists(agents)
while msgsQueues or openListsCT_nodes:
    '''Handle a new CTNode from OpenSet
    Handle Incoming Messages'''
    RoundRobin_Iterations=RoundRobin_Iterations+1
    for i in range(M):
        if Print_flag == 1:
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
                if Print_flag == 1:
                    print('all the Nodes in the open list are more expensive than incumbentSolutionCost - pop them all')
                while(agents[i].openList).empty() == False:
                    (agents[i].openList).get()
        if Print_flag == 1:
            print('the turn of agent{} is OVER'.format(agents[i].agent_id))

    msgsQueues = checkMsgsQueues(MsgsQueues)
    openListsCT_nodes = checkOpenLists(agents)
end = time.time()
total_msgs=Counter_InitMsgs+Counter_GoalMsgs+Counter_NewNodeMsgs

###results##
print('Agent{} final solution cost:{}'.format(0, agents[0].incumbentSolutionCost))
agents[0].incumbentSolution.print_solutions()
print('Msgs-Counter:')
print('Counter_InitMsgs:{}\nCounter_GoalMsgs:{}\nCounter_NewNodeMsgs:{}'.format(Counter_InitMsgs,Counter_GoalMsgs,Counter_NewNodeMsgs))
print('Total number of Msgs:{}'.format(total_msgs))
print('RoundRobin_Iterations:{}\nCounter expanded Nodes:{}'.format(RoundRobin_Iterations,Counter_expand_Nodes))
print('time taken: {}'.format(end - start))

df=add_new_result_to_cvs('random-32-32-10.map-10.scen','random-32-32-10',M,agents[0].incumbentSolutionCost,end - start,Counter_InitMsgs,Counter_NewNodeMsgs,Counter_GoalMsgs,total_msgs,Counter_expand_Nodes,RoundRobin_Iterations)
new_line(df)
