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
#with threads
# 0 - doesn't print comments 1- print comments
Print_flag=0

class Counters:
    def __init__(self):
        self.Counter_InitMsgs = 0
        self.Counter_GoalMsgs= 0
        self.Counter_NewNodeMsgs= 0
        self.openListCounter = 0
        self.RoundRobin_Iterations=0
        self.Counter_expand_Nodes=0

class Agent:
    def __init__(self, agent_id, startpoint, goal ,map ,constrains,incumbentSolutionCost,incumbentSolution,map_cols,map_rows,MsgsQueues):
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
        self.MsgsQueues=MsgsQueues
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
def initialization_step_1(agent,counters,M):
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
        create_Send_init_msgs(path,cost,agent.agent_id,M,counters,agent)
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
def initialization_step_2(agent_id,M,agents):
    if Print_flag==1:
        print('agent{}: start Initialization step 2'.format(agent_id))
    q=agents[agent_id].MsgsQueues[agent_id]
    if q.qsize() == M:
        if Print_flag==1:
            print("agent{} received all the init Msgs".format(agent_id))
    else:
        if Print_flag==1:
            print("Error: agent{} Not received all the init Msgs".format(agent_id))
    if Print_flag == 1:
        print('agent{}: finished Initialization step 2'.format(agent_id))


# Create CT.Root with the solution from the init_msgs  - return CT.root
def Create_CT_Root_for_agent_id(agent_id,agent):
    solutions = []
    totalCost =0
    msgsQueue = agent.MsgsQueues[agent_id]
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
def create_Send_init_msgs(path,cost,agent_id, M ,counters,agent):
    for i in range(M):
        new_init_Msg = Init_Msg(path, cost, agent_id, i)
        if Print_flag==1:
            new_init_Msg.print_Msg()
        counters.Counter_InitMsgs=counters.Counter_InitMsgs+1
        agent.MsgsQueues[i].put((1, counters.Counter_InitMsgs, new_init_Msg))
        if Print_flag==1:
            print('succsed to put')



def print_Msg_queue(queueToprint,name):
    print('print {}'.format(name))
    for i in range(queueToprint.qsize()):
        (queueToprint.get()[2]).print_Msg()

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
def checkOpenLists(agents,M):
    for i in range(M):
        if agents[i].openList.empty() == False:
            if Print_flag == 1:
                print('more CT Nodes to handle in openList...')
            return True
    if Print_flag == 1:
        print("No more CT Nodes to handle in openList")
    return False

def handleNewMsg(newmsg,agent,counters,M):
    if newmsg.type == 2:
        if Print_flag == 1:
            print('agent{}: start handle New Goal Msg'.format(agent.agent_id))
        handle_Goal_Msg(newmsg,agent)
        if Print_flag == 1:
            print('agent{}: finshed handle New Goal Msg'.format(agent.agent_id))
    elif newmsg.type == 3:
        if Print_flag == 1:
            print('agent{}: start handle NewCTNode_Msg'.format(agent.agent_id))
        handle_NewCTNode_Msg(newmsg,agent, M, counters)
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


def handle_NewCTNode_Msg(newCTNode_msg, agent, M, counters):
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
            counters.openListCounter=counters.openListCounter+1
            agent.openList.put((new_Total_cost,counters.openListCounter, CTNodeChild))
            counters.Counter_expand_Nodes = counters.Counter_expand_Nodes + 1
            if Print_flag == 1:
                print('new CTNode child created and addedm to openlist')
        else:
            if Print_flag == 1:
                print('The incumbent solution cost is better than the new solution of NewCTNode_Msg, drop NewCTNode_Msg')
    if Print_flag == 1:
        print('Done')

def handleNewCT_Node(new_Node,agent,M,counters):
    agent_id=agent.agent_id
    if Print_flag == 1:
        print('agent{}: start handle New CT_Node from openList'.format(agent_id))
    newConflict=new_Node.find_conflicts()
    if newConflict == None:
        newEdgeConflict = new_Node.find_Edges_conflicts()
        if newEdgeConflict == None:
            #Create new Goal Msg - broadcast message to all the agents
            create_Send_goal_msgs(new_Node, agent_id,M,counters,agent)
            if Print_flag == 1:
                print('there is no conflict in new_Node - create Goal Msg and send to all the agents')
        else: #new Edge conflict
            if Print_flag == 1:
                print('there is new Edge conflict:')
                newEdgeConflict.print_Edge_conflict()
            for i in range(M):
                # Create NewCTNode_Msg
                CT_Node_msg = NewCTNode_Msg(new_Node, newEdgeConflict, agent_id, i)
                if i in newEdgeConflict.involved_Agents:
                    if i == agent_id:
                        handle_NewCTNode_Msg(CT_Node_msg, agent, M, counters)
                    else:
                        counters.Counter_NewNodeMsgs = counters.Counter_NewNodeMsgs + 1
                        agent.MsgsQueues[i].put((3, counters.Counter_NewNodeMsgs, CT_Node_msg))
    else: #new conflict
        if Print_flag == 1:
            print('there is new conflict:')
            newConflict.print_conflict()
        for i in range(M):
            # Create NewCTNode_Msg
            CT_Node_msg = NewCTNode_Msg(new_Node, newConflict, agent_id, i)
            if i in newConflict.involved_Agents:
                if i == agent_id:
                    handle_NewCTNode_Msg(CT_Node_msg,agent, M, counters)
                else:
                    msg_q = agent.MsgsQueues[i]
                    counters.Counter_NewNodeMsgs=counters.Counter_NewNodeMsgs+1
                    msg_q.put((3,counters.Counter_NewNodeMsgs,CT_Node_msg))

# Create goal Msg and send it to all the agents
def create_Send_goal_msgs(new_Node, agent_id,M,counters,agent):
    for i in range(M):
        new_goal_Msg = Goal_Msg(new_Node, new_Node.totalCost, agent_id, i)
        counters.Counter_GoalMsgs =counters.Counter_GoalMsgs + 1
        agent.MsgsQueues[i].put((1, counters.Counter_GoalMsgs, new_goal_Msg))
        if Print_flag == 1:
            print('succsed to put')
    if Print_flag == 1:
        print('agent{}: done handle New CT_Node'.format(agent_id))

def initialization_step_1_M_agents(agents,M,counters):
    '''
    Initialization Step 1:
    Find optimal path for yourself and send Init Msgs to all the agents
    '''
    for i in range(M):
        check_path=initialization_step_1(agents[i],counters,M)
        if check_path ==-1:
            print('exit')
            sys.exit("there is no solution")
'''
Initialization Step 2:
check that agent received all the init msgs 
'''
def initialization_step_2_M_agents(agents,M):
    for i in range(M):
        initialization_step_2(agents[i].agent_id,M,agents)

def Create_CT_Roots_for_M_agents(agents,M,counters):
    for i in range(M):
        CT_Root = Create_CT_Root_for_agent_id(agents[i].agent_id,agents[i])
        if Print_flag == 1:
            print('\nPrint CT_root for agent{}'.format(i))
            CT_Root.print_CT_Node()
        counters.openListCounter=counters.openListCounter+1
        (agents[i].openList).put((CT_Root.totalCost,counters.openListCounter,CT_Root))


def Main_program():
    numOfAgents=6
    for CnumOfAgents in range(numOfAgents):
        print('________________________{} Agents______________________________'.format(CnumOfAgents+1))
        #Set the problem data
        agents=[]
        counters=Counters()
        test_map,map_cols, map_rows, M, startpoints, goals =empty48x48(CnumOfAgents+1)
        start = time.time()
        # list of Priority Queues msgs
        MsgsQueues = []
        for i in range(M):
            MsgsQueues.append(queue.PriorityQueue())
        #initiate M agents
        for i in range(M):
            new_agent=Agent(i,startpoints[i],goals[i],test_map,[],math.inf,[],map_cols,map_rows,MsgsQueues)
            agents.append(new_agent)
            if Print_flag == 1:
                agents[i].print_agent_attributs()
        initialization_step_1_M_agents(agents,M,counters)
        initialization_step_2_M_agents(agents, M)
        Create_CT_Roots_for_M_agents(agents, M,counters)

        # Main Process(Agent Ai)
        msgsQueues = checkMsgsQueues(agents[0].MsgsQueues)
        openListsCT_nodes = checkOpenLists(agents,M)
        while msgsQueues or openListsCT_nodes:
            '''Handle a new CTNode from OpenSet
            Handle Incoming Messages'''
            counters.RoundRobin_Iterations = counters.RoundRobin_Iterations + 1
            for i in range(M):
                if Print_flag == 1:
                    print('the turn of agent{} is START'.format(agents[i].agent_id))
                # Handle Incoming Messages
                q=agents[i].MsgsQueues[agents[i].agent_id]
                if q.empty() == False:
                    new_msg = q.get()
                    handleNewMsg(new_msg[2], agents[i],counters,M)

                # Handle a new CTNode from OpenList
                if (agents[i].openList).empty() == False:
                    new_Node = (agents[i].openList).get()
                    # openList is Priority Queue - it pops the lowest cost every time
                    if new_Node[0] < agents[i].incumbentSolutionCost:
                        handleNewCT_Node(new_Node[2], agents[i],M,counters)
                    else:
                        if Print_flag == 1:
                            print(
                                'all the Nodes in the open list are more expensive than incumbentSolutionCost - pop them all')
                        while (agents[i].openList).empty() == False:
                            (agents[i].openList).get()
                if Print_flag == 1:
                    print('the turn of agent{} is OVER'.format(agents[i].agent_id))

            msgsQueues = checkMsgsQueues(agents[0].MsgsQueues)
            openListsCT_nodes = checkOpenLists(agents,M)
        end = time.time()
        total_msgs = counters.Counter_InitMsgs + counters.Counter_GoalMsgs + counters.Counter_NewNodeMsgs

        # ###results##
        print('Agent{} final solution cost:{}'.format(0, agents[0].incumbentSolutionCost))
        agents[0].incumbentSolution.print_solutions()
        print('Msgs-Counter:')
        print('Counter_InitMsgs:{}\nCounter_GoalMsgs:{}\nCounter_NewNodeMsgs:{}'.format(counters.Counter_InitMsgs,counters.Counter_GoalMsgs,counters.Counter_NewNodeMsgs))
        print('Total number of Msgs:{}'.format(total_msgs))
        print('RoundRobin_Iterations:{}\nCounter expanded Nodes:{}'.format(counters.RoundRobin_Iterations,counters.Counter_expand_Nodes))
        print('time taken: {}'.format(end - start))
    # df=add_new_result_to_cvs('empty-8-8.map-1.scen','empty-8-8',M,agents[0].incumbentSolutionCost,end - start,counters.Counter_InitMsgs,counters.Counter_NewNodeMsgs,counters.Counter_GoalMsgs,total_msgs,counters.Counter_expand_Nodes,counters.RoundRobin_Iterations)
    # new_line(df)



if __name__ == "__main__":
    # execute only if run as a script
    Main_program()
