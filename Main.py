# import queue
import concurrent.futures as fs
import sys
import time

from CT_Node import *
from Heuristic_Dijkstra import *
from Low_Level import *
from Msgs import *
from Solution import Solution
from Tests import *

# with threads
# 0 - doesn't print comments 1- print comments
Print_flag = 0

M = 6
MsgsQueues = []


class Counters:
    def __init__(self):
        self.Counter_InitMsgs = 0
        self.Counter_GoalMsgs = 0
        self.Counter_NewNodeMsgs = 0
        self.openListCounter = 0
        self.RoundRobin_Iterations = 0
        self.Counter_expand_Nodes = 0

    def clear(self):
        self.Counter_InitMsgs = 0
        self.Counter_GoalMsgs = 0
        self.Counter_NewNodeMsgs = 0
        self.openListCounter = 0
        self.RoundRobin_Iterations = 0
        self.Counter_expand_Nodes = 0


counters = Counters()


class Agent:
    def __init__(self, agent_id, startpoint, goal, map, constrains, incumbentSolutionCost, incumbentSolution, map_cols,
                 map_rows):
        # initiate  attributes
        self.agent_id = agent_id
        self.startpoint = startpoint
        self.goal = goal
        self.map = map
        self.constrains = constrains
        self.openList = queue.PriorityQueue()
        self.incumbentSolutionCost = incumbentSolutionCost
        self.incumbentSolution = incumbentSolution
        self.map_cols = map_cols
        self.map_rows = map_rows
        self.heuristicMap = create_Heuristic_map(map_rows, map_cols, map, (goal[0], goal[1]))

    # def __reduce__(self): return (self.__class__, (self.agent_id, self.startpoint, self.goal, self.map,
    # self.constrains, self.incumbentSolutionCost, self.incumbentSolution, self.map_cols, self.map_rows,
    # self queue.PriorityQueue()sgsQueues)) print Agent attributs for check
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
    if Print_flag == 1:
        print('\nagent{}: start Initialization step 1'.format(agent.agent_id))
        # -----------------   Initialization Step 1 ------------------
        # Find optimal path for yourself
    path, cost = find_optimal_path(agent.startpoint[0],
                                   agent.startpoint[1],
                                   agent.goal[0],
                                   agent.goal[1],
                                   agent.map,
                                   agent.heuristicMap,
                                   [],
                                   agent.map_cols,
                                   agent.map_rows,
                                   Print_flag)
    # agent.map_rows)

    if path:
        if Print_flag == 1:
            print('cost: {} path:'.format(cost))
            print_path(path)
        # Create Init Msg and send it to all the agents
        create_Send_init_msgs(path, cost, agent.agent_id, agent)
        if Print_flag == 1:
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


def initialization_step_2(agent_id, agents):
    if Print_flag == 1:
        print('agent{}: start Initialization step 2'.format(agent_id))
    q = MsgsQueues[agent_id]
    if q.qsize() == M:
        if Print_flag == 1:
            print("agent{} received all the init Msgs".format(agent_id))
    else:
        if Print_flag == 1:
            print("Error: agent{} Not received all the init Msgs".format(agent_id))
    if Print_flag == 1:
        print('agent{}: finished Initialization step 2'.format(agent_id))


# Create CT.Root with the solution from the init_msgs  - return CT.root
def Create_CT_Root_for_agent_id(agent_id, agent):
    solutions = []
    totalCost = 0
    msgsQueue = MsgsQueues[agent_id]
    while not msgsQueue.empty():
        msg_tuple = msgsQueue.get()
        incoming_Init_Msg = msg_tuple[2]
        # check if : Msg is init Msg with type = 1
        if incoming_Init_Msg.type == 1:
            newSolution = Solution(incoming_Init_Msg.source, incoming_Init_Msg.path, incoming_Init_Msg.cost)
            solutions.append(newSolution)
            totalCost = totalCost + incoming_Init_Msg.cost
        else:
            print("Error: unexpected message received")
            return -1
    CT_root = CT_Node(solutions, totalCost, [], None)
    return CT_root


# Create Init Msg and send it to all the agents
def create_Send_init_msgs(path, cost, agent_id, agent, m):
    for i in range(m):
        new_init_Msg = Init_Msg(path, cost, agent_id, i)
        if Print_flag == 1:
            new_init_Msg.print_Msg()
        counters.Counter_InitMsgs = counters.Counter_InitMsgs + 1
        MsgsQueues[i].put((1, counters.Counter_InitMsgs, new_init_Msg))
        if Print_flag == 1:
            print('succsseed to put')


def print_Msg_queue(queueToprint, name):
    print('print {}'.format(name))
    for i in range(queueToprint.qsize()):
        (queueToprint.get()[2]).print_Msg()


# check if there are more msgs to handle
def checkMsgsQueues():
    for i in range(len(MsgsQueues)):
        if not MsgsQueues[i].empty():
            if Print_flag == 1:
                print("there are more msgs to handle...")
            return True
    if Print_flag == 1:
        print("No more msgs to handle...")
    return False


# check if there are more CT_Nodes to handle
def checkOpenLists(agents):
    for i in range(M):
        if not agents[i].openList.empty():
            if Print_flag == 1:
                print('more CT Nodes to handle in openList...')
            return True
    if Print_flag == 1:
        print("No more CT Nodes to handle in openList")
    return False


def handle_agent_msgs(agent, m):
    if Print_flag == 1:
        total_msgs = counters.Counter_InitMsgs + counters.Counter_GoalMsgs + counters.Counter_NewNodeMsgs
        print('total messages:  {}  '.format(total_msgs))
        print('the turn of agent{} is START'.format(agent.agent_id))
    # Handle Incoming Messages
    # q = agent.MsgsQueues[agent.agent_id]
    q = MsgsQueues[agent.agent_id]
    if not q.empty():
        new_msg = q.get()
        handleNewMsg(new_msg[2], agent, m)
    # Handle a new CTNode from OpenList
    if not agent.openList.empty():
        new_node = agent.openList.get()
        # openList is Priority Queue - it pops the lowest cost every time
        if new_node[0] < agent.incumbentSolutionCost:
            handleNewCT_Node(new_node[2], agent, m)
        else:
            if Print_flag == 1:
                print('all the Nodes in the open list are more expensive than incumbentSolutionCost - '
                      'pop them all')
            while not agent.openList.empty():
                agent.openList.get()
    if Print_flag == 1:
        print('the turn of agent{} is OVER'.format(agent.agent_id))
    return agent.incumbentSolutionCost


# shawn 4
def handle(agents):
    counters.RoundRobin_Iterations = counters.RoundRobin_Iterations + 1
    with fs.ThreadPoolExecutor() as executor:
        for i in range(M):
            fu = executor.submit(handle_agent_msgs, agents[i])
        # if not fu.result() is None:
    executor.shutdown(wait=True)
    msgs_queues = checkMsgsQueues()
    open_lists_ct_nodes = checkOpenLists(agents)
    return msgs_queues, open_lists_ct_nodes


def handleNewMsg(newmsg, agent, m):
    if newmsg.type == 2:
        if Print_flag == 1:
            print('agent{}: start handle New Goal Msg'.format(agent.agent_id))
        handle_Goal_Msg(newmsg, agent)
        if Print_flag == 1:
            print('agent{}: finished handle New Goal Msg'.format(agent.agent_id))
    elif newmsg.type == 3:
        if Print_flag == 1:
            print('agent{}: start handle NewCTNode_Msg'.format(agent.agent_id))
        handle_NewCTNode_Msg(newmsg, agent, m)
        if Print_flag == 1:
            print('agent{}: finished handle NewCTNode_Msg'.format(agent.agent_id))


def handle_Goal_Msg(goal_msg, agent):
    if goal_msg.solutionCost < agent.incumbentSolutionCost:
        # Update the best solution and the cost
        if Print_flag == 1:
            print('Update the new best solution and the cost:{}'.format(goal_msg.solutionCost))
        agent.incumbentSolutionCost = goal_msg.solutionCost
        agent.incumbentSolution = copy.deepcopy(goal_msg.CTNode_solution)
    else:
        if Print_flag == 1:
            print('The incumbent cost solution is better drop the goal message')


def handle_NewCTNode_Msg(newCTNode_msg, agent, m):
    constrains = copy.deepcopy(newCTNode_msg.CTNode.conflicts)
    constrains.append(newCTNode_msg.constrains)
    start_i = agent.startpoint[0]
    start_j = agent.startpoint[1]
    goal_i = agent.goal[0]
    goal_j = agent.goal[1]
    path, cost = find_optimal_path(start_i, start_j, goal_i, goal_j, agent.map, agent.heuristicMap,
                                   constrains, agent.map_cols, agent.map_rows, Print_flag)
    # , constrains, agent.map_cols, agent.map_rows)
    if Print_flag == 1:
        print('print new path:')
        print_path(path)
    if path:
        # Calculate new solution cost:
        new_Total_cost = cost
        for i in range(m):
            if newCTNode_msg.CTNode.solutions[i].agent_Id != newCTNode_msg.destination:
                new_Total_cost = new_Total_cost + newCTNode_msg.CTNode.solutions[i].cost
        if new_Total_cost < agent.incumbentSolutionCost:
            # pdb.set_trace()
            new_Solution = copy.deepcopy(newCTNode_msg.CTNode.solutions)
            # To add: find index of solution of agent t.name
            new_Solution[agent.agent_id] = Solution(agent.agent_id, path, cost)
            # Create CTNodeChild
            CTNodeChild = CT_Node(new_Solution, new_Total_cost, constrains, newCTNode_msg.CTNode)
            counters.openListCounter = counters.openListCounter + 1
            # print("put4")
            agent.openList.put((new_Total_cost, counters.openListCounter, CTNodeChild))
            counters.Counter_expand_Nodes = counters.Counter_expand_Nodes + 1
            if Print_flag == 1:
                print('new CTNode child created and addedm to openlist')
        else:
            if Print_flag == 1:
                print(
                    'The incumbent solution cost is better than the new solution of NewCTNode_Msg, drop NewCTNode_Msg')
    if Print_flag == 1:
        print('Done')


def handleNewCT_Node(new_Node, agent, m):
    agent_id = agent.agent_id
    if Print_flag == 1:
        print('agent{}: start handle New CT_Node from openList'.format(agent_id))
    newConflict = new_Node.find_conflicts()
    if newConflict is None:
        newEdgeConflict = new_Node.find_Edges_conflicts()
        if newEdgeConflict is None:
            # Create new Goal Msg - broadcast message to all the agents
            create_Send_goal_msgs(new_Node, agent_id, agent, m)
            if Print_flag == 1:
                print('there is no conflict in new_Node - create Goal Msg and send to all the agents')
        else:  # new Edge conflict
            if Print_flag == 1:
                print('there is new Edge conflict:')
                newEdgeConflict.print_Edge_conflict()
            for i in range(M):
                # Create NewCTNode_Msg
                CT_Node_msg = NewCTNode_Msg(new_Node, newEdgeConflict, agent_id, i)
                if i in newEdgeConflict.involved_Agents:
                    if i == agent_id:
                        handle_NewCTNode_Msg(CT_Node_msg, agent, m)
                    else:
                        counters.Counter_NewNodeMsgs = counters.Counter_NewNodeMsgs + 1
                        # print("put5")
                        MsgsQueues[i].put((3, counters.Counter_NewNodeMsgs, CT_Node_msg))
                        #shawn 5
    else:  # new conflict
        if Print_flag == 1:
            print('there is new conflict:')
            newConflict.print_conflict()
        for i in range(M):
            # Create NewCTNode_Msg
            CT_Node_msg = NewCTNode_Msg(new_Node, newConflict, agent_id, i)
            if i in newConflict.involved_Agents:
                if i == agent_id:
                    handle_NewCTNode_Msg(CT_Node_msg, agent, m)
                else:
                    counters.Counter_NewNodeMsgs = counters.Counter_NewNodeMsgs + 1
                    MsgsQueues[i].put((3, counters.Counter_NewNodeMsgs, CT_Node_msg))
                    #shawn 6


# Create goal Msg and send it to all the agents
def create_Send_goal_msgs(new_Node, agent_id, agent, m):
    for i in range(m):
        new_goal_Msg = Goal_Msg(new_Node, new_Node.totalCost, agent_id, i)
        counters.Counter_GoalMsgs = counters.Counter_GoalMsgs + 1
        MsgsQueues[i].put((1, counters.Counter_GoalMsgs, new_goal_Msg))
        #shawn 7
        if Print_flag == 1:
            print('succeeded to put')
    if Print_flag == 1:
        print('agent{}: done handle New CT_Node'.format(agent_id))


def initialization_step_1_M_agents(agents):
    """
    Initialization Step 1:
    Find optimal path for yourself and send Init Msgs to all the agents
    """
    for i in range(M):
        check_path = initialization_step_1(agents[i])
        if check_path == -1:
            print('exit')
            sys.exit("there is no solution")


def init_step1(agent, m):
    # agent = agents[i]
    if Print_flag == 1:
        print('\nagent{}: start Initialization step 1'.format(agent.agent_id))
        # -----------------   Initialization Step 1 ------------------
        # Find optimal path for yourself
    path, cost = find_optimal_path(agent.startpoint[0], agent.startpoint[1], agent.goal[0], agent.goal[1], agent.map,
                                   agent.heuristicMap, [], agent.map_cols, agent.map_rows, Print_flag)
    # agent.map_rows)

    if path:
        if Print_flag == 1:
            print('cost: {} path:'.format(cost))
            print_path(path)
        # Create Init Msg and send it to all the agents
        create_Send_init_msgs(path, cost, agent.agent_id, agent, m)
        if Print_flag == 1:
            print('agent{}: finished Initialization step 1'.format(agent.agent_id))
    else:
        print('exit')
        # sys.exit("there is no solution")


'''
Initialization Step 2:
check that agent received all the init msgs 
'''


def initialization_step_2_M_agents(agents):
    for i in range(M):
        initialization_step_2(agents[i].agent_id, agents)


def Create_CT_Roots_for_an_agent(agent, m):
    CT_Root = Create_CT_Root_for_agent_id(agent.agent_id, agent)
    counters.openListCounter = counters.openListCounter + 1
    agent.openList.put((CT_Root.totalCost, counters.openListCounter, CT_Root))


def Create_CT_Roots_for_M_agents(agents):
    for i in range(M):
        CT_Root = Create_CT_Root_for_agent_id(agents[i].agent_id, agents[i])
        if Print_flag == 1:
            print('\nPrint CT_root for agent{}'.format(i))
            CT_Root.print_CT_Node()
        counters.openListCounter = counters.openListCounter + 1
        agents[i].openList.put((CT_Root.totalCost, counters.openListCounter, CT_Root))


# def New_Agent(i, start_point, goal, map, constrains, incumbentSolutionCost, incumbentSolution, map_cols, map_rows,
#               msgs_queues):
#     a = Agent(i, start_point, goal, map, constrains, incumbentSolutionCost, incumbentSolution, map_cols, map_rows,
#               msgs_queues)
#     # a.start()
#     return a

def handle_agent_i(agent, m):
    # Main Process(Agent Ai)
    msgs_queues = not MsgsQueues[agent.agent_id].empty()
    open_lists_ct_nodes = not agent.openList.empty()
    while msgs_queues or open_lists_ct_nodes:
        '''Handle a new CTNode from OpenSet
        Handle Incoming Messages'''
        counters.RoundRobin_Iterations = counters.RoundRobin_Iterations + 1
        handle_agent_msgs(agent, m)
        msgs_queues = not MsgsQueues[agent.agent_id].empty()
        open_lists_ct_nodes = not agent.openList.empty()
    return agent.incumbentSolutionCost


def run_for_each_agent(agents, m, func):
    with fs.ThreadPoolExecutor() as executor:
        for Ai in range(m):
            fu = executor.submit(func, agents[Ai], m)
            if fu is not None:
                sys.stdout.write("\r" + str(fu.result()))
                sys.stdout.flush()
    executor.shutdown(wait=True)
    return fu.result()


def Main_program(m, number_of_iterations):
    runs = 0
    while runs < number_of_iterations:
        runs += 1
        print('________________________{} Agents______________________________'.format(m))
        # Set the problem data
        agents = []
        test_map, map_cols, map_rows, startpoints, goals = map1_22X28(m)
        start = time.time()
        # list of Priority Queues msgs
        for i in range(m):
            MsgsQueues.append(queue.PriorityQueue())
        # initiate M agents
        for i in range(m):
            new_agent = Agent(i, startpoints[i], goals[i], test_map, [], math.inf, [], map_cols, map_rows)
            agents.append(new_agent)
            if Print_flag == 1:
                agents[i].print_agent_attributs()
        run_for_each_agent(agents, m, init_step1)
        run_for_each_agent(agents, m, Create_CT_Roots_for_an_agent)
        solCost = run_for_each_agent(agents, m, handle_agent_i)
        end = time.time()
        total_msgs = counters.Counter_InitMsgs + counters.Counter_GoalMsgs + counters.Counter_NewNodeMsgs

        ###results##
        print('\nAgent{} final solution cost:{}'.format(0, solCost))
        if agents[0] is not None:
            if agents[0].incumbentSolution is not None:
                agents[0].incumbentSolution.print_solutions()
        print('Msgs-Counter:')
        print('Counter_InitMsgs:{}\nCounter_GoalMsgs:{}\nCounter_NewNodeMsgs:{}'.format(counters.Counter_InitMsgs,
                                                                                        counters.Counter_GoalMsgs,
                                                                                        counters.Counter_NewNodeMsgs))
        print('Total number of Msgs:{}'.format(total_msgs))
        print('RoundRobin_Iterations:{}\nCounter expanded Nodes:{}'.format(counters.RoundRobin_Iterations,
                                                                           counters.Counter_expand_Nodes))
        print('time taken: {}'.format(end - start))
        # df = add_new_result_to_cvs('map1_22X28.map-1.scen', 'map1_22X28.map', m, solCost,
        #                            end - start,
        #                            counters.Counter_InitMsgs, counters.Counter_NewNodeMsgs,
        #                            counters.Counter_GoalMsgs,
        #                            total_msgs, counters.Counter_expand_Nodes, counters.RoundRobin_Iterations)
        # new_line(df)
        MsgsQueues.clear()
        counters.clear()


if __name__ == "__main__":
    # execute only if run as a script
    for number_of_agents in range(3, 6):
        # for i in range(M):
        Main_program(number_of_agents + 1, 50)
