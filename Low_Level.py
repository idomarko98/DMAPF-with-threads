import queue
import math
import copy
import pdb

Counter = 0
class Spot:
    def __init__(self, state, f, g, h, path):
        self.state = copy.deepcopy(state)
        self.f = f
        self.g= g
        self.h=h
        self.path = copy.deepcopy(path) # list of states from start point to goal point

    def print_spot(self):
        print('print spot:')
        print('f:{} \ncurrent state:'.format(self.f))
        self.state.print_State()
        print('\npath:')
        for i in range(len(self.path)):
            self.path[i].print_State()

class State():
    def __init__(self, i, j, time):
        self.i = i
        self.j = j
        self.time=time
    def print_State(self):
        state = "(i={},j={},time={}),"
        print(state.format(self.i, self.j, self.time), end ="")

    def compareStates(self,i,j,time):
        if self.i == i and self.j == j and self.time == time:
            return True
        return False

def check_if_popedSpot_is_goal(state, goal_i, goal_j):
    if state.i == goal_i and state.j == goal_j:
        return True
    return False

def Check_if_possible_extension(extand_state,dic_close_list,dic_open_list,constrains,map):
    #chack if extand_state is already in closeList
    key=(extand_state.i,extand_state.j,extand_state.time)
    if key in dic_close_list:
        return False
    #chack if extand_state is already in openList
    if key in dic_open_list:
        return False
    # chack if extand_state is 'wall' in map
    if map[extand_state.i][extand_state.j]== 1:
        return False
    # chack if extand_state is constrain
    stateInConstrains = checkIfstateInConstrains(extand_state,constrains)
    if stateInConstrains:
        return False
    return True

def checkIfstateInConstrains(extand_state,constrains):
    for i in range(len(constrains)):
        stateIsConstrain= extand_state.compareStates(constrains[i].i,constrains[i].j,constrains[i].time)
        if stateIsConstrain:
            return True
    return False

#i-row j-col
def create_extand_states(state, map_cols, map_rows):
    new_states_lists=[]
    if state.i < map_rows-1:
        new_states_lists.append(State(state.i+1,state.j,state.time+1))
    if state.i >0:
        new_states_lists.append(State(state.i-1, state.j, state.time + 1))
    if state.j < map_cols-1:
        new_states_lists.append(State(state.i, state.j+1, state.time + 1))
    if state.j > 0:
        new_states_lists.append(State(state.i, state.j-1, state.time + 1))
    return new_states_lists


def find_optimal_path(start_i,start_j,goal_i,goal_j,map,heuristicMap,constrains, map_cols, map_rows):
    h_start = heuristicMap[start_i,start_j]
    g_start=0
    f_start=h_start+g_start
    time=0
    start_spot=Spot(State(start_i,start_j,time),f_start,g_start,h_start,[])
    dic_open_list = {}  # dictionary <Key: state(i,j,t)> Val:<Spot object>
    openList=queue.PriorityQueue()
    global Counter
    Counter = Counter + 1
    openList.put((f_start,Counter,start_spot))
    dic_open_list[(start_spot.state.i, start_spot.state.j,start_spot.state.time)] = start_spot.f
    dic_close_list={} #dictionary <Key: state(i,j,t)> Val:<Spot object>
    while openList.empty() == False:
        popedSpot=openList.get()[2]
        isgoal = check_if_popedSpot_is_goal(popedSpot.state,goal_i, goal_j)
        if isgoal:
            print("path found!")
            popedSpot.path.append(popedSpot.state)
            g = popedSpot.g
            h=0
            f =g+h
            popedSpot.g=0
            popedSpot.f=f
            return popedSpot.path, popedSpot.f
        else:
            extand_list=create_extand_states(popedSpot.state,map_cols, map_rows)
            extand_list.append(State(popedSpot.state.i,popedSpot.state.j,popedSpot.state.time+1))
            for i in range(len(extand_list)):
                possible_extension=Check_if_possible_extension(extand_list[i],dic_close_list,dic_open_list,constrains,map)
                if possible_extension:
                    #h=heuristicMap[extand_list[i].i][extand_list[i].j]
                    h= heuristicMap[extand_list[i].i,extand_list[i].j]
                    g=popedSpot.g+1
                    f=g+h
                    path=copy.deepcopy(popedSpot.path)
                    path.append(popedSpot.state)
                    new__extensions_Spot=Spot(extand_list[i],f,g,h,path)
                    Counter=Counter+1
                    openList.put((f,Counter,new__extensions_Spot))
                    dic_open_list[(new__extensions_Spot.state.i,new__extensions_Spot.state.j,new__extensions_Spot.state.time)]=f
            dic_close_list[(popedSpot.state.i,popedSpot.state.j,popedSpot.state.time)]=popedSpot



# new_state=State(0,0,0)
# new_state1=State(0,1,1)
# new_state2=State(1,1,2)
# new_Spot =Spot(new_state,5,0,0,[new_state,new_state1,new_state2])
# new_Spot.print_spot()