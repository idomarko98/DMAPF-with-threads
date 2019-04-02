from Conform_map import *
def test1():
    map8x8 = [
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0]]
    map_cols = 8
    map_rows = 8
    numOfAgents =2
    startpoints = [[6,5],[3,4],[4,5],[0,7],[7,5],[6,3],[5,0],[6,0],[1,7],[3,0],[0,2],[7,2],[0,0],[1,4],[4,1],[0,4],[0,5],[6,2],[2,3],[2,2],[1,2],[5,2],[3,7],[4,7],[0,1],[3,6],[4,3],[4,3],[5,5],[3,2],[5,3],[7,7],[6,7]]
    goals =       [[7,6],[4,6],[1,3],[0,6],[5,7],[3,1],[5,4],[7,3],[6,4],[1,5],[2,4],[2,1],[1,0],[3,3],[4,2],[4,4],[7,4],[6,1],[2,7],[2,0],[1,1],[4,0],[5,1],[6,6],[3,5],[0,3],[1,6],[2,5],[7,0],[5,6],[2,6],[7,1]]

    return map8x8, map_cols, map_rows, numOfAgents, startpoints, goals

def test2():
    map16x16 = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
    map_cols = 16
    map_rows = 16
    numOfAgents =10
    startpoints = [[12,6],[12,4],[12,0],[5,5],[10,6],[2,1],[10,5],[11,3],[9,11],[13,10]]
    goals =       [[4,10],[15,1],[12,15],[0,3],[1,12],[4,2],[10,13],[7,0],[0,1],[8,11]]

    return map16x16, map_cols, map_rows, numOfAgents, startpoints, goals


def test4X4():
    map4x4 = [
            [1, 0, 0, 1],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [1, 0, 0, 1]]
    map_cols = 4
    map_rows = 4
    numOfAgents =2
    startpoints =[[0,1],[1,0]]
    goals=[[3,2],[2,3]]
    return map4x4, map_cols, map_rows, numOfAgents, startpoints, goals
def test4():
    map_rows,map_cols, map_Berlin_1_256 = txtfile_to_2D_map('Berlin_1_256.map')
    numOfAgents =1
    startpoints = [[254,112]]
    goals =       [[222,219]]
    return map_Berlin_1_256, map_cols, map_rows, numOfAgents, startpoints, goals

def test_room32X32_4():
    map_rows,map_cols, room32X32_4 = txtfile_to_2D_map('room-32-32-4.map')
    numOfAgents =16
    startpoints,goals = scan_txtfile_to_start_goal_lists('room-32-32-4.map-1.scen')
    return room32X32_4, map_cols, map_rows, numOfAgents, startpoints, goals

def test_room64X64_8():
    map_rows,map_cols, room64X64 = txtfile_to_2D_map('room-64-64-8.map')
    numOfAgents =4
    startpoints,goals = scan_txtfile_to_start_goal_lists('room-64-64-8.map-1.scen')
    return room64X64, map_cols, map_rows, numOfAgents, startpoints, goals

def test_radom64X64_20():
    map_rows,map_cols, radom64X64 = txtfile_to_2D_map('random-64-64-20.map')
    numOfAgents =5
    startpoints,goals = scan_txtfile_to_start_goal_lists('random-64-64-20.map-1.scen')
    return radom64X64, map_cols, map_rows, numOfAgents, startpoints, goals
def test_radom32X32_20():
    map_rows,map_cols, radom32X32 = txtfile_to_2D_map('random-32-32-20.map')
    numOfAgents =13
    startpoints,goals = scan_txtfile_to_start_goal_lists('random-32-32-20.map-1.scen')
    return radom32X32, map_cols, map_rows, numOfAgents, startpoints, goals

def test_radom32X32_10():
    map_rows,map_cols, radom32X32 = txtfile_to_2D_map('Maps_files/random-32-32-10.map')
    numOfAgents =1
    startpoints,goals = scan_txtfile_to_start_goal_lists('Tests_files/random-32-32-10.map-10.scen')
    return radom32X32, map_cols, map_rows, numOfAgents, startpoints, goals

def test_maze32x32_2():
    map_rows,map_cols, map_maze32x32 = txtfile_to_2D_map('Maps_files/maze-32-32-2.map')
    numOfAgents =6
    startpoints,goals = scan_txtfile_to_start_goal_lists('Tests_files/maze-32-32-2.map-1.scen')
    return map_maze32x32, map_cols, map_rows, numOfAgents, startpoints, goals