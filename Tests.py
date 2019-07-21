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
    numOfAgents = 2
    startpoints = [[6, 5], [3, 4], [4, 5], [0, 7], [7, 5], [6, 3], [5, 0], [6, 0], [1, 7], [3, 0], [0, 2], [7, 2],
                   [0, 0], [1, 4], [4, 1], [0, 4], [0, 5], [6, 2], [2, 3], [2, 2], [1, 2], [5, 2], [3, 7], [4, 7],
                   [0, 1], [3, 6], [4, 3], [4, 3], [5, 5], [3, 2], [5, 3], [7, 7], [6, 7]]
    goals = [[7, 6], [4, 6], [1, 3], [0, 6], [5, 7], [3, 1], [5, 4], [7, 3], [6, 4], [1, 5], [2, 4], [2, 1], [1, 0],
             [3, 3], [4, 2], [4, 4], [7, 4], [6, 1], [2, 7], [2, 0], [1, 1], [4, 0], [5, 1], [6, 6], [3, 5], [0, 3],
             [1, 6], [2, 5], [7, 0], [5, 6], [2, 6], [7, 1]]

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
    numOfAgents = 10
    startpoints = [[12, 6], [12, 4], [12, 0], [5, 5], [10, 6], [2, 1], [10, 5], [11, 3], [9, 11], [13, 10]]
    goals = [[4, 10], [15, 1], [12, 15], [0, 3], [1, 12], [4, 2], [10, 13], [7, 0], [0, 1], [8, 11]]

    return map16x16, map_cols, map_rows, numOfAgents, startpoints, goals


def test4X4():
    map4x4 = [
        [1, 0, 0, 1],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [1, 0, 0, 1]]
    map_cols = 4
    map_rows = 4
    numOfAgents = 2
    startpoints = [[1, 0], [1, 1], [0, 1], [1, 0]]
    goals = [[1, 1], [1, 0], [3, 2], [2, 3]]
    return map4x4, map_cols, map_rows, numOfAgents, startpoints, goals


def test2X2():
    map2x2 = [
        [0, 0],
        [0, 0]]
    map_cols = 2
    map_rows = 2
    numOfAgents = 2
    startpoints = [[1, 0], [1, 1]]
    goals = [[1, 1], [1, 0]]
    return map2x2, map_cols, map_rows, numOfAgents, startpoints, goals


def tests():
    # test4(1)
    # test_room32X32_4(4)
    # test_room64X64_8(8)
    # test_radom64X64_20(20)
    # test_radom32X32_20(20)
    # test_radom32X32_10(10)
    # test_maze32x32_2(2)
    # empty8x8(10)
    # empty48x48(20)
    # den312d(20)
    map1_22X28(6)

def run_scene(scen, mapa, numOfAgents):
    map_rows, map_cols, map1 = txtfile_to_2D_map(mapa)
    startpoints, goals = scan_txtfile_to_start_goal_lists(scen)
    return map1, map_cols, map_rows, startpoints, goals

def ht_chantry(numOfAgents):
    map_rows, map_cols, map1 = txtfile_to_2D_map('Maps_files/map1_22X28.map')
    startpoints, goals = scan_txtfile_to_start_goal_lists('Tests_files/ht_chantry.map-1.scen')
    return map1, map_cols, map_rows, numOfAgents, startpoints, goals


def ThreeAgents(numOfAgents):
    map_rows, map_cols, map1 = txtfile_to_2D_map('Maps_files/3agents.map')
    startpoints, goals = scan_txtfile_to_start_goal_lists('Tests_files/3agents.map-1.scen')
    return map1, map_cols, map_rows, numOfAgents, startpoints, goals


def test4(numOfAgents):
    map_rows, map_cols, map_Berlin_1_256 = txtfile_to_2D_map('Berlin_1_256.map')
    startpoints = [[254, 112]]
    goals = [[222, 219]]
    return map_Berlin_1_256, map_cols, map_rows, numOfAgents, startpoints, goals


def test_room32X32_4(numOfAgents):
    map_rows, map_cols, room32X32_4 = txtfile_to_2D_map('room-32-32-4.map')
    startpoints, goals = scan_txtfile_to_start_goal_lists('room-32-32-4.map-1.scen')
    return room32X32_4, map_cols, map_rows, numOfAgents, startpoints, goals


def test_room64X64_8(numOfAgents):
    map_rows, map_cols, room64X64 = txtfile_to_2D_map('room-64-64-8.map')
    startpoints, goals = scan_txtfile_to_start_goal_lists('room-64-64-8.map-1.scen')
    return room64X64, map_cols, map_rows, numOfAgents, startpoints, goals


def test_radom64X64_20(numOfAgents):
    map_rows, map_cols, radom64X64 = txtfile_to_2D_map('random-64-64-20.map')
    startpoints, goals = scan_txtfile_to_start_goal_lists('random-64-64-20.map-1.scen')
    return radom64X64, map_cols, map_rows, numOfAgents, startpoints, goals


def map1_22X28(numOfAgents):
    map_rows, map_cols, map1 = txtfile_to_2D_map('Maps_files/map1_22X28.map')
    startpoints, goals = scan_txtfile_to_start_goal_lists('Tests_files/map1_22X28.map-1.scen')
    return map1, map_cols, map_rows, startpoints, goals


def test_radom32X32_20(numOfAgents):
    map_rows, map_cols, radom32X32 = txtfile_to_2D_map('Maps_files/random-32-32-20.map')
    startpoints, goals = scan_txtfile_to_start_goal_lists('Tests_files/random-32-32-20.map-1.scen')
    return radom32X32, map_cols, map_rows, startpoints, goals


def test_radom32X32_10(numOfAgents):
    map_rows, map_cols, radom32X32 = txtfile_to_2D_map('Maps_files/random-32-32-10.map')
    startpoints, goals = scan_txtfile_to_start_goal_lists('Tests_files/random-32-32-10.map-10.scen')
    return radom32X32, map_cols, map_rows, numOfAgents, startpoints, goals


def test_maze32x32_2(numOfAgents):
    map_rows, map_cols, map_maze32x32 = txtfile_to_2D_map('Maps_files/maze-32-32-2.map')
    startpoints, goals = scan_txtfile_to_start_goal_lists('Tests_files/maze-32-32-2.map-1.scen')
    return map_maze32x32, map_cols, map_rows, startpoints, goals


def empty48x48(numOfAgents):
    map_rows, map_cols, empty48x48 = txtfile_to_2D_map('Maps_files/empty-48-48.map')
    startpoints, goals = scan_txtfile_to_start_goal_lists('Tests_files/empty-48-48.map-1.scen')
    return empty48x48, map_cols, map_rows, numOfAgents, startpoints, goals


def empty8x8(numOfAgents):
    map_rows, map_cols, empty8x8 = txtfile_to_2D_map('Maps_files/empty-8-8.map')
    startpoints, goals = scan_txtfile_to_start_goal_lists('Tests_files/empty-8-8.map-1.scen')
    return empty8x8, map_cols, map_rows, numOfAgents, startpoints, goals


def den312d(numOfAgents):
    map_rows, map_cols, den312d = txtfile_to_2D_map('Maps_files/den312d.map')
    startpoints, goals = scan_txtfile_to_start_goal_lists('Tests_files/den312d.map-1.scen')
    return den312d, map_cols, map_rows, numOfAgents, startpoints, goals
