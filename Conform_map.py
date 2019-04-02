import pdb
import pandas as pd

'''
This file contain fuction that handle with write and read from files - maps, scen fils. 
'''

#convert text file map to 2dim array map with 0s-passable and 1s-wall unpassable.
def txtfile_to_2D_map(fname):
    map = [] #2dim map
    with open(fname,'r') as f:
        #read hight from the file
        f_contents = f.read(19)
        rows= int(f.readline())

        #read width from the
        f_contents = f.read(6)
        cols= int(f.readline())

        #Reading lines and encoding - 0 passable terrain, 1- unpassable terrain
        f_contents=f.readline()
        lineNum = 0
        for line in f:
          map.append([])
          curruntline=line.split()
          for i in range(cols):
              if curruntline[0][i] == '.':
                  map[lineNum].append(0)# passable terrain
              else:
                  map[lineNum].append(1) # unpassable terrain
          lineNum = lineNum+1
        return rows,cols,map

#Read scen text file and return 2 lists of starts and goals point of the test.
def scan_txtfile_to_start_goal_lists(fname):
    starts = []  # list of points
    goals = [] #list of points
    with open(fname, 'r') as f:
        #skip the first line
        next(f)
        for line in f:
            curruntline = line.split()
            starts.append([int(curruntline[5]),int(curruntline[4])])
            goals.append([int(curruntline[7]),int(curruntline[6])])
    return starts,goals

#the function create excel file with the head of the results file
def create_head_frame_results_to_cvs():
    Result = {'File': [],
              'Map': [],
              'Number of Agents': [],
              'Cost': [],
              'time': [],
              'init_msgs': [],
              'CT_Node_msgs': [],
              'Goal Msgs': [],
              'total_msgs': [],
              'expanded_nodes': [],
              'roundRobinIteration': []
              }
    df = pd.DataFrame(Result, columns=['File', 'Map', 'Number of Agents', 'Cost', 'time', 'init_msgs', 'CT_Node_msgs',
                                       'Goal Msgs', 'total_msgs', 'expanded_nodes', 'roundRobinIteration'])
    df.to_excel('results.xlsx')

#the function gets all the new result parameters and return data frame that contain them.
def add_new_result_to_cvs(fname_test,fname_map,numberOfAgents,cost,time,init_msgs,CT_Node_msgs,goal_msgs,total_msgs,expanded_nodes,roundRobinIteration):
    #create_head_frame_results_to_cvs()
    df2 = pd.DataFrame({'File': [fname_test],
    'Map': [fname_map],
    'Number of Agents': [numberOfAgents],
    'Cost': [cost],
    'time': [time],
    'init_msgs': [init_msgs],
    'CT_Node_msgs': [CT_Node_msgs],
    'Goal Msgs': [goal_msgs],
    'total_msgs': [total_msgs],
    'expanded_nodes': [expanded_nodes],'roundRobinIteration': [roundRobinIteration]})
    return df2

#the function gets a new frame and append it to the resuls excel file.
def new_line(df1):
    writer = pd.ExcelWriter('Results1.xlsx')
    df1.to_excel(writer, 'Sheet1',index=False)
    writer.save()




