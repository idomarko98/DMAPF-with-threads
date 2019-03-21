#File Objects
#Suitable for every map from this link: https://movingai.com/benchmarks/grids.html
import pdb

def txtfile_to_2D_map(fname):
    map = [] #2dim map
    with open(fname,'r') as f:
        #read hight from the file
        f_contents = f.read(19)
        rows= int(f.readline())
        print("The rows number is: "+str(rows))

        #read width from the
        f_contents = f.read(6)
        cols= int(f.readline())
        print("The cols number is: "+str(cols))

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
                  map[lineNum].append(-1) # unpassable terrain
          lineNum = lineNum+1
        for i in range(len(map)):
            print(map[i])
        return rows,cols,map

