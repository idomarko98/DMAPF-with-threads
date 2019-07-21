# Parent class
import copy


class Msg:
    def __init__(self, type, source, destination):
        self.type = type
        self.source = source
        self.destination = destination
    def print_Msg(self):
        msg= "Msg type: {}, Msg source: agent Id {}, Msg destination: agent Id {}."  # 3 {} placeholders
        print(msg.format(self.type, self.source, self.destination))  # Pass 3 strings into method, separated by a comma


# This class inherits type and from and to arguments from Msg class (which can work for any Msg)

# Initiates a InitMsg object with path, cost parameters.
# InitMsg type is  1
#
class Init_Msg(Msg):
    def __init__(self, path, cost, source, destination):
        Msg.__init__(self, 1, source, destination)
        self.path = copy.deepcopy(path)
        self.cost = cost

    def print_Msg(self):
        print("Init Msg")
        Msg.print_Msg(self)
        msg = "path: {}, path cost: {}."  # 2 {} placeholders
        print(msg.format(self.path, self.cost)+"\n")  # Pass 2 strings into method, separated by a comma

# Initiates a NewCTNode_Msg object with CTNode, constranins parameters.
# InitMsg type is  3
class NewCTNode_Msg(Msg):
    def __init__(self, CTNode, constrains, source, destination):
        Msg.__init__(self, 3, source, destination)
        self.CTNode = copy.deepcopy(CTNode)
        self.constrains = copy.deepcopy(constrains)

    def print_Msg(self):
        print("NewCTNode_Msg")
        Msg.print_Msg(self)
        msg = "CTNode: {}, Constrains: {}."  # 2 {} placeholders
        print(msg.format(self.CTNode, self.constrains)+"\n")  # Pass 2 strings into method, separated by a comma


# Initiates a Goal_Msg object with CTNode_Solution, solutionCost parameters.
# InitMsg type is  2
class Goal_Msg(Msg):
    def __init__(self, CTNode_Solution, solutionCost, source, destination):
        Msg.__init__(self, 2, source, destination)
        self.CTNode_solution = copy.deepcopy(CTNode_Solution)
        self.solutionCost = solutionCost

    def print_Msg(self):
        print("Goal_Msg")
        Msg.print_Msg(self)
        msg = "CTNode_solution: {}, solution Cost: {}."  # 2 {} placeholders
        print(msg.format(self.CTNode_solution, self.solutionCost))  # Pass 2 strings into method, separated by a comma


