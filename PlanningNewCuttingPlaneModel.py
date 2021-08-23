# -*- coding: utf-8 -*-
"""
Created on Thu Nov 21 14:11:06 2019

@author: shmhatre
"""
import sys # Library for INT_MAX 
import numpy as np
import csv
from docplex.mp.model import Model
from docplex.mp.context import Context
import datetime
import networkx as nx
import matplotlib.pyplot as plt

#All the functions

#Start of create graph 
def create_graph(edgeList):
    # create networkx graph
   
    DG=nx.DiGraph()
   
    # add edges
    for edge in edgeList :
        DG.add_edge(edge[0], edge[1])
      
#    print(NodeArcIndc_matrix)
    return DG

def create_weightedgraph(edgeList):
    # create networkx graph
   
    DGw =nx.DiGraph()
   
    # add edges
    for edge, value in edgeList.items() :
        DGw.add_edge(edge[0], edge[1], weight= value)
      
#    print(NodeArcIndc_matrix)
    return DGw
#End of Create_graph function
    
#Node_Arc incidence matrix
def NodeArcIncidence(DG):
    
    NodeArcInd = {}
    for arc in DG.edges():
        NodeArcInd[arc[0],(arc[0],arc[1])] = 1
        NodeArcInd[arc[1],(arc[0],arc[1])] = -1
        
    return NodeArcInd

#Generate Damage Scenario
def ScenarioGeneration(SampleSize,DamageProb,AssetGroup):
   
    sample =np.random.rand(SampleSize,len(DamageProb))
    
    GroupDamage={}
    i=0
    for row in sample: 
        j=0
        for key, value in DamageProb.items():
            if row[j] <value:
                GroupDamage[key,i] =1
            else:
                GroupDamage[key,i] = 0
            j +=1
            
        i +=1
    
    AssetDamage = {}
    for key, value in AssetGroup.items():
        for i in range(SampleSize):
            
            AssetDamage[key,i] = GroupDamage [value, i]
    
    
    return AssetDamage
#end of function ScenarioGeneration        

# Union of two lists
def union_lists(List1, List2):
    List = List1.copy()
    
    for i in List2:
        flag = 0
        for j in List1:
            if i==j:
                flag = 1
                break
        if flag == 0:
            List.append(i)
#    print('List = ', List)        
    return List       
            
# End of the union_lists()
    

def ParaLoading(ArcFile, AssetFile, AssetArcFile, DamageProbFile):
    AssetArc = {}
    AssetList= []
    TravellingTime = {}
    edgeList = []
    ProteResource = {}
    AssetGroup ={}
    DamageProb = {}
    RestorationResource ={}
    
    #Asset_association with arc_(i,j)
    with open(AssetArcFile, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        next(reader)
        
        for row in reader:
            AssetArc[(int(row[0]),int(row[1])),int(row[3])] = 1

    #Travelling time
    with open(ArcFile,'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        next(reader)                                                                                                                                                                                                                                                                     
        for row in reader:  
             TravellingTime[(int(row[0]),int(row[1]))] = float(row[2])
             edgeList.append([int(row[0]),int(row[1])])
             
    #Protection resources
    with open(AssetFile,'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        next(reader)
        for row in reader:  
             ProteResource[int(row[0])] = float(row[3])
             AssetGroup[int(row[0])] = int(row[4]) 
             RestorationResource[int(row[0])] = float(row[4])
             AssetList.append(int(row[0]))
             
    #Reading Asset group and Probability     
    with open(DamageProbFile,'r') as csvfile:
    
        reader = csv.reader(csvfile, delimiter=',')
        next(reader)
        for row in reader:  
            DamageProb[int(row[0])] = float(row[7])
            
            
    return AssetArc, AssetList, AssetGroup, TravellingTime, DamageProb, edgeList, \
           ProteResource,RestorationResource
        
#End of Parameter lading function                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
        
def STRPInit(SourceNode, DemandNode, NumScenario, 
         AssetArc, AssetList, AssetGroup, TravellingTime, DamageProb, 
         edgeList, ProteResource,RestorationResource):
    # Create a road network
    roadNet = create_graph(edgeList)
    NodeList = roadNet.nodes
    An_ij = NodeArcIncidence(roadNet)

    # Pair of (O,D)    
    ODPair =[[i,j] for i in SourceNode for j in DemandNode]
    DSPair =[[i,j] for i in DemandNode for j in SourceNode]
    ODPair.extend(DSPair)

    # Generate random damage scenarios
    #Set of Scenario
    SampleSpace=list(range(NumScenario))
    Damage = ScenarioGeneration(NumScenario, DamageProb, AssetGroup)   
      
    StartTime1 = datetime.datetime.now()  
    #Short term road protection model         
    STRoadProtect = Model(name="STRP")

    #Decision variables
    #Binary decision variable for asset protection
    x = {k : STRoadProtect.binary_var(name ="x_k{0}".format(k)) 
                        for k in AssetList}     
    
    v = {(k,s) : STRoadProtect.binary_var(name ="v_k{0}_s{1}".format(k,s)) 
                        for k in AssetList for s in SampleSpace}
    
    u ={(i,j,s) : STRoadProtect.binary_var(name ="u_(i,j){0}_s{1}".format((i,j),s)) 
                        for (i,j) in edgeList for s in SampleSpace}
    
    y = {(i,j,o,d,s) : STRoadProtect.binary_var(name ="y_(i,j){0}_(o,d){1}_s{2}".format((i,j),(o,d),s)) 
                        for (i,j) in edgeList for (o,d) in ODPair for s in SampleSpace}
    
    z = {(o,d,s) : STRoadProtect.binary_var(name ="z_(o,d){0}_s{1}".format((o,d),s)) 
                        for (o,d) in ODPair for s in SampleSpace}
    
##    objective function with z version
#    
    STRoadProtect.minimize(STRoadProtect.sum(TravellingTime.get((i,j),float('inf'))*\
    y[i,j,o,d,s] for (i,j) in edgeList for (o,d) in ODPair for s in SampleSpace) 
    +STRoadProtect.sum(1000*(1- z[o,d,s]) for (o,d) in ODPair for s in SampleSpace))
    
    #Constraints
     #5
    for s in SampleSpace:
        for k in AssetList:
             STRoadProtect.add_constraint(v[k,s] <= x[k] + (1 - Damage.get((k,s), 0)),\
            ctname = "survived_k{0}_s{1}".format(k,s))
    
#2z
    for s in SampleSpace:
        for (o,d) in ODPair:
            for n in NodeList:
                if n == o:
                    STRoadProtect.add_constraint(STRoadProtect.sum(An_ij.get((n,(i,j)), 0)\
                    * y[i,j,o,d,s] for (i,j) in edgeList) == z[o,d,s])
                    
                elif n== d:
                    STRoadProtect.add_constraint(STRoadProtect.sum(An_ij.get((n,(i,j)), 0)\
                * y[i,j,o,d,s] for (i,j) in edgeList) == -z[o,d,s])
                else:
                    STRoadProtect.add_constraint(STRoadProtect.sum(An_ij.get((n,(i,j)), 0)\
            * y[i,j,o,d,s] for (i,j) in edgeList) == 0)         
    #3     
    
    for s in SampleSpace:
        for (i,j) in edgeList:
            for (o,d) in ODPair:
                
                STRoadProtect.add_constraint(y[i,j,o,d,s] <= u[i,j,s])
    #4         
    for s in SampleSpace:
        for (i,j) in edgeList:
            for k in AssetList:
                STRoadProtect.add_constraint(u[i,j,s] <= 1- AssetArc.get(((i,j),k),0) + v[k,s])
                
#    #6
    STRoadProtect.add_constraint(STRoadProtect.sum(ProteResource.get(k, float('inf'))*\
     x[k] for k in AssetList)<= ProtectionResource)
    
    EndTime1 = datetime.datetime.now()
    print('Time to initialise model = ', EndTime1- StartTime1)
    STRoadProtect.print_information()
    return STRoadProtect, x, u, v, y
#End of STRP

def TotalTravlTime(roadNet,SourceNode, DemandNode):
#    StartTime = datetime.datetime.now()
    TotalTime = 0
    for node in SourceNode:
        sp = nx.single_source_dijkstra_path_length(roadNet, node, weight='weight')
        for i in DemandNode:
            if sp[i] < 1000:
                TotalTime += sp[i]
            else:
                TotalTime += 1000
                
            
            
    for node in DemandNode:
        sp = nx.single_source_dijkstra_path_length(roadNet, node, weight='weight')
        for i in SourceNode:
            if sp[i] < 1000:
                TotalTime += sp[i]
            else:
                TotalTime += 1000

    
    return TotalTime

#End of TotalTravTime

#Input data (files)
AssetArcFile ='C:/Users/shmhatre/Desktop/BridgeData/python file_Planning/Asset_association1.csv'
ArcFile ='C:/Users/shmhatre/Desktop/BridgeData/python file_Planning/Edgelist.csv'
AssetFile ='C:/Users/shmhatre/Desktop/BridgeData/python file_Planning/Asset.csv'
DamageProbFile ='C:/Users/shmhatre/Desktop/BridgeData/python file_Planning/DamageProb.csv'

overallStartTime = datetime.datetime.now()

LoadStartTime = datetime.datetime.now()
# Loading the input
Parameters = ParaLoading(ArcFile, AssetFile, AssetArcFile, DamageProbFile)
AssetArc = Parameters[0]
AssetList = Parameters[1]
AssetGroup = Parameters[2]
TravellingTime = Parameters[3]
DamageProb = Parameters[4]
edgeList = Parameters[5]
ProteResource = Parameters[6]
RestorationResource = Parameters[7]

SourceNode = [100, 101, 102, 103]
DemandNode = [200,201, 202, 203, 204, 205, 206, 207, 208, 209, 210,13]

ProtectionResource = 1500

seed = np.random.seed(101)
NumScenario = 1
LoadEndTime = datetime.datetime.now()

print("LoadTime =", LoadEndTime - LoadStartTime)

# Create the initial STRP model
STRP, x, u, v, y = STRPInit(SourceNode, DemandNode, NumScenario, 
                            AssetArc, AssetList, AssetGroup, TravellingTime, 
                            DamageProb, edgeList, ProteResource,RestorationResource)

# Create an initial evaluation model

# ***********************************************************************
# Start of Greedy Scemnrarion Decomposition Algorithm

# Step 1
UB = float('inf')
LB = 0
Tolerance = 0.05
S = []
optimalX = []
N = 100
Damage = ScenarioGeneration(N, DamageProb, AssetGroup)


while (UB - LB) > Tolerance*LB:
    print("#$$$$$$$$$$$$$$$$$#")
    print("UB=", UB, "LB=", LB)
    
# Step 2 *********************************************************************
    XSet = []
    TotalObj = 0
    
    
    StartTime2 = datetime.datetime.now()
    for i in range(int(N/NumScenario)):
        # Remove the previous damage scenario
#        StartTime2 = datetime.datetime.now()
        for k in AssetList:
            for s in range(NumScenario):
                CTName = "survived_k{0}_s{1}".format(k,s)
                STRP.remove_constraint(CTName)
            
    #    STRP.print_information()
            
        # Add a new damage scenario
        for k in AssetList:
            for s in range(NumScenario):
                STRP.add_constraint(v[k,s] <= x[k] + (1 - Damage.get((k,i*NumScenario+s), 0)), \
                ctname = "survived_k{0}_s{1}".format(k,s))
            
        STRP.solve()
        EndTime = datetime.datetime.now()
    
        #STRoadProtect.print_solution()
    
#        print('N =', i+1)
        X = []
        StartTime3 = datetime.datetime.now()
        for k in AssetList:
            VarName = "x_k{0}".format(k)
            value = int(STRP.get_var_by_name(VarName).solution_value)
            if value > 0:
#                print(VarName, '=',value)
                X.append(VarName)
        
        TotalObj += float(STRP.objective_value)
        
        if len(XSet) == 0:
            XSet = [X]
        else:
    #        print('XSet = ', XSet)
    #        print('X = ', X)
            XSet = union_lists(XSet, [X])

    
    print('XSet = ', XSet)
    if LB < TotalObj/N:
        LB = TotalObj/N
        print('Lower Bound =', LB)
    EndTime2 =  datetime.datetime.now() 
    print('Total time2 = ', EndTime2 - StartTime2)
# Step 3***********************************************************************
    # Create a road network
    print("#$$$$$$$$$$$$$$$$$#")
    roadNet = create_weightedgraph(TravellingTime)

    SampleSpace = list(range(N))
    
    EStartTime = datetime.datetime.now()
    for X in XSet:
#        EStartTime = datetime.datetime.now()
        TotalResource = 0
        sol ={}
        
        for varName in X:
            k =int(varName[3:])
            TotalResource += ProteResource[k]
            sol[k] = 1

#Add condition if restoration resource  plus protection resource <= 20000 (r) otherwise go to next  
       
        if (TotalResource) > ProtectionResource:
            continue
            
        TotalTvlTime = 0
        for s in SampleSpace:
            GraphCopy = roadNet.copy()
            vDamage ={}
            for k in AssetList:
                
                if(sol.get(k,0)== 0) and (Damage[k,s]== 1):
                    vDamage[k] = 0
                else:
                    vDamage[k] = 1
#            print(vDamage)   
            for key, value in AssetArc.items():
#                print("key=",key,"key[0]=",key[0],'vdamagekey[1]=',vDamage[key[1]],"key[0,0][0,1]=",key[0][0],key[0][1])
                if vDamage[key[1]] == 0:
                    GraphCopy.edges[key[0]]['weight'] = 1000
                    
            TotalTvlTime += TotalTravlTime(GraphCopy, SourceNode, DemandNode)
            
            
        obj =TotalTvlTime/N
#        EEndTime = datetime.datetime.now()
             
                
        if UB > obj:
           UB = obj
           optimalX = X
           print('Current optimal solution = ', optimalX, 'Objective value is ', UB)
    EEndTime = datetime.datetime.now()
    print('Evaluation Time = ', EEndTime- EStartTime)    

    #add constriant based on the Xset
    if (UB - LB) > Tolerance*LB:      
        for X in XSet:
            TotalResource = 0
            kSet = []
            nSet = AssetList.copy()
            
            for x_k in X:
                k = int(x_k[3:])
                TotalResource += ProteResource[k]
                
                kSet.append(int(x_k[3:]))
                nSet.remove(int(x_k[3:]))
            
            rhs = len(kSet) - 1
            if (TotalResource) < ProtectionResource:
                STRP.add_constraint(STRP.sum(x[k] for k in kSet) - STRP.sum(x[n] for n in nSet) <= rhs)
            else:
                STRP.add_constraint(STRP.sum(x[k] for k in kSet) <= rhs)
                    

overallEndTime = datetime.datetime.now()
print("total_resource=",  TotalResource)
print("Protection_resource=",ProtectionResource)
print("OverallTime =", overallEndTime - overallStartTime)

f = open("TestNCutting_R_1500_N_100_n_1_hh_101" + "output.txt", "a")
print("The optimal solution is: ",optimalX ,'Objective value is ', UB, file =f)
#print('Time to initialise model = ', EndTime1- StartTime1, file =f)
print('Total time2 = ', EndTime2 - StartTime2, file = f)
print('Evaluation Time = ', EEndTime- EStartTime, file = f)  
print("OverallTime =", overallEndTime - overallStartTime, file=f)
f.close()
            