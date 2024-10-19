# -*- coding: utf-8 -*-
# Create Time  :  2024/9/16  10:17
# Author       :  Xiaotian Yan and Wenbo Fan
# File Name    :  Flex-FBT.PY
# Kit          :  PyCharm
# Python Ver.  :  pytorch
# Description  :  stay patience and stay focus
# ********************************** #

# Required packages
import traci
import sumolib
from sumolib import checkBinary
import optparse
from gurobipy import *
import numpy as np
import pandas as pd
import math
# Input: the number of patron
passenger_number = 200
# Input: the number of vehicle
taxi_number = 27

# Define vehicle related information variables
df_car = pd.read_excel('./sumo_config/vehicle_info_homo.xlsx', sheet_name='N27')
car_hash_map = dict()  # vehicle initial position
for index, row in df_car.iterrows():
    car_hash_map[str(row['id'])] = [row['edge']]
car_hashmap = dict()  # vehicle capacity
Car_hashmap = dict()  # vehicle matching map
car_relocate = dict()  # vehicle reclocation map
car_flag = dict()  # vehicle flag:inbound:1;outbound:-1;initial:outbound
vehicle_service_nums = dict()  # Cumulative number of passengers served by the vehicle
vehicle_pick = dict()  # vehicle pick-up ids
vehicle_drop = dict()  # vehicle drop-off ids
Car_map = dict()  # vehicle check patron on-board
for i in range(taxi_number):
    car_hashmap[str(i+1)] = 0
    Car_hashmap[str(i+1)] = 1
    Car_map[str(i+1)] = -1
    car_relocate[str(i+1)] = -1
    car_flag[str(i+1)] = -1
    vehicle_service_nums[str(i+1)] = 0
    vehicle_pick[str(i+1)] = []
    vehicle_drop[str(i+1)] = []

# Define patron related information variables
df_pax = pd.read_excel('./sumo_config/pax_info_homo.xlsx', sheet_name='basic')
passenger_hash_map = dict()  # patron information:request_time,location,etc.
for index, row in df_pax.iterrows():
    passenger_hash_map[str(row['id'])] = [row['request time'],{row['edge']:row['pos']},str(row['zone'])]
passenger_hashmap = dict()  # patron status map
personStep_hashmap = dict()  # patron pick-up record
depotStep_hashmap = dict()  # patron drop-off record
for i in range(passenger_number):
    passenger_hashmap[str(i+1)] = 0
    personStep_hashmap[str(i+1)] = -1
    depotStep_hashmap[str(i+1)] = 0

terminal = 'E5103'
terminal_reverse = '-E5103'
street_space = 72.8

# Define the mapping between Junctions and downstream road segments
data=pd.read_excel("./sumo_config/JunctionToEdge.xlsx",sheet_name=0)
JunctionToEdge_hashmap=dict(zip(data['key'],data['value']))
newVehicletype = 'taxi'

# Load the network
net=sumolib.net.readNet('./sumo_config/demo_homo.net.xml')
all_of_edge = [SingleEdge.getID() for SingleEdge in net.getEdges()]
all_of_edge.remove(terminal)
all_of_edge.remove(terminal_reverse)

# Generate topology map module
def generateTopology():
    AdjacencyList = {}
    for e in net.getEdges():
        if e.allows(newVehicletype) == False:
            continue
        if AdjacencyList.__contains__(str(e.getFromNode().getID())) == False:
            AdjacencyList[str(e.getFromNode().getID())] = {}
        AdjacencyList[str(e.getFromNode().getID())][str(e.getToNode().getID())] = e.getLanes()[0].getLength()
    return AdjacencyList
AdjacencyList = generateTopology()

lenofEdges = len(net.getEdges())

# Distance calculation module
def getDistance(ef,et,ETP):
    EF = net.getEdge(ef)
    ET = net.getEdge(et)
    temp = net.getShortestPath(EF,ET)[1]-ETP[ef]-(net.getShortestPath(ET,ET)[1]-ETP[et])
    # temp=abs(net.getShortestPath(EF,ET)[1]-ETP[ef]-(472.8-ETP[et]))
    return temp

# Passenger status detection module——no usage in current version
def personcheck(pickup) :
    global passenger_hashmap
    person_net = traci.person.getIDList()
    person_net = sorted(person_net,key=lambda x: int(x[:-1]+x[-2:]))
    for each in person_net:
        if len(pickup) >= 4:
            break
        if passenger_hashmap[each] == 0:
            if each not in traci.edge.getLastStepPersonIDs(edgeID=terminal_reverse):
                pickup.append(each)
                passenger_hashmap[each] = 1
        elif passenger_hashmap[each] != 0:
            continue
    return pickup

def person2assign(zone_request):
    global passenger_hashmap
    # print('--------------------------------')
    person_net = traci.person.getIDList()
    # print(person_net)
    person_net = sorted(person_net, key=lambda x: int(x[:-1]+x[-2:]))
    for each in person_net:
        if passenger_hashmap[each] == 0:
            passenger_hashmap[each] = 1
            if each not in traci.edge.getLastStepPersonIDs(edgeID=terminal_reverse):
                if len(zone_request[passenger_hash_map[each][2]]) >= 4:
                    continue
                else:
                    zone_request[passenger_hash_map[each][2]].append(each)
                    passenger_hashmap[each] = 2
    # print(zone_request)
    return zone_request

# TSP module
def calculate(dp, countid):
    path = np.zeros((2 ** countid, countid), np.int32)
    visit = np.full((2 ** countid, countid), -1.0)
    s = 0
    for i in range(1,countid):
        s = s | (1 << i)

    distance = min_distance(s,0,matrix=dp,count=countid,paths=path,visited=visit)
    path_ret = [0]
    index = 0
    for i in range(countid - 2):
        index = path[s][index]
        path_ret.append(index)
        s = s & (~(1 << index))
    path_ret.append(countid - 1)
    return (distance, path_ret)

def min_distance(s, init, matrix,count,paths,visited):
    if visited[s][init] != -1:
        return visited[s][init]
    if s == (1 << (count - 1)):
        return matrix[count - 1][init]
    min_length = 10000000000000000000.0
    min_index = 0
    for i in range(count - 1):
        if s & (1 << i):
            s_w = s & (~(1 << i))
            ret = min_distance(s_w, i,matrix,count,paths,visited)
            if (ret + matrix[i][init]) < min_length:
                min_length = ret + matrix[i][init]
                min_index = i
    paths[s][init] = min_index
    visited[s][init] = min_length
    return min_length

# Routing_algorithm——outbound
def addRoute(passengerIDList):
    passengerEdgeList=list()
    for each in passengerIDList:
        passengerEdgeList.append(list(passenger_hash_map[each][1].keys())[0])
    passengerPosList = list()
    for each in passengerIDList:
        passengerPosList.append(list(passenger_hash_map[each][1].values())[0])
    routeid=[terminal_reverse]+passengerEdgeList+[terminal]
    posid=[0]+passengerPosList+[4886]
    EdgeToPos=dict()
    for i in range(len(routeid)):
        EdgeToPos[routeid[i]]=posid[i]
    routehash=dict()

    for i in range(len(routeid)):
        routehash[i]=routeid[i]

    dismatrix=[[0]*len(routeid) for _ in range(len(routeid))]
    for i in range(len(routeid)):
        for j in range(len(routeid)):
            if i!=j:
                dismatrix[i][j]=getDistance(routeid[i],routeid[j],EdgeToPos)
            if i==j:
                dismatrix[i][j]=0
    ret = calculate(dismatrix,len(routeid))
    print(ret)
    routeNumberid=ret[1]
    routeEdgeid=list()
    for each in routeNumberid:
        routeEdgeid.append(routehash[each])
    routePosid=list()
    for each in routeNumberid:
        routePosid.append(posid[each])
    routeEdge=list()

    for i in range(len(routeEdgeid)):
        if i<len(routeEdgeid)-2:
            x = net.getEdge(routeEdgeid[i])
            y = net.getEdge(routeEdgeid[i+1])
            routeEdge+=net.getShortestPath(x,y)[0]
            x_EdgeIDList = [SingleEdge.getID() for SingleEdge in routeEdge]
            routeEdge.pop()
        elif i==len(routeEdgeid)-2:
            x = net.getEdge(routeEdgeid[i])
            y = net.getEdge(routeEdgeid[i + 1])
            routeEdge+=net.getShortestPath(x,y)[0]
            x_EdgeIDList = [SingleEdge.getID() for SingleEdge in routeEdge]
    return x_EdgeIDList,routeEdgeid,routePosid

# Routing_algorithm——outbound+inbound
def addRoute_inbound(passengerIDList,inbound_demand):
    # inbound calculation
    inboundEdgeList=list()
    for each in inbound_demand:
        inboundEdgeList.append(list(passenger_hash_map[each][1].keys())[0])
    inboundPosList=list()
    for each in inbound_demand:
        inboundPosList.append(20)
    routeid=[terminal_reverse]+inboundEdgeList
    posid=[0]+inboundPosList
    EdgetoPos=dict()
    for i in range(len(routeid)):
        EdgetoPos[routeid[i]]=posid[i]
    routehash=dict()
    for i in range(len(routeid)):
        routehash[i]=routeid[i]
    dismatrix=[[0]*len(routeid) for _ in range(len(routeid))]
    for i in range(len(routeid)):
        for j in range(len(routeid)):
            if i!=j:
                dismatrix[i][j]=getDistance(routeid[i],routeid[j],EdgetoPos)
            if i==j:
                dismatrix[i][j]=0
    ret=calculate(dismatrix,len(routeid))
    routeNumberid=ret[1]
    routeEdgeid=list()
    for each in routeNumberid:
        routeEdgeid.append(routehash[each])
    print(routeEdgeid)
    routePosid=list()
    for each in routeNumberid:
        routePosid.append(posid[each])
    print(routePosid)
    routeEdge=list()
    for i in range(len(routeEdgeid)):
        if i<len(routeEdgeid)-2:
            x=net.getEdge(routeEdgeid[i])
            y=net.getEdge(routeEdgeid[i+1])
            routeEdge+=net.getShortestPath(x,y)[0]
            x_EdgeIDList=[SingleEdge.getID() for SingleEdge in routeEdge]
            routeEdge.pop()
        elif i==len(routeEdgeid)-2:
            x=net.getEdge(routeEdgeid[i])
            y=net.getEdge(routeEdgeid[i+1])
            routeEdge+=net.getShortestPath(x,y)[0]
            x_EdgeIDList=[SingleEdge.getID() for SingleEdge in routeEdge]
    # outbound calculation
    y_edge=x_EdgeIDList.pop()
    y_pos=routePosid.pop()
    routeEdgeid.pop()
    passengerEdgeList=list()
    for each in passengerIDList:
        passengerEdgeList.append(list(passenger_hash_map[each][1].keys())[0])
    passengerPosList=list()
    for each in passengerIDList:
        passengerPosList.append(list(passenger_hash_map[each][1].values())[0])
    Routeid=[y_edge]+passengerEdgeList+[terminal]
    print(Routeid)
    Posid=[y_pos]+passengerPosList+[4886]
    print(Posid)
    EdgeToPos=dict()
    for i in range(len(Routeid)):
        EdgeToPos[Routeid[i]]=Posid[i]
    Routehash=dict()
    for i in range(len(Routeid)):
        Routehash[i]=Routeid[i]
    Dismatrix=[[0]*len(Routeid) for _ in range(len(Routeid))]
    for i in range(len(Routeid)):
        for j in range(len(Routeid)):
            if i!=j:
                Dismatrix[i][j]=getDistance(Routeid[i],Routeid[j],EdgeToPos)
            if i==j:
                Dismatrix[i][j]=0
    Ret=calculate(Dismatrix,len(Routeid))
    RouteNumberid=Ret[1]
    RouteEdgeid=list()
    for each in RouteNumberid:
        RouteEdgeid.append(Routehash[each])
    RoutePosid=list()
    for each in RouteNumberid:
        RoutePosid.append(Posid[each])
    RouteEdge=list()
    for i in range(len(RouteEdgeid)):
        if i <len(RouteEdgeid)-2:
            x=net.getEdge(RouteEdgeid[i])
            y=net.getEdge(RouteEdgeid[i+1])
            RouteEdge+=net.getShortestPath(x,y)[0]
            X_EdgeIDList=[SingleEdge.getID() for SingleEdge in RouteEdge]
            RouteEdge.pop()
        elif i==len(RouteEdgeid)-2:
            x=net.getEdge(RouteEdgeid[i])
            y=net.getEdge(RouteEdgeid[i+1])
            RouteEdge+=net.getShortestPath(x,y)[0]
            X_EdgeIDList=[SingleEdge.getID() for SingleEdge in RouteEdge]
    x_EdgeIDList=x_EdgeIDList+X_EdgeIDList
    routeEdgeid=routeEdgeid+RouteEdgeid
    routePosid=routePosid+RoutePosid
    return x_EdgeIDList,routeEdgeid,routePosid

# Record patron boarding module
def getPersonStep(step):
    global personStep_hashmap
    car_list = traci.vehicle.getIDList()  # for key in car_hash_map.keys()
    for each in car_list:
        if len(traci.vehicle.getPersonIDList(vehID=each)) != 0:
            for i in traci.vehicle.getPersonIDList(vehID=each):
                if personStep_hashmap[i] == -1:
                    personStep_hashmap[i] = step
        elif len(traci.vehicle.getPersonIDList(vehID=each)) == 0:
            continue
    return personStep_hashmap

# Record outbound patron alighting module
def getDepotStep(step):
    global depotStep_hashmap
    passenger_list=traci.person.getIDList()
    if len(passenger_list)!=0:
        for each in passenger_list:
            if traci.person.getRoadID(personID=each)==terminal and traci.person.getLanePosition(personID=each)>4884 and traci.person.getLanePosition(personID=each)<4887 and depotStep_hashmap[each]==0:
                depotStep_hashmap[each] = step
            else:
                continue
    return depotStep_hashmap

# Record inbound patron alighting module
def inboundstep(step, Inbound):
    global depotStep_hashmap
    for each in Inbound:
        if depotStep_hashmap[each] != 0:
            continue
        else:
            if traci.person.getRoadID(personID=each) == list(passenger_hash_map[each][1].keys())[0]:
                if traci.person.getLanePosition(personID=each)<21 and traci.person.getLanePosition(personID=each)>19:
                    depotStep_hashmap[each] = step
                    Inbound.remove(each)
    return depotStep_hashmap

# inbound-match
def inbound2zone(zone_inbound_demand):
    for each in traci.edge.getLastStepPersonIDs(edgeID=terminal_reverse)[:]:
        # passenger_hashmap[each] = 1
        if passenger_hashmap[each] == 1 and each not in zone_inbound_demand[passenger_hash_map[each][2]]:
            zone_inbound_demand[passenger_hash_map[each][2]].append(each)
        elif passenger_hashmap[each] == 2 and each in zone_inbound_demand[passenger_hash_map[each][2]]:
            zone_inbound_demand[passenger_hash_map[each][2]].remove(each)
    return zone_inbound_demand

def vehiclelist(zone_vehicle_id,vehicle_online):  # check vehicle
    for key in zone_vehicle_id.keys():
        for each in zone_vehicle_id[key][:]:
            if each not in vehicle_online:
                zone_vehicle_id[key].remove(each)
    return zone_vehicle_id

# main——run SUMO
def get_options():
    optParse=optparse.OptionParser()
    optParse.add_option('--nogui',action='store_true',default=False,help='run the commandline version of sumo')
    options,args=optParse.parse_args()
    return options
def run():
    # Initialization information
    step = 0
    arrival_outcome=list()
    pickup = list()
    run_step = 360
    headway = 360
    Inbound = list()
    pick_up_list = list()
    zone_nums = 4
    zone_request = dict()
    zone_vehicle_id = dict()
    zone_inbound_demand = dict()
    zone_headway = dict()
    zone_cancel_request = dict()
    # initialization the num of vehicle in each zone
    if taxi_number == 10:
        zone_vehicle_nums = {'1': 2, '2': 3, '3': 2, '4': 3}
    elif taxi_number == 11:
        zone_vehicle_nums = {'1': 2, '2': 3, '3': 3, '4': 3}
    elif taxi_number == 12:
        zone_vehicle_nums = {'1': 2, '2': 3, '3': 3, '4': 4}
    elif taxi_number == 13:
        zone_vehicle_nums = {'1': 3, '2': 3, '3': 3, '4': 4}
    elif taxi_number == 14:
        zone_vehicle_nums = {'1': 3, '2': 4, '3': 3, '4': 4}
    elif taxi_number == 15:
        zone_vehicle_nums = {'1': 4, '2': 4, '3': 3, '4': 4}
    elif taxi_number == 16:
        zone_vehicle_nums = {'1': 4, '2': 5, '3': 3, '4': 4}
    elif taxi_number == 17:
        zone_vehicle_nums = {'1': 4, '2': 5, '3': 4, '4': 4}
    elif taxi_number == 18:
        zone_vehicle_nums = {'1': 4, '2': 5, '3': 4, '4': 5}
    elif taxi_number == 19:
        zone_vehicle_nums = {'1': 3, '2': 5, '3': 5, '4': 6}
    elif taxi_number == 20:
        zone_vehicle_nums = {'1': 3, '2': 6, '3': 5, '4': 6}
    elif taxi_number == 21:
        zone_vehicle_nums = {'1': 3, '2': 6, '3': 5, '4': 7}
    elif taxi_number == 22:
        zone_vehicle_nums = {'1': 4, '2': 5, '3': 6, '4': 7}
    elif taxi_number == 23:
        zone_vehicle_nums = {'1': 4, '2': 6, '3': 6, '4': 7}
    elif taxi_number == 24:
        zone_vehicle_nums = {'1': 5, '2': 6, '3': 6, '4': 7}
    elif taxi_number == 25:
        zone_vehicle_nums = {'1': 4, '2': 7, '3': 6, '4': 8}
    elif taxi_number == 26:
        zone_vehicle_nums = {'1': 5, '2': 7, '3': 6, '4': 8}
    elif taxi_number == 27:
        zone_vehicle_nums = {'1': 6, '2': 7, '3': 6, '4': 8}
    elif taxi_number == 28:
        zone_vehicle_nums = {'1': 7, '2': 7, '3': 6, '4': 8}
    elif taxi_number == 29:
        zone_vehicle_nums = {'1': 7, '2': 8, '3': 6, '4': 8}
    elif taxi_number == 30:
        zone_vehicle_nums = {'1': 7, '2': 7, '3': 7, '4': 9}
    elif taxi_number == 31:
        zone_vehicle_nums = {'1': 7, '2': 8, '3': 7, '4': 9}
    elif taxi_number == 32:
        zone_vehicle_nums = {'1': 9, '2': 7, '3': 7, '4': 9}
    elif taxi_number == 33:
        zone_vehicle_nums = {'1': 9, '2': 8, '3': 7, '4': 9}
    elif taxi_number == 34:
        zone_vehicle_nums = {'1': 9, '2': 8, '3': 8, '4': 9}
    elif taxi_number == 35:
        zone_vehicle_nums = {'1': 9, '2': 8, '3': 8, '4': 10}
    elif taxi_number == 36:
        zone_vehicle_nums = {'1': 10, '2': 8, '3': 8, '4': 10}
    elif taxi_number == 9:
        zone_vehicle_nums = {'1': 2, '2': 2, '3': 2, '4': 3}
    elif taxi_number == 8:
        zone_vehicle_nums = {'1': 1, '2': 2, '3': 2, '4': 3}
    elif taxi_number == 7:
        zone_vehicle_nums = {'1': 1, '2': 2, '3': 2, '4': 2}
    elif taxi_number == 6:
        zone_vehicle_nums = {'1': 1, '2': 2, '3': 1, '4': 2}
    for i in range(zone_nums):
        zone_request[str(i + 1)] = []
        zone_vehicle_id[str(i + 1)] = []
        zone_inbound_demand[str(i + 1)] = []
        zone_headway[str(i + 1)] = 360
        zone_cancel_request[str(i + 1)] = 0
    tolerance_time = 360
    end_time = 8000
    count1 = 0
    remove_pax_id = []
    while traci.simulation.getTime() < end_time:
        traci.simulationStep()
        # dispatch
        PersonStep = getPersonStep(step)
        print('patron boarding information', PersonStep)
        for each1 in list(traci.person.getIDList()):
            if each1 not in traci.edge.getLastStepPersonIDs(edgeID=terminal_reverse) and passenger_hashmap[each1] == 1:
                if math.floor(step-passenger_hash_map[each1][0]) >= tolerance_time:
                    traci.person.remove(personID=each1)
                    zone_cancel_request[passenger_hash_map[each1][2]] = zone_cancel_request[
                                                                            passenger_hash_map[each1][2]] + 1
                    passenger_hashmap[each1] = 3
                    count1 = count1 + 1
                    remove_pax_id.append(each1)
        print(f"decline request {count1}")
        print(f"zone cancel request {zone_cancel_request}")
        # print(f"remove_pax {remove_pax_id}")
        # for key, value in zone_request.items():
        #     for pax_id in zone_request[key]:
        #         if math.floor(step-passenger_hash_map[pax_id][0]) >= tolerance_time:
        #             passenger_hashmap[pax_id] = 3  # remove this pax
        #             traci.person.remove(personID=pax_id)
        #             zone_request[key].remove(pax_id)
        zone_request = person2assign(zone_request)  # 乘客名单
        print('outbound match', zone_request)
        print('patron state', passenger_hashmap)
        zone_inbound = inbound2zone(zone_inbound_demand)
        print('inbound match', zone_inbound)
        Inbound = list(set(Inbound))
        InboundStep = inboundstep(step, Inbound)
        print('inbound arrival information', InboundStep)
        print('dispatch interval', zone_headway)
        for key in zone_request.keys():
            # print('路网中的车辆',traci.vehicle.getIDList())
            # zone_vehicle_id = vehiclelist(zone_vehicle_id, traci.vehicle.getIDList())
            # print('各分区的车辆使用情况', zone_vehicle_id)
            if len(zone_vehicle_id[key]) < zone_vehicle_nums[key]:
                print(key + 'zone has available vehicles')
                if len(zone_request[key]) == 4:
                    print('reach full-match')
                    print('outbound request', zone_request[key])
                    if len(zone_inbound[key]) != 0:
                        if len(zone_inbound[key]) > 4:
                            inbound_demand = zone_inbound[key][0:4]
                        else:
                            inbound_demand = zone_inbound[key]
                        for each2 in inbound_demand:
                            passenger_hashmap[each2] = 2
                            Inbound.append(each2)
                        route, routeEdgeid, routePosid = addRoute_inbound(zone_request[key], inbound_demand)
                        print(route)
                        print(routeEdgeid)
                        print(routePosid)
                        traci.route.add(routeID=str(step)+key, edges=route)
                        traci.vehicle.add(vehID=str(step)+key, routeID=str(step)+key, typeID="DEFAULT_VEHTYPE", depart=step,
                                          line="taxi", personCapacity=5, personNumber=0)
                        zone_vehicle_id[key].append(str(step)+key)
                        traci.vehicle.setStop(vehID=str(step)+key, edgeID=terminal_reverse, pos=20 + int(key) * 20, laneIndex=0,
                                              duration=3,
                                              startPos=20 + int(key) * 20)
                        for i in range(1, len(routeEdgeid)):
                            traci.vehicle.setStop(vehID=str(step)+key, edgeID=routeEdgeid[i], pos=routePosid[i],
                                                  laneIndex=0, duration=3, startPos=routePosid[i])
                        run_step = step + headway
                        zone_headway[key] = run_step
                        zone_request[key] = []
                    else:
                        route, routeEdgeid, routePosid = addRoute(zone_request[key])
                        print(route)
                        print(routeEdgeid)
                        print(routePosid)
                        traci.route.add(routeID=str(step)+key, edges=route)
                        traci.vehicle.add(vehID=str(step)+key, routeID=str(step)+key, typeID="DEFAULT_VEHTYPE", depart=step,
                                          line="taxi", personCapacity=5, personNumber=0)
                        zone_vehicle_id[key].append(str(step)+key)
                        for i in range(1, len(routeEdgeid)):
                            traci.vehicle.setStop(vehID=str(step)+key, edgeID=routeEdgeid[i], pos=routePosid[i],
                                                  laneIndex=0, duration=3, startPos=routePosid[i])
                        run_step = step + headway
                        zone_headway[key] = run_step
                        zone_request[key] = []
                elif step >= zone_headway[key]:
                    print('reach dispatch interval')
                    if len(zone_request[key]) > 0:
                        print('have request to pick up', zone_request[key])
                        if len(zone_inbound[key]) != 0:
                            if len(zone_inbound[key]) > 4:
                                inbound_demand = zone_inbound[key][0:4]
                            else:
                                inbound_demand = zone_inbound[key]
                            for each3 in inbound_demand:
                                passenger_hashmap[each3] = 2
                                Inbound.append(each3)
                            route, routeEdgeid, routePosid = addRoute_inbound(zone_request[key], inbound_demand)
                            print(route)
                            print(routeEdgeid)
                            print(routePosid)
                            traci.route.add(routeID=str(step)+key, edges=route)
                            traci.vehicle.add(vehID=str(step)+key, routeID=str(step)+key, typeID="DEFAULT_VEHTYPE", depart=step,
                                              line="taxi", personCapacity=5, personNumber=0)
                            zone_vehicle_id[key].append(str(step)+key)
                            traci.vehicle.setStop(vehID=str(step)+key, edgeID=terminal_reverse, pos=20 + int(key) * 20, laneIndex=0,
                                                  duration=3,
                                                  startPos=20 + int(key) * 20)
                            for i in range(1, len(routeEdgeid)):
                                traci.vehicle.setStop(vehID=str(step)+key, edgeID=routeEdgeid[i], pos=routePosid[i],
                                                      laneIndex=0, duration=3, startPos=routePosid[i])
                            run_step = step + headway
                            zone_headway[key] = run_step
                            zone_request[key] = []
                        else:
                            print('no inbound request')
                            route, routeEdgeid, routePosid = addRoute(zone_request[key])
                            print(route)
                            print(routeEdgeid)
                            print(routePosid)
                            traci.route.add(routeID=str(step)+key, edges=route)
                            traci.vehicle.add(vehID=str(step)+key, routeID=str(step)+key, typeID="DEFAULT_VEHTYPE", depart=step,
                                              line="taxi", personCapacity=5, personNumber=0)
                            zone_vehicle_id[key].append(str(step)+key)
                            for i in range(1, len(routeEdgeid)):
                                traci.vehicle.setStop(vehID=str(step)+key, edgeID=routeEdgeid[i], pos=routePosid[i],
                                                      laneIndex=0, duration=3, startPos=routePosid[i])
                            run_step = step + headway
                            zone_headway[key] = run_step
                            zone_request[key] = []
                    else:
                        print('have no request to pick up, please continue waiting')

            else:
                print('this zone has no available vehicle')
        print('the number of vehicle is dispatch in the zone', zone_vehicle_id)
        hub_edge = traci.edge.getLastStepVehicleIDs(terminal)
        if len(hub_edge) != 0:
            for i in range(len(hub_edge)):
                if traci.vehicle.getLanePosition(vehID=hub_edge[i]) < 4885 and traci.vehicle.getLanePosition(
                        vehID=hub_edge[i]) > 4882:
                    temp = list(traci.vehicle.getPersonIDList(vehID=hub_edge[i]))
                elif traci.vehicle.getLanePosition(vehID=hub_edge[i]) < 4888 and traci.vehicle.getLanePosition(
                        vehID=hub_edge[i]) > 4885:
                    temp = list(traci.vehicle.getPersonIDList(vehID=hub_edge[i]))
                    temp = temp + [step]
                    if hub_edge[i] in zone_vehicle_id[hub_edge[i][-1]]:
                        zone_vehicle_id[hub_edge[i][-1]].remove(hub_edge[i])
                    arrival_outcome.append(temp)
        print('arrival information', arrival_outcome)
        DepotStep = getDepotStep(step)
        print('patron arrival information', DepotStep)

        print(f"pickup list {pick_up_list}")
        step+=1
    traci.close()
    sys.exit()

if __name__=='__main__':
    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
        traci.start([sumoBinary, '-c', './sumo_config/demo_homo.sumocfg',
                     '--quit-on-end',
                     '--gui-settings-file', './sumo_config/viewsettings.view.xml'])  # ,'--quit-on-end'
        # traci.start([sumoBinary, "-c", "data/cross.sumocfg","--tripinfo-output", "tripinfo.xml", "--start"])
    run()