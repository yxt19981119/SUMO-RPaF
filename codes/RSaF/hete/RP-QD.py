#!/usr/bin/env python
# -*- coding:utf-8 -*-
# author: Xiaotian Yan and Wenbo Fan
# datetime:2023/3/5 13:49
# software: PyCharm
# strategy: RP-QD & FCFS & RP_size = 4
# Required packages
import random
import traci
import sumolib
from sumolib import checkBinary
import optparse
from gurobipy import *
import numpy as np
import pandas as pd

# Input: the number of patron
passenger_number = 200
# Input: the number of vehicle
# taxi_number=10
# Define vehicle related information variables
df_car = pd.read_excel('./sumo_config/vehicle_info_hete.xlsx', sheet_name='N25')  # option [fleet size]
taxi_number = len(df_car['id'])
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
df_pax = pd.read_excel('./sumo_config/pax_info_hete.xlsx', sheet_name='basic')  # option [demand level]
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

zone_matching = dict()
zone1 = pd.read_excel('./sumo_config/Distance-decaying distribution.xls', sheet_name='1')
zone2 = pd.read_excel('./sumo_config/Distance-decaying distribution.xls', sheet_name='2')
zone3 = pd.read_excel('./sumo_config/Distance-decaying distribution.xls', sheet_name='3')
zone4 = pd.read_excel('./sumo_config/Distance-decaying distribution.xls', sheet_name='4')
for i in range(4):
    if i == 0:
        zone_matching[str(i+1)]=[]
        for each in zone1['edge']:
            zone_matching[str(i + 1)].append(each)
    if i == 1:
        zone_matching[str(i + 1)] = []
        for each in zone2['edge']:
            zone_matching[str(i + 1)].append(each)
    if i == 2:
        zone_matching[str(i + 1)] = []
        for each in zone3['edge']:
            zone_matching[str(i + 1)].append(each)
    if i == 3:
        zone_matching[str(i + 1)] = []
        for each in zone4['edge']:
            zone_matching[str(i + 1)].append(each)
df_zone = df_car['zone']
RPV2Zone = dict()
Zone2RPV = {'1':[],'2':[],'3':[],'4':[]}
for i in range(taxi_number):
    RPV2Zone[str(i+1)] = str(df_zone[i])
    Zone2RPV[str(df_zone[i])].append(str(i+1))
# RPV2Zone = {'1':'1','2':'1','3':'2','4':'2','5':'2','6':'3','7':'3','8':'4','9':'4','10':'4'}
# Zone2RPV = {'1':['1','2'],'2':['3','4','5'],'3':['6','7'],'4':['8','9','10']}

terminal = 'E5103'
terminal_reverse = '-E5103'
street_space = 72.8

# Define the mapping between Junctions and downstream road segments
data = pd.read_excel("./sumo_config/JunctionToEdge.xlsx",sheet_name=0)
JunctionToEdge_hashmap=dict(zip(data['key'],data['value']))
newVehicletype = 'taxi'

# Load the network
net=sumolib.net.readNet('./sumo_config/demo_hete.net.xml')
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

# Passenger status detection module
def checkpassenger(zone_request):
    global passenger_hashmap
    person_net = list(traci.person.getIDList())
    person_net = sorted(person_net, key=lambda x: int(x[:-1]+x[-2:]))
    for each in person_net:
        if passenger_hashmap[each]==0 and each not in traci.edge.getLastStepPersonIDs(edgeID=terminal_reverse):
            passenger_hashmap[each]=1
            zone_request[passenger_hash_map[each][2]].append(each)
        elif passenger_hashmap[each] !=0:
            continue
    return passenger_hashmap, zone_request

# Vehicle capacity detection module
def checkcar():
    global car_hashmap
    for key in car_hash_map.keys():
        car_hashmap[key]=len(car_hash_map[key])
    return car_hashmap

# Distance calculation module
def getDistance(ef,et,ETP):
    if ef[0] == ':':
        ef = JunctionToEdge_hashmap[ef]
        EF = net.getEdge(ef)
    else:
        EF = net.getEdge(ef)
    if et[0] == ':':
        et = JunctionToEdge_hashmap[et]
        ET = net.getEdge(et)
    else:
        ET = net.getEdge(et)
    temp=net.getShortestPath(EF,ET)[1]-ETP[ef]-(street_space-ETP[et])
    return temp

# TSP module
def calculate(dp,countid):
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
    min_length = 1000000000000000.0
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

# matching_algorithm
def distribute(zone_request):
    p,zone_request = checkpassenger(zone_request)
    print('patron state', p)
    print('patron request across the network', zone_request)
    for key in p.keys():
        if p[key] == 1:  # patron state
            zone_id = passenger_hash_map[key][2]
            c = checkcar()
            exit_flag = False
            for each in Zone2RPV[zone_id]:
                if c[each] >= 5:
                    exit_flag = True
                else:
                    exit_flag = False
                    break
            if exit_flag == True:
                print('there is no available vehicle')

            else:
                dis = np.zeros([1,len(Zone2RPV[zone_id])])
                temp_hashmap = dict()
                for index,each in enumerate(Zone2RPV[zone_id]):
                    temp_hashmap[index] = each
                    if c[each]<5:
                        person_edge = list(passenger_hash_map[key][1].keys())[0]
                        person_pos = list(passenger_hash_map[key][1].values())[0]
                        car_edge = traci.vehicle.getRoadID(each)
                        car_pos = traci.vehicle.getLanePosition(each)
                        if car_edge[0] == ':':
                            car_edge=JunctionToEdge_hashmap[car_edge]
                            car_pos=0
                        temp_distance = net.getShortestPath(net.getEdge(car_edge),net.getEdge(person_edge))[1]
                        dis[0][index] = temp_distance-car_pos-(street_space-person_pos)
                    elif c[each] >= 5:
                        dis[0][index] = 1e10
                temp = int(np.argmin(dis[0]))
                car_hash_map[temp_hashmap[temp]].append(key)
                p[key] = 2
                if len(car_hash_map[temp_hashmap[temp]]) == 5:
                    print('this vehicle has full-match')
                    continue
        elif p[key] != 1:
            continue
    return car_hash_map

# Routing_algorithm——outbound
def addRoute(edgelist):
    passengerEdgeList = list()
    for each in edgelist[1:len(edgelist)]:
        passengerEdgeList.append(list(passenger_hash_map[each][1].keys())[0])
        print(passengerEdgeList)
    passengerPosList = list()
    for each in edgelist[1:len(edgelist)]:
        passengerPosList.append(list(passenger_hash_map[each][1].values())[0])
    routeid = passengerEdgeList+[terminal]
    posid = passengerPosList + [4886]
    EdgeToPos = dict()
    for i in range(len(routeid)):
        EdgeToPos[routeid[i]] = posid[i]
    routehash=dict()
    for i in range(len(routeid)):
        routehash[i]=routeid[i]
    dismatrix=[[0]*len(routeid) for _ in range(len(routeid))]
    for i in range(len(routeid)):
        for j in range(len(routeid)):
            if i != j:
                dismatrix[i][j] = getDistance(routeid[i],routeid[j],EdgeToPos)
            if i == j:
                dismatrix[i][j] = 0
    ret = calculate(dismatrix, len(routeid))
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
            if i==0 and routeEdgeid[i] == routeEdgeid[i+1]:
                x=net.getEdge(routeEdgeid[i])
                if routeEdgeid[i][0] == '-':
                    y = net.getEdge(routeEdgeid[i + 1].strip('-'))
                else:
                    y = net.getEdge('-' + routeEdgeid[i + 1])
                routeEdge += net.getShortestPath(x, y)[0]
                x_EdgeIDList = [SingleEdge.getID() for SingleEdge in routeEdge]
            else:
                x = net.getEdge(routeEdgeid[i])
                y = net.getEdge(routeEdgeid[i+1])
                routeEdge+=net.getShortestPath(x,y)[0]
                x_EdgeIDList = [SingleEdge.getID() for SingleEdge in routeEdge]
                routeEdge.pop()
        elif i==len(routeEdgeid)-2:
            if i==0 and routeEdgeid[i] == routeEdgeid[i+1]:
                x = net.getEdge(routeEdgeid[i])
                if routeEdgeid[i][0] == '-':
                    y = net.getEdge(routeEdgeid[i + 1].strip('-'))
                else:
                    y = net.getEdge('-' + routeEdgeid[i + 1])
                routeEdge += net.getShortestPath(x, y)[0]
                x_EdgeIDList = [SingleEdge.getID() for SingleEdge in routeEdge]
            else:
                x = net.getEdge(routeEdgeid[i])
                y = net.getEdge(routeEdgeid[i + 1])
                routeEdge+=net.getShortestPath(x,y)[0]
                x_EdgeIDList = [SingleEdge.getID() for SingleEdge in routeEdge]
    return x_EdgeIDList,routeEdgeid,routePosid

# Routing_algorithm——inbound
def addRoute_inbound(inbound_demand):
    inboundEdgeList=list()
    for each in inbound_demand:
        inboundEdgeList.append(list(passenger_hash_map[each][1].keys())[0])
    inboundPosList=list()
    for each in inbound_demand:
        inboundPosList.append(20)
    routeid = [terminal_reverse] + inboundEdgeList
    posid = [500] + inboundPosList
    EdgetoPos = dict()
    for i in range(len(routeid)):
        EdgetoPos[routeid[i]] = posid[i]  # edge-pos hashmap
    routehash = dict()
    for i in range(len(routeid)):
        routehash[i] = routeid[i]  # number-edge hashmap
    dismatrix = [[0] * len(routeid) for _ in range(len(routeid))]
    for i in range(len(routeid)):
        for j in range(len(routeid)):
            if i != j:
                dismatrix[i][j] = getDistance(routeid[i], routeid[j], EdgetoPos)
            if i == j:
                dismatrix[i][j] = 0
    ret = calculate(dismatrix, len(routeid))
    routeNumberid = ret[1]
    routeEdgeid = list()
    for each in routeNumberid:
        routeEdgeid.append(routehash[each])
    #print(routeEdgeid)
    routePosid = list()
    for each in routeNumberid:
        routePosid.append(posid[each])
    #print(routePosid)
    routeEdge = list()
    x_EdgeIDList = list()
    for i in range(len(routeEdgeid)):
        if i < len(routeEdgeid) - 2:
            if i==0 and routeEdgeid[i] == routeEdgeid[i + 1]:
                x = net.getEdge(routeEdgeid[i])
                if routeEdgeid[i][0] == '-':
                    y = net.getEdge(routeEdgeid[i + 1].strip('-'))
                else:
                    y = net.getEdge('-'+routeEdgeid[i + 1])
                routeEdge += net.getShortestPath(x, y)[0]
                x_EdgeIDList = [SingleEdge.getID() for SingleEdge in routeEdge]
                print(x_EdgeIDList)
            else:
                x = net.getEdge(routeEdgeid[i])
                y = net.getEdge(routeEdgeid[i + 1])
                if y.getID() in x_EdgeIDList:
                    temp=routeEdgeid[i]
                    routeEdgeid[i] = routeEdgeid[i+1]
                    routeEdgeid[i+1] = temp
                    continue
                routeEdge += net.getShortestPath(x, y)[0]
                x_EdgeIDList = [SingleEdge.getID() for SingleEdge in routeEdge]
                routeEdge.pop()
        elif i == len(routeEdgeid) - 2:
            if i == 0 and routeEdgeid[i] == routeEdgeid[i + 1]:
                x = net.getEdge(routeEdgeid[i])
                if routeEdgeid[i][0] == '-':
                    y = net.getEdge(routeEdgeid[i].strip('-'))
                else:
                    y = net.getEdge('-' + routeEdgeid[i])
                routeEdge += net.getShortestPath(x, y)[0]
                x_EdgeIDList = [SingleEdge.getID() for SingleEdge in routeEdge]
            else:
                x = net.getEdge(routeEdgeid[i])
                y = net.getEdge(routeEdgeid[i + 1])
                if y.getID() in x_EdgeIDList:
                    temp = routeEdgeid[i]
                    routeEdgeid[i] = routeEdgeid[i+1]
                    routeEdgeid[i+1] = temp
                routeEdge += net.getShortestPath(x, y)[0]
                x_EdgeIDList = [SingleEdge.getID() for SingleEdge in routeEdge]
    return x_EdgeIDList,routeEdgeid,routePosid

# Record patron boarding module
def getPersonStep(step):
    global personStep_hashmap
    for key in car_hash_map.keys():
        if len(traci.vehicle.getPersonIDList(vehID=key)) != 0:
            for each in traci.vehicle.getPersonIDList(vehID=key):
                if personStep_hashmap[each] == -1:
                    personStep_hashmap[each] = step
                else:
                    continue
        else:
            continue
    return personStep_hashmap

# Record outbound patron alighting module
def getDepotStep(step):
    global depotStep_hashmap
    passenger_list=traci.person.getIDList()
    if len(passenger_list)!=0:
        for each in passenger_list:
            if traci.person.getRoadID(personID=each)==terminal and  traci.person.getLanePosition(personID=each)>4884 and traci.person.getLanePosition(personID=each)<4887 and depotStep_hashmap[each]==0:
                depotStep_hashmap[each]=step
            else:
                continue
    return depotStep_hashmap

# Record inbound patron alighting module
def inboundStep(step,key):
    for each in traci.vehicle.getPersonIDList(vehID=key):
        if depotStep_hashmap[each] == 0 and traci.vehicle.getRoadID(vehID=key) == list(passenger_hash_map[each][1].keys())[0]:
            if traci.vehicle.getLanePosition(vehID=key)<21 and traci.vehicle.getLanePosition(vehID=key)>19:
                depotStep_hashmap[each] = step
    return depotStep_hashmap

def urgent(request,vehicle,step):
    V = 35/3.6
    unmatched_time = step - passenger_hash_map[request][0]
    et = list(passenger_hash_map[request][1].keys())[0]
    ET = net.getEdge(et)
    ef = traci.vehicle.getRoadID(vehID=vehicle)
    if ef[0]==':':
        ef = JunctionToEdge_hashmap[ef]
        EF = net.getEdge(ef)
    else:
        EF = net.getEdge(ef)

    distance_time = (net.getShortestPath(EF,ET)[1]-traci.vehicle.getLanePosition(vehID=vehicle)-(street_space-list(passenger_hash_map[request][1].values())[0]))/V
    return unmatched_time-distance_time

def reposition_algorithm(key,step,unmatched_request):
    if len(unmatched_request) == 0:
        return
    else:
        zone_id = RPV2Zone[key]
        unmatched_request_zone = []
        for each in unmatched_request:
            if passenger_hash_map[each][2] == zone_id:
                unmatched_request_zone.append(each)
        if len(unmatched_request_zone) == 0:
            return []
        urgent_value = [urgent(request, vehicle=key, step=step) for request in unmatched_request_zone]
        return unmatched_request_zone[urgent_value.index(max(urgent_value))]


# main——run SUMO
def get_options():
    optParse=optparse.OptionParser()
    optParse.add_option('--nogui',action='store_true',default=False,help='run the commandline version of sumo')
    options,args=optParse.parse_args()
    return options
def run():
    # Initialization information
    step = 0
    arrival_outcome = list()
    # pick_up_list = {'1': [], '2': [], '3': [], '4': [], '5': [], '6': [], '7': [], '8': [], '9': [], '10': []}
    pick_up_list = dict()
    car_stop = dict()
    for i in range(taxi_number):
        pick_up_list[str(i + 1)] = []
        car_stop[str(i + 1)] = 20
    # car_stop = {'1': 20, '2': 20, '3': 20, '4': 20, '5': 20, '6': 20, '7': 20, '8': 20, '9': 20, '10': 20}

    zone_nums = 4
    zone_request = dict()
    zone_vehicle_id = dict()
    zone_inbound_demand = dict()
    zone_headway = dict()
    zone_cancel_request = dict()
    zone_vehicle_nums = {'1': 3, '2': 2, '3': 2, '4': 3}
    for i in range(zone_nums):
        zone_request[str(i + 1)] = []
        zone_vehicle_id[str(i + 1)] = []
        zone_inbound_demand[str(i + 1)] = []
        zone_headway[str(i + 1)] = 560
        zone_cancel_request[str(i + 1)] = 0
    tolerance_time = 360
    end_time = 8000
    count1 = 0
    while traci.simulation.getTime() < end_time:
        traci.simulationStep()
        global passenger_hashmap
        global car_hash_map
        global Car_hashmap
        global car_flag
        global Car_map
        global car_hashmap
        # step=0
        if step == 0:
            for key in car_hash_map.keys():
                edge_temp = net.getEdge(car_hash_map[key][0]).getID()
                # print(edge_temp)
                traci.route.add(routeID=key, edges=[edge_temp])
                traci.vehicle.add(vehID=key, routeID=key, typeID="DEFAULT_VEHTYPE", depart=step, line='taxi',
                                  personCapacity=5, personNumber=0)

        for each1 in list(traci.person.getIDList()):
            if each1 not in traci.edge.getLastStepPersonIDs(edgeID=terminal_reverse) and passenger_hashmap[each1] == 1:
                if math.floor(step-passenger_hash_map[each1][0]) >= tolerance_time:
                    traci.person.remove(each1)
                    passenger_hashmap[each1] = 3
                    zone_cancel_request[passenger_hash_map[each1][2]] = zone_cancel_request[passenger_hash_map[each1][2]]+1
                    count1 = count1 + 1
        print(f"cancel request {count1}")
        print(f"zone cancel request {zone_cancel_request}")
        # update dynamically
        Car_hash_map = distribute(zone_request)
        print('vehicle-patron-matching', Car_hash_map)
        #
        print('patron state', passenger_hashmap)
        #
        print('vehicle flag', car_flag)
        #
        PersonStep = getPersonStep(step)
        print('patron boarding information', PersonStep)
        for key in Car_hash_map.keys():
            if car_relocate[key] == 1:
                if traci.vehicle.getRoadID(vehID=key) == car_hash_map[key][0]:
                    if car_hash_map[key][1] == '1':
                        car_hash_map[key] = car_hash_map[key][0:1]
                    elif car_hash_map[key][1] != '1':
                        car_hash_map[key] = car_hash_map[key][0:2]
                    car_relocate[key] = -1
                    # traci.vehicle.setStop(vehID=key, edgeID=net.getEdge(Car_hash_map[key][0]).getID(), pos=40,laneIndex=0, startPos=40, duration=2)
            else:
                if car_flag[key] == -1:  # outbound车辆状态指令
                    if len(Car_hash_map[key]) == Car_hashmap[key] == 1:  # 车辆没有被分配到乘客
                        traci.vehicle.setStop(vehID=key, edgeID=net.getEdge(Car_hash_map[key][0]).getID(), pos=car_stop[key],
                                              laneIndex=0, startPos=car_stop[key], duration=2)
                        print('taxi%s not assigned' % key)
                        continue
                    elif Car_hashmap[key] == 1 and len(Car_hash_map[key]) > Car_hashmap[key]:  # 一旦车辆出现分配
                        print('taxi%s is assigned and ready to dispatch' % key)
                        traci.vehicle.changeTarget(vehID=key, edgeID=
                        list(passenger_hash_map[Car_hash_map[key][Car_hashmap[key]]][1].keys())[0])
                        next_edge = traci.edge.getLastStepVehicleIDs(
                            list(passenger_hash_map[Car_hash_map[key][1]][1].keys())[0])
                        # print(list(passenger_hash_map[Car_hash_map[key][Car_hashmap[key]]][1].keys())[0])
                        if key in next_edge:
                            traci.vehicle.setStop(vehID=key, edgeID=
                            list(passenger_hash_map[Car_hash_map[key][Car_hashmap[key]]][1].keys())[0],
                                                  pos=list(passenger_hash_map[Car_hash_map[key][Car_hashmap[key]]][
                                                               1].values())[0], laneIndex=0,
                                                  startPos=list(passenger_hash_map[Car_hash_map[key][Car_hashmap[key]]][
                                                                    1].values())[0], duration=5)
                            if PersonStep[Car_hash_map[key][Car_hashmap[key]]] != -1:
                                Car_hashmap[key] = Car_hashmap[key] + 1
                    elif len(Car_hash_map[key]) == Car_hashmap[key] == 2:
                        # unavailable
                        count = len(Car_hash_map[key])
                        car_hash_map[key][0] = list(passenger_hash_map[car_hash_map[key][count-1]][1].keys())[0]
                        empty_seats = 5 - len(Car_hash_map[key])
                        for i in range(empty_seats):
                            car_hash_map[key].append(str(i))
                        Car_hashmap[key] = len(Car_hash_map[key])
                        pick_up_list[key].append(Car_hash_map[key])
                        print('taxi%s is assigned and no new matched requests, go directly to the hub' % key)
                        traci.vehicle.changeTarget(vehID=key, edgeID=terminal)
                        traci.vehicle.setStop(vehID=key, edgeID=terminal, pos=4886, laneIndex=0, startPos=4886, duration=3)
                    elif len(Car_hash_map[key]) > 2 and len(Car_hash_map[key]) > Car_hashmap[key] and Car_hashmap[
                        key] == 2 and key in traci.edge.getLastStepVehicleIDs(
                            list(passenger_hash_map[Car_hash_map[key][1]][1].keys())[0]):
                        print('taxi%s is assigned and occur new matched requests, pick up' % key)
                        route, routeEdgeid, routePosid = addRoute(Car_hash_map[key])
                        print('vehicle route information', route)
                        print(routeEdgeid)
                        print(routePosid)
                        pick_up_list[key].append(Car_hash_map[key])
                        traci.vehicle.setRoute(vehID=key, edgeList=route)
                        for i in range(1, len(routeEdgeid)):
                            traci.vehicle.setStop(vehID=key, edgeID=routeEdgeid[i], pos=routePosid[i], laneIndex=0,
                                                  duration=3, startPos=routePosid[i])
                        count = len(Car_hash_map[key])
                        car_hash_map[key][0] = list(passenger_hash_map[car_hash_map[key][count - 1]][1].keys())[0]
                        empty_seats = 5 - len(Car_hash_map[key])
                        if empty_seats > 0:
                            for i in range(empty_seats):
                                car_hash_map[key].append(str(i))
                        Car_hashmap[key] = len(Car_hash_map[key])
                    print('vehicle capacity information', Car_hashmap)
                    # arrival the hub
                    hub_edge = traci.edge.getLastStepVehicleIDs(terminal)
                    if key in hub_edge:
                        if traci.vehicle.getLanePosition(vehID=key) < 4885 and traci.vehicle.getLanePosition(
                                vehID=key) > 4882:
                            temp = list(traci.vehicle.getPersonIDList(vehID=key))
                        elif traci.vehicle.getLanePosition(vehID=key) < 4888 and traci.vehicle.getLanePosition(
                                vehID=key) > 4885:
                            temp = list(traci.vehicle.getPersonIDList(vehID=key))
                            temp = temp + [step]
                            arrival_outcome.append(temp)
                            print('arriving hub information', arrival_outcome)
                            # temp_edge = all_of_edge[random.randint(0, len(all_of_edge) - 1)]
                            traci.vehicle.changeTarget(vehID=key, edgeID=terminal_reverse)
                            traci.vehicle.setStop(vehID=key, edgeID=terminal_reverse, pos=20 + 20 * int(RPV2Zone[key]),
                                                  laneIndex=0, startPos=20 + 20 * int(RPV2Zone[key]), duration=3)
                            # traci.vehicle.setStop(vehID=key, edgeID=temp_edge, pos=10, laneIndex=0, startPos=10,duration=2)

                            if car_flag[key] == -1:
                                car_hash_map[key] = [car_hash_map[key][0],'2','3','4','5']
                                car_hashmap[key] = 5
                                car_flag[key] = 1
                            if Car_map[key] != -1:
                                Car_map[key] = -1
                elif car_flag[key] == 1:  # inbound
                    hub_edge_inverse = traci.edge.getLastStepVehicleIDs(terminal_reverse)
                    if key in hub_edge_inverse:
                        if traci.vehicle.getLanePosition(vehID=key) > 501 and Car_map[key] == -1:
                            inbound_demand = traci.vehicle.getPersonIDList(vehID=key)
                            if len(inbound_demand) != 0:
                                for each in inbound_demand:
                                    passenger_hashmap[each] = 2
                                route, routeEdgeid, routePosid = addRoute_inbound(inbound_demand)
                                print(route)
                                print(routeEdgeid)
                                print(routePosid)
                                traci.vehicle.setRoute(vehID=key, edgeList=route)
                                for i in range(1, len(routeEdgeid)):
                                    traci.vehicle.setStop(vehID=key, edgeID=routeEdgeid[i], pos=routePosid[i],
                                                          laneIndex=0, duration=3, startPos=routePosid[i])
                                car_hash_map[key][0] = routeEdgeid[-1]
                                Car_map[key] = 1
                                Car_hashmap[key] = 1
                            else:
                                car_flag[key] = -1
                                unmatched_request = [key for key, value in passenger_hashmap.items() if value == 1]
                                if len(unmatched_request) == 0:
                                    temp_edge = car_hash_map[key][0]
                                    traci.vehicle.changeTarget(vehID=key, edgeID=temp_edge)
                                    traci.vehicle.setStop(vehID=key, edgeID=temp_edge,
                                                          pos=20,
                                                          laneIndex=0, startPos=20, duration=3)
                                    car_stop[key] = 20
                                    car_hash_map[key] = [net.getEdge(temp_edge).getID(), '1', '2', '3', '4']
                                else:
                                    urgent_request = reposition_algorithm(key, step, unmatched_request)
                                    if urgent_request == []:
                                        temp_edge = car_hash_map[key][0]
                                        traci.vehicle.changeTarget(vehID=key, edgeID=temp_edge)
                                        traci.vehicle.setStop(vehID=key, edgeID=temp_edge,
                                                              pos=20,
                                                              laneIndex=0, startPos=20, duration=3)
                                        car_stop[key] = 20
                                        car_hash_map[key] = [net.getEdge(temp_edge).getID(), '1', '2', '3', '4']
                                    else:
                                        passenger_hashmap[urgent_request] = 2
                                        temp_edge = list(passenger_hash_map[urgent_request][1].keys())[0]
                                        traci.vehicle.changeTarget(vehID=key, edgeID=temp_edge)
                                        traci.vehicle.setStop(vehID=key, edgeID=temp_edge,
                                                              pos=list(passenger_hash_map[urgent_request][1].values())[
                                                                  0],
                                                              laneIndex=0, startPos=
                                                              list(passenger_hash_map[urgent_request][1].values())[0],
                                                              duration=3)
                                        car_stop[key] = list(passenger_hash_map[urgent_request][1].values())[0]

                                        car_hash_map[key] = [net.getEdge(temp_edge).getID(), urgent_request, '2', '3',
                                                             '4']
                                car_relocate[key] = 1
                                Car_hashmap[key] = 1
                    InboundStep = inboundStep(step, key)
                    print('inbound patron arrival information', InboundStep)
                    if Car_map[key] == 1 and car_flag[key] == 1:
                        inbound_depot = traci.edge.getLastStepVehicleIDs(edgeID=car_hash_map[key][0])
                        if key in inbound_depot:
                            if traci.vehicle.getLanePosition(vehID=key) < 21 and traci.vehicle.getLanePosition(
                                    vehID=key) > 19 and Car_map[key] == 1:
                                print('drop off the last patron')
                                unmatched_request = [key for key, value in passenger_hashmap.items() if value == 1]
                                car_flag[key] = -1
                                vehicle_drop[key].append(step)
                                if len(unmatched_request) == 0:
                                    car_hash_map[key] = [car_hash_map[key][0]]
                                    # Car_hashmap[key] = -1
                                    Car_map[key] = -1
                                    traci.vehicle.setStop(vehID=key, edgeID=car_hash_map[key][0],
                                                          pos=20,
                                                          laneIndex=0, startPos=20, duration=3)
                                else:
                                    urgent_request = reposition_algorithm(key, step, unmatched_request)
                                    if urgent_request == []:
                                        car_hash_map[key] = [car_hash_map[key][0]]
                                        # Car_hashmap[key] = -1
                                        Car_map[key] = -1
                                        traci.vehicle.setStop(vehID=key, edgeID=car_hash_map[key][0],
                                                              pos=20,
                                                              laneIndex=0, startPos=20, duration=3)
                                    else:
                                        passenger_hashmap[urgent_request] = 2
                                        temp_edge = list(passenger_hash_map[urgent_request][1].keys())[0]
                                        traci.vehicle.changeTarget(vehID=key, edgeID=temp_edge)
                                        traci.vehicle.setStop(vehID=key, edgeID=temp_edge,
                                                              pos=list(passenger_hash_map[urgent_request][1].values())[
                                                                  0],
                                                              laneIndex=0,
                                                              startPos=
                                                              list(passenger_hash_map[urgent_request][1].values())[
                                                                  0], duration=3)

                                        car_hash_map[key] = [net.getEdge(temp_edge).getID(), urgent_request, '2', '3',
                                                             '4']
                                        car_relocate[key] = 1
                                        Car_hashmap[key] = 1
                                        car_stop[key] = list(passenger_hash_map[urgent_request][1].values())[0]

        print('vehicle arrival information', arrival_outcome)
        DepotStep = getDepotStep(step)
        print('patron arrival information', DepotStep)
        print('pick-uo list', pick_up_list)
        step += 1
    traci.close()
    sys.exit()

if __name__ == '__main__':
    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
        traci.start([sumoBinary, '-c', './sumo_config/demo_hete.sumocfg',
                     '--start',
                     '--quit-on-end',
                     '--gui-settings-file', './sumo_config/viewsettings.view.xml'])  # ,'--quit-on-end'
    run()