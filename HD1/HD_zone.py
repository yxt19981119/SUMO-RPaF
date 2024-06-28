#!/usr/bin/env python
# -*- coding:utf-8 -*-
# author:严啸天
# datetime:2024/1/12 19:33
# software: PyCharm
# Strategy: RP-HD & Batch-matching & RP_size = 4
# Required packages
import heapq
import random
import traci
import sumolib
from sumolib import checkBinary
import optparse
# from gurobipy import *
import numpy as np
import pandas as pd
import time
import sys
# Required packages
# Input: the number of patron
passenger_number = 200
# Input: the number of vehicle
taxi_number=10
# Define vehicle related information variables
df_car = pd.read_excel('./sumo_config/vehicle_info_homo.xlsx', sheet_name='information table')  # 替换为你的文件名和工作表名称
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
for i in range(taxi_number):
    car_hashmap[str(i+1)] = 0
    Car_hashmap[str(i+1)] = 1
    # Car_map[str(i+1)] = -1
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


zone_matching = dict()
zone1 = pd.read_excel('./sumo_config/uniform distribution.xls', sheet_name='1')
zone2 = pd.read_excel('./sumo_config/uniform distribution.xls', sheet_name='2')
zone3 = pd.read_excel('./sumo_config/uniform distribution.xls', sheet_name='3')
zone4 = pd.read_excel('./sumo_config/uniform distribution.xls', sheet_name='4')
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

RPV2Zone = {'1':'1','2':'1','3':'2','4':'2','5':'2','6':'3','7':'3','8':'4','9':'4','10':'4'}
Zone2RPV = {'1':['1','2'],'2':['3','4','5'],'3':['6','7'],'4':['8','9','10']}

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
        ef= JunctionToEdge_hashmap[ef]
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

def distribute(Delta,unmatched_request,step,zone_request):
    p,zone_request = checkpassenger(zone_request)  #
    print(p)
    # for key in p.keys():
    c = checkcar()  #
    exit_flag = False
    for each in c.values():
        if each >= 5:
            exit_flag = True
        else:
            exit_flag = False
            break
    # unmatched_request = [key for key, value in p.items() if value == 1]
    if len(unmatched_request) == 0 or exit_flag == True:  # no patron or no vehicle available
        print('nothing occur')
    elif len(unmatched_request) == 1:  # under-saturation
        print('a patron')
        zone_id = passenger_hash_map[unmatched_request[0]][2]
        dis = np.zeros([1, len(Zone2RPV[zone_id])])  #
        temp_hashmap = dict()
        for index,each in enumerate(Zone2RPV[zone_id]):
            temp_hashmap[index] = each
            if (c[each]) < 5:  # get patrons' information
                person_edge = list(passenger_hash_map[unmatched_request[0]][1].keys())[0]
                person_pos = list(passenger_hash_map[unmatched_request[0]][1].values())[0]
                car_edge = traci.vehicle.getRoadID(each)  #
                car_pos = traci.vehicle.getLanePosition(each)  # sumo problem
                if car_edge[0] == ':':
                    car_edge = JunctionToEdge_hashmap[car_edge]
                    car_pos = 0  #
                temp_distance = net.getShortestPath(net.getEdge(car_edge), net.getEdge(person_edge))[1] # attain shortest path
                dis[0][index] = temp_distance - car_pos - (street_space - person_pos)  #
            elif (c[each]) >= 5:
                dis[0][index] = 1e10
        if min(dis[0]) <= Delta:  # buffer distance
            temp = int(np.argmin(dis[0]))  # attain nearest veh
            car_hash_map[temp_hashmap[temp]].append(unmatched_request[0])
            p[unmatched_request[0]] = 2  # 2 indicate finish match
            print('request list',unmatched_request[0])
    elif len(unmatched_request) > 1:
        print('multiple requests to match')
        for c_key in c.keys():
            if c[c_key] >= 5:
                continue
            else:
                passenger_candidata = list()
                zone_num = RPV2Zone[c_key]
                for each in unmatched_request:
                    if p[each] == 1 and passenger_hash_map[each][2] == zone_num:
                        person_edge = list(passenger_hash_map[each][1].keys())[0]
                        person_pos = list(passenger_hash_map[each][1].values())[0]
                        car_edge = traci.vehicle.getRoadID(c_key)  #
                        car_pos = traci.vehicle.getLanePosition(c_key)  #
                        if car_edge[0] == ':':
                            car_edge = JunctionToEdge_hashmap[car_edge]
                            car_pos = 0  #
                        temp= net.getShortestPath(net.getEdge(car_edge), net.getEdge(person_edge))[
                            1]  # attain shortest path
                        temp_distance = temp - car_pos - (
                                    street_space - person_pos)  #
                        print(each+'distance', temp_distance)
                        if temp_distance <= Delta:
                            passenger_candidata.append({each:urgent(request=each,vehicle=c_key,step=step)})
                        else:
                            continue
                print('pax_urgent_pair',passenger_candidata)
                empty_seat = 5-c[c_key]
                distance_candidate = list()
                for index, hash_map in enumerate(passenger_candidata):
                    distance_candidate.append(list(hash_map.values())[0])
                tmp = zip(range(len(distance_candidate)),distance_candidate)
                small = heapq.nlargest(empty_seat,tmp,key=lambda x:x[1])
                for each in small:
                    car_hash_map[c_key].append(list(passenger_candidata[each[0]].keys())[0])
                    p[list(passenger_candidata[each[0]].keys())[0]] = 2
    # Remark: Input, output
    return car_hash_map

# Routing_algorithm——outbound
def addRoute(edgelist, key):
    start_time = time.time()
    passengerEdgeList = list()
    for each in edgelist[1:5]:
        passengerEdgeList.append(list(passenger_hash_map[each][1].keys())[0])
    passengerPosList = list()
    for each in edgelist[1:5]:
        passengerPosList.append(list(passenger_hash_map[each][1].values())[0])
    car_edge = traci.vehicle.getRoadID(vehID=key)
    if car_edge[0] == ':':
        car_edge = JunctionToEdge_hashmap[car_edge]
        car_pos = 0
    else:
        car_pos = traci.vehicle.getLanePosition(vehID=key)
    routeid = [car_edge]+passengerEdgeList+[terminal]
    posid = [car_pos]+passengerPosList + [4886]
    EdgeToPos = dict()
    for i in range(len(routeid)):
        EdgeToPos[routeid[i]] = posid[i]
    routehash = dict()
    for i in range(len(routeid)):
        routehash[i]=routeid[i]
    dismatrix = [[0]*len(routeid) for _ in range(len(routeid))]
    for i in range(len(routeid)):
        for j in range(len(routeid)):
            if i != j:
                dismatrix[i][j]=getDistance(routeid[i],routeid[j],EdgeToPos)
            if i == j:
                dismatrix[i][j] = 0
    ret = calculate(dismatrix, len(routeid))
    print(ret)
    routeNumberid = ret[1]
    routeEdgeid = list()
    #
    for each in routeNumberid:
        routeEdgeid.append(routehash[each])
    routePosid = list()
    for each in routeNumberid:
        routePosid.append(posid[each])
    routeEdge = list()
    for i in range(len(routeEdgeid)):
        if i < len(routeEdgeid)-2:
            if i == 0 and routeEdgeid[i] == routeEdgeid[i+1]:
                x = net.getEdge(routeEdgeid[i])
                if routeEdgeid[i][0] == '-':
                    y = net.getEdge(routeEdgeid[i+1].strip('-'))
                else:
                    y = net.getEdge('-'+routeEdgeid[i+1])
                routeEdge += net.getShortestPath(x,y)[0]
                x_EdgeIDList = [SingleEdge.getID() for SingleEdge in routeEdge]
            else:
                x = net.getEdge(routeEdgeid[i])
                y = net.getEdge(routeEdgeid[i+1])
                routeEdge+=net.getShortestPath(x,y)[0]
                x_EdgeIDList = [SingleEdge.getID() for SingleEdge in routeEdge]
                routeEdge.pop()
        elif i==len(routeEdgeid)-2:
            if i == 0 and routeEdgeid[i] == routeEdgeid[i+1]:
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
    end_time = time.time()
    print('TSP time',end_time-start_time)
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
        EdgetoPos[routeid[i]] = posid[i]
    routehash = dict()
    for i in range(len(routeid)):
        routehash[i] = routeid[i]
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
                # print(x_EdgeIDList)
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
                    continue
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
            if traci.person.getRoadID(personID=each)==terminal and traci.person.getLanePosition(personID=each)>4884 and traci.person.getLanePosition(personID=each)<4887 and depotStep_hashmap[each]==0:
                depotStep_hashmap[each]=step
            else:
                continue
    return depotStep_hashmap
# Record inbound patron alighting module
def inboundStep(step,key):
    global depotStep_hashmap
    person_list=list(traci.vehicle.getPersonIDList(vehID=key))

    for each in person_list:
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
    arrival_outcome=list()
    # time_window = 100
    pick_up_list = {'1': [], '2': [], '3': [],'4': [], '5': [], '6': [],'7': [], '8': [], '9': [],'10': []}
    car_stop = {'1':20,'2':20,'3':20,'4':20,'5':20,'6':20,'7':20,'8':20,'9':20,'10':20}
    Delta = 2.5e3
    zone_nums = 4
    zone_request = dict()
    zone_vehicle_id = dict()
    zone_inbound_demand = dict()
    zone_headway = dict()
    zone_vehicle_nums = {'1': 2, '2': 3, '3': 2, '4': 3}
    for i in range(zone_nums):
        zone_request[str(i + 1)] = []
        zone_vehicle_id[str(i + 1)] = []
        zone_inbound_demand[str(i + 1)] = []
        zone_headway[str(i + 1)] = 560
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        global passenger_hashmap
        global car_hash_map
        global Car_hashmap
        global car_flag
        # step=0
        if step == 0:
            for key in car_hash_map.keys():
                edge_temp = net.getEdge(car_hash_map[key][0]).getID()
                # print(edge_temp)
                traci.route.add(routeID=key, edges=[edge_temp])
                traci.vehicle.add(vehID=key, routeID=key, typeID="DEFAULT_VEHTYPE", depart=step, line='taxi',
                                  personCapacity=5, personNumber=0)

        unmatched_request = [key for key, value in passenger_hashmap.items() if value == 1]
        print('unmatched request list', unmatched_request)
        Car_hash_map = distribute(Delta,unmatched_request,step,zone_request)

        print('matching information', Car_hash_map)
        print('patron status', passenger_hashmap)
        print('vehicle status', car_flag)
        print('car_stop',car_stop)
        PersonStep=getPersonStep(step)
        print('patron boarding',PersonStep)


        # dispatch_algorithm
        for key in Car_hash_map.keys():
            if car_relocate[key] == 1:
                if traci.vehicle.getRoadID(vehID=key) == car_hash_map[key][0]:
                    if car_hash_map[key][1] == '1':
                        car_hash_map[key] = car_hash_map[key][0:1]
                    elif car_hash_map[key][1] != '1':
                        car_hash_map[key] = car_hash_map[key][0:2]
                    car_relocate[key] = -1
                    # traci.vehicle.setStop(vehID=key, edgeID=net.getEdge(Car_hash_map[key][0]).getID(), pos=40,
                    #                       laneIndex=0, startPos=40, duration=2)
                    vehicle_drop[key].append(step)
            else:
                if car_flag[key] == -1:  # outbound
                    if len(Car_hash_map[key]) < 5:
                        traci.vehicle.setStop(vehID=key, edgeID=net.getEdge(Car_hash_map[key][0]).getID(), pos=car_stop[key], laneIndex=0, startPos=car_stop[key], duration=2)
                        print('taxi%s failure to meet departure requirements, continue to wait for allocation'%key)
                    elif len(Car_hash_map[key]) == 5:
                        print('taxi%s meet departure requirements and prepare to depart'%key)
                        route,routeEdgeid,routePosid=addRoute(Car_hash_map[key],key)
                        print(route)
                        print(routeEdgeid)
                        print(routePosid)
                        # traci.vehicle.setStop(vehID=key, edgeID=net.getEdge(Car_hash_map[key][0]).getID(), pos=76.8, laneIndex=0, startPos=76.8, duration=0)
                        pick_up_list[key].append(Car_hash_map[key])
                        traci.vehicle.setRoute(vehID=key,edgeList=route)
                        for i in range(1,len(routeEdgeid)):
                            traci.vehicle.setStop(vehID=key, edgeID=routeEdgeid[i], pos=routePosid[i], laneIndex=0,duration=3, startPos=routePosid[i])
                        Car_hash_map[key].append('1')
                        vehicle_service_nums[key] = vehicle_service_nums[key]+4
                        vehicle_pick[key].append(step)
                    elif len(Car_hash_map[key]) > 5:
                        print('taxi%s in service'%key)
                    hub_edge = traci.edge.getLastStepVehicleIDs(terminal)
                    if key in hub_edge:
                        if traci.vehicle.getLanePosition(vehID=key) < 4885 and traci.vehicle.getLanePosition(vehID=key) > 4882:
                            temp = list(traci.vehicle.getPersonIDList(vehID=key))
                        elif traci.vehicle.getLanePosition(vehID=key) < 4888 and traci.vehicle.getLanePosition(vehID=key) > 4885:
                            temp = list(traci.vehicle.getPersonIDList(vehID=key))
                            temp = temp + [step]
                            arrival_outcome.append(temp)

                            # temp_edge = all_of_edge[random.randint(0, len(all_of_edge) - 1)]
                            traci.vehicle.changeTarget(vehID=key, edgeID=terminal_reverse)
                            traci.vehicle.setStop(vehID=key, edgeID=terminal_reverse, pos=20+20*int(RPV2Zone[key]), laneIndex=0, startPos=20+20*int(RPV2Zone[key]), duration=3)
                            # traci.vehicle.setStop(vehID=key, edgeID=temp_edge, pos=10, laneIndex=0, startPos=10, duration=2)

                            if car_flag[key] == -1:
                                car_hash_map[key] = [list(passenger_hash_map[car_hash_map[key][4]][1].keys())[0],'2','3','4','5']
                                car_hashmap[key] = 5
                                car_flag[key] = 1
                            if Car_hashmap[key] != -1:
                                Car_hashmap[key] = -1
                                # Car_hashmap[key] = 0
                elif car_flag[key] == 1:  # inbound
                    hub_edge_inverse = traci.edge.getLastStepVehicleIDs(terminal_reverse)
                    if key in hub_edge_inverse:
                        # if traci.vehicle.getLanePosition(vehID=key) < 481 and traci.vehicle.getLanePosition(vehID=key) > 479 and Car_hashmap[key] == -1:
                        if traci.vehicle.getLanePosition(vehID=key) > 501 and Car_hashmap[key] == -1:
                            inbound_demand = traci.vehicle.getPersonIDList(vehID=key)
                            if len(inbound_demand) != 0:
                                for each in inbound_demand:
                                    passenger_hashmap[each] = 2
                                route,routeEdgeid,routePosid=addRoute_inbound(inbound_demand)
                                print(route)
                                print('inbound_edge',routeEdgeid)
                                print(routePosid)
                                traci.vehicle.setRoute(vehID=key, edgeList=route)
                                for i in range(1,len(routeEdgeid)):
                                    traci.vehicle.setStop(vehID=key, edgeID=routeEdgeid[i], pos=routePosid[i], laneIndex=0,duration=3, startPos=routePosid[i])
                                car_hash_map[key][0] = routeEdgeid[-1]
                                Car_hashmap[key] = 1
                                vehicle_service_nums[key] = vehicle_service_nums[key]+len(inbound_demand)
                            else:
                                # global car_flag
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
                                                              pos=list(passenger_hash_map[urgent_request][1].values())[0],
                                                              laneIndex=0, startPos=list(passenger_hash_map[urgent_request][1].values())[0], duration=3)
                                        car_stop[key] = list(passenger_hash_map[urgent_request][1].values())[0]

                                        car_hash_map[key]=[net.getEdge(temp_edge).getID(),urgent_request,'2','3','4']
                                car_relocate[key] = 1
                                Car_hashmap[key] = 1

                    if Car_hashmap[key] == 1 and car_flag[key]==1:
                        inbound_depot = traci.edge.getLastStepVehicleIDs(edgeID=car_hash_map[key][0])

                        if key in inbound_depot:
                            if traci.vehicle.getLanePosition(vehID=key) < 21 and traci.vehicle.getLanePosition(vehID=key) > 19 and Car_hashmap[key] == 1:
                                print('finish inbound service')
                                unmatched_request = [key for key, value in passenger_hashmap.items() if value == 1]
                                car_flag[key] = -1
                                vehicle_drop[key].append(step)
                                if len(unmatched_request) == 0:
                                    car_hash_map[key] = [car_hash_map[key][0]]
                                    Car_hashmap[key] = -1
                                else:
                                    urgent_request = reposition_algorithm(key, step, unmatched_request)
                                    if urgent_request == []:
                                        car_hash_map[key] = [car_hash_map[key][0]]
                                        Car_hashmap[key] = -1
                                    else:
                                        passenger_hashmap[urgent_request] = 2
                                        temp_edge = list(passenger_hash_map[urgent_request][1].keys())[0]
                                        traci.vehicle.changeTarget(vehID=key, edgeID=temp_edge)
                                        traci.vehicle.setStop(vehID=key, edgeID=temp_edge,
                                                              pos=list(passenger_hash_map[urgent_request][1].values())[0],
                                                              laneIndex=0,
                                                              startPos=list(passenger_hash_map[urgent_request][1].values())[
                                                                  0], duration=3)

                                        car_hash_map[key] = [net.getEdge(temp_edge).getID(), urgent_request, '2', '3', '4']
                                        car_relocate[key] = 1
                                        Car_hashmap[key] = 1
                                        car_stop[key] = list(passenger_hash_map[urgent_request][1].values())[0]
                    InboundStep=inboundStep(step, key)
                    print('record inbound patron alight', InboundStep)

        DepotStep = getDepotStep(step)
        print('all patron arrival information',DepotStep)

        step+=1
    traci.close()
    sys.exit()

if __name__=='__main__':
    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
        traci.start([sumoBinary,'-c','./sumo_config/demo_homo.sumocfg'])
    run()
