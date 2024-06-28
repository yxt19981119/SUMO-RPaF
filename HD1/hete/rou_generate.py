#!/usr/bin/env python
# -*- coding:utf-8 -*-
# author:严啸天
# datetime:2024/1/12 10:15
# software: PyCharm
import pandas as pd

df_pax = pd.read_excel('./sumo_config/pax_info_hete.xlsx',sheet_name='basic')
pax_id = df_pax['id']
Timestamp = df_pax['request time']
pax_edge = df_pax['edge']
pax_pos = df_pax['pos']
passenger_number = len(pax_id)
def generate_routefile():
    # 打开文件,“w” 代表将进行写入操作
    # wb = op.load_workbook('E:\sumo_demo\demo24.111\乘客信息.xlsx')
    # sh = wb["Sheet1"]
    with open("./sumo_config/demo_hete.rou.xml", "w") as routes:
        print("""<?xml version="1.0" encoding="UTF-8"?>
           <routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">
           <!-- Route -->
           <route edges="E0 E1 E2 E3 E4 -E11 -E10 -E8" color="yellow" id="route_0"/>
           <!-- Taxi-Type -->
           <vType id="taxi" vClass="taxi" personCapacity="5">
               <param key="has.taxi.device" value="true"/>
           </vType>
           <!-- People heading to the busstop -->""", file=routes)
        # 初始乘客为0，接下来开始随机乘客需求的位置，%表示随机乘客的位置
        for i in range(passenger_number):
            # if i % 10 == 9:
            # passenger_hashmap[str(i + 1)] = 0  # 初始状态为0,表示乘客未进入路网
            # personStep_hashmap[str(i + 1)] = -1  # 初始状态为-1，表示乘客未上车
            temp_edge = pax_edge[i]
            temp_pos = pax_pos[i]
            if temp_pos == 100:
                #temp_pos1 = int(random.uniform(40, 440))#获取inbound乘客的下车位置，其他程序调用时更加方便
                temp_pos1 = 30#固定inbound乘客的下车位置
                # sh.cell(i + 2, 1, int(i+1))
                # sh.cell(i + 2, 2, temp_edge)
                # sh.cell(i + 2, 3, temp_pos)
                # sh.cell(i + 2, 4, Timestamp[i])
                # passenger_hash_map[str(i + 1)] = [Timestamp[i], {"%s" % (temp_edge): temp_pos1}]
                print('           <person id="%i" depart="%f" color="yellow" departPos="%i">' % (i + 1, Timestamp[i], temp_pos), file=routes)
                print('               <walk from="-E5103" to="-E5103" arrivalPos="%i"/>' % (temp_pos),file=routes)
                print('               <ride from="-E5103" to="%s" lines="taxi"/>' % (temp_edge), file=routes)
                print("           </person>", file=routes)
            else:
                # passenger_hashmap[str(i + 1)] = 0  # 初始状态为0,表示乘客未进入路网
                # personStep_hashmap[str(i + 1)] = -1 # 初始状态为-1，表示乘客未上车
                # temp_edge = pax_edge[i]
                # temp_pos = pax_pos[i]
                # sh.cell(i + 2, 1, int(i + 1))
                # sh.cell(i + 2, 2, temp_edge)
                # sh.cell(i + 2, 3, temp_pos)
                # sh.cell(i + 2, 4, Timestamp[i])
                # passenger_hash_map[str(i + 1)] = [Timestamp[i],{"%s" %(temp_edge):temp_pos}]
                print('           <person id="%i" depart="%f" color="yellow" departPos="%i">' % (i + 1, Timestamp[i],temp_pos), file=routes)
                print('               <walk from="%s" to="%s" arrivalPos="%i"/>' % (temp_edge,temp_edge,temp_pos), file=routes)
                print('               <ride from="%s" to="E5103" lines="taxi"/>' % (temp_edge), file=routes)
                print("           </person>", file=routes)
        print("</routes>", file=routes)
        # wb.save('E:\sumo_demo\demo24.111\乘客信息.xlsx')
generate_routefile()
# print(passenger_hash_map)
# print(passenger_hashmap)
# print(personStep_hashmap)