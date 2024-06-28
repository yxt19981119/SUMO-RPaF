#!/usr/bin/env python
# -*- coding:utf-8 -*-
# author:Xiaotian Yan and Wenbo Fan
# datetime:2024/1/12 10:15
# software: PyCharm
import pandas as pd

df_pax = pd.read_excel('./sumo_config/pax_info_homo.xlsx',sheet_name='basic')
pax_id = df_pax['id']
Timestamp = df_pax['request time']
pax_edge = df_pax['edge']
pax_pos = df_pax['pos']
pax_zone = df_pax['zone']
passenger_number = len(pax_id)
def generate_routefile():
    # write XX.rou.xml
    with open("./sumo_config/demo_homo.rou.xml", "w") as routes:
        print("""<?xml version="1.0" encoding="UTF-8"?>
           <routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">
           <!-- Route -->
           <route edges="E0 E1 E2 E3 E4 -E11 -E10 -E8" color="yellow" id="route_0"/>
           <!-- Taxi-Type -->
           <vType id="taxi" vClass="taxi" personCapacity="5">
               <param key="has.taxi.device" value="true"/>
           </vType>
           <!-- People heading to the busstop -->""", file=routes)
        for i in range(passenger_number):
            temp_edge = pax_edge[i]
            temp_pos = pax_pos[i]
            if temp_pos == 100:
                temp_pos1 = 30
                temp_pos = 20+20*int(pax_zone[i])
                print('           <person id="%i" depart="%f" color="yellow" departPos="%i">' % (i + 1, Timestamp[i], temp_pos), file=routes)
                print('               <walk from="-E5103" to="-E5103" arrivalPos="%i"/>' % (temp_pos),file=routes)
                print('               <ride from="-E5103" to="%s" lines="taxi"/>' % (temp_edge), file=routes)
                print("           </person>", file=routes)
            else:
                print('           <person id="%i" depart="%f" color="yellow" departPos="%i">' % (i + 1, Timestamp[i],temp_pos), file=routes)
                print('               <walk from="%s" to="%s" arrivalPos="%i"/>' % (temp_edge,temp_edge,temp_pos), file=routes)
                print('               <ride from="%s" to="E5103" lines="taxi"/>' % (temp_edge), file=routes)
                print("           </person>", file=routes)
        print("</routes>", file=routes)

generate_routefile()
