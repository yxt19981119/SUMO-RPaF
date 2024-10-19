#!/usr/bin/env python
# -*- coding:utf-8 -*-
# author:严啸天
# datetime:2024/1/14 10:09
# software: PyCharm
import pandas as pd
df_car = pd.read_excel('./sumo_config/vehicle_info_homo.xlsx', sheet_name='information table')  # 替换为你的文件名和工作表名称
taxi_number = len(df_car['id'])
RPV2Zone = {'1':'1','2':'1','3':'2','4':'2','5':'2','6':'3','7':'3','8':'4','9':'4','10':'4'}
Zone2RPV = {'1':['1','2'],'2':['3','4','5'],'3':['6','7'],'4':['8','9','10']}
df_zone = df_car['zone']
RPV2Zone1 = dict()
Zone2RPV1 = {'1':[],'2':[],'3':[],'4':[]}

for i in range(taxi_number):
    RPV2Zone1[str(i+1)] = str(df_zone[i])
    Zone2RPV1[str(df_zone[i])].append(str(i+1))

print(RPV2Zone==RPV2Zone1)
print(Zone2RPV1 == Zone2RPV)