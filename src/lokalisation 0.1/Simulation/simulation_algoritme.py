# -*- coding: utf-8 -*-
"""
Created on Tue Nov  4 11:11:13 2025

@author: renyb

@name: Antena simulation

@description: Simulates the antena with the test data, it returns the RSSI-value and the tag ID

"""

import pandas as pd
file_path = r"rfid_data_051125_133757_demo.xlsx"

def get_values(data_type, i):
    df1 = pd.read_excel(file_path)
    if data_type == "antenna":
        return get_values_antena(df1, i)
    if data_type == "mecabot":
        return get_values_mecabot(df1, i)
    

def get_values_antena(df1, i):
    # i ++ with each reaiding command 
    row = df1.iloc[i].to_dict()
    tag_id = row.get('Tag ID')
    tag_rssi = row.get('RSSI')
    
    antenna_data = {
        "Tag ID": tag_id,
        "RSSI": tag_rssi
        }
    
    return antenna_data


def get_values_mecabot(df1, i):
    # i ++ with each reaiding command 
    
    row = df1.iloc[i].to_dict()
    
    x = row.get('Antenna X [m]')
    y = row.get('Antenna Y [m]')
    z = row.get('Antenna Z [m]')
    rot = row.get('Antenna Rot Z [deg]')
    
    mecabot_data = {
        "x": x,
        "y": y,
        "z": z,
        "rot": rot
        }
    
    return mecabot_data

def amount_data():
    df1 = pd.read_excel(file_path)
    num_rows = len(df1)
    return num_rows
    


