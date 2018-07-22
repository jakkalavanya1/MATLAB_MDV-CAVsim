# -*- coding: utf-8 -*-
"""
Created on Sat Jul 21 19:30:22 2018

@author: LAVANYA
"""

import pandas as pd
from pandas import ExcelWriter
from pandas import ExcelFile
 
df = pd.read_excel('F:\GitHub\jakkalavanya1 GITHUB\MATLAB_MDV-CAVsim\xydata.xlsx', sheetname='Sheet1')
 

x_data = pd.ExcelFile(r"F:\GitHub\jakkalavanya1 GITHUB\MATLAB_MDV-CAVsim\xydata.xlsx")