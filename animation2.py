# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import csv
import numpy
filename = "F:/GitHub/jakkalavanya1 GITHUB/MATLAB_MDV-CAVsim/index.csv"
ifile = open(filename, "rU");
reader = (csv.reader(ifile, delimiter=' ', quotechar='|'));
rownum = 0;	
#a=[][]
'''
a=numpy.empty((160,8))
for row in reader:
    for column in len(reader[row]):
        # u can use i instead of row
        a[row][column] = reader[row][column]
        rownum += 1;
ifile.close()
print("the value of a is",a)
'''
# numpy method of saving into matrix

reader_list=list(csv.reader(ifile, delimiter=',')) #it makes list of lists into normal list
result=numpy.array(reader_list).astype("float")
print("the result is",result)

index=result[7][:] #in python index starts from 0
print("the index of cav and non CAV are",index)
time=len(result) #equals to the numel in MATLAB (time)
for i in time:
    if index[k]==3 && road[]==1:
        plot(result[k])