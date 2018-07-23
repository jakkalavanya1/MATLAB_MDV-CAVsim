# -*- coding: utf-8 -*-
"""
Created on Sat Jul 21 19:30:22 2018

@author: LAVANYA
"""

import csv
with open('F:\GitHub\jakkalavanya1 GITHUB\MATLAB_MDV-CAVsim\index.csv', 'rb') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    spamwriter.writerow(['Spam'] * 5 + ['Baked Beans'])
    #for row in spamreader:
     #   print( ', '.join(row))