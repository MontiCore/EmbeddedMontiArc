# -*- coding: utf-8 -*-
"""
Created on Thu Nov 11 14:13:57 2021

@author: j_buehring
"""
def correction(filepath,filename):
    # Read in the file
    with open(filepath+filename, 'r') as file :       
        filedata = file.read()
    
    # Replace the target string
    
    #name initial string and what it should be replaced with
    filedata = filedata.replace('C***', '$***') 
    
    # Write the file out again
    #name file which needs to be changed
    with open(filepath+filename, 'w') as file:
        file.write(filedata)


for i in range(1,2):

    filepath = 'C:/Users/Anis/Desktop/toolchain/files/Lattice_Structures/'
    filename = '{}.k'.format(i)                             #name file which needs to be changed
    correction(filepath, filename)
