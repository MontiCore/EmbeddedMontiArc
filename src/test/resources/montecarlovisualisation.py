import json
import numpy as np
import matplotlib.pyplot as plt
import argparse
import math

parser = argparse.ArgumentParser(description='Get json files')
parser.add_argument('autopilot', metavar='autopilot',
                    help='montecarloresult json location of autopilot')
parser.add_argument('pacman', metavar='pacman',
                    help='montecarloresult json location of pacman')
parser.add_argument('supermario', metavar='supermario',
                    help='montecarloresult json location of supermario')
parser.add_argument('daimler', metavar='daimler',
                    help='montecarloresult json location of daimlermodel')

args = parser.parse_args()

with open(args.autopilot) as data_file:
    autopilot_data = json.load(data_file)
	
with open(args.pacman) as data_file:
    pacman_data = json.load(data_file)
	
with open(args.supermario) as data_file:
    supermario_data = json.load(data_file)
	
with open(args.daimler) as data_file:
    daimler_data = json.load(data_file)

	
autopilot_montecarlodata = []	
pacman_montecarlodata = []
supermario_montecarlodata = []		
i = 1
while i < 1001:
    autopilot_montecarlodata.append(autopilot_data[0]["MCResult(" + str(i) + ")"])
    pacman_montecarlodata.append(pacman_data[0]["MCResult(" + str(i) + ")"])
    supermario_montecarlodata.append(supermario_data[0]["MCResult(" + str(i) + ")"])
    i = i + 1
	
daimler_montecarlodata = []	
i = 1
while i < 101:
    daimler_montecarlodata.append(daimler_data[0]["MCResult(" + str(i) + ")"])
    i = i + 1
	
t = np.arange(1,1001,1)

fig, ax = plt.subplots()
ax.plot(t, autopilot_montecarlodata)

ax.set(xlabel='Iterations', ylabel='Score',
       title='Montecarlo Clustering of Autopilotmodel with 3 Clusters')
ax.grid()
plt.show()

fig, ax = plt.subplots()
ax.plot(t, pacman_montecarlodata)

ax.set(xlabel='Iterations', ylabel='Score',
       title='Montecarlo Clustering of Pacmanmodel with 3 Clusters')
ax.grid()
plt.show()

fig, ax = plt.subplots()
ax.plot(t, supermario_montecarlodata)

ax.set(xlabel='Iterations', ylabel='Score',
       title='Montecarlo Clustering of Supermariomodel with 3 Clusters')
ax.grid()
plt.show()

t = np.arange(0,100,1)
fig, ax = plt.subplots()
ax.plot(t, daimler_montecarlodata)

ax.set(xlabel='Iterations', ylabel='Score',
       title='Montecarlo Clustering of Daimlermodel')
ax.grid()
plt.show()