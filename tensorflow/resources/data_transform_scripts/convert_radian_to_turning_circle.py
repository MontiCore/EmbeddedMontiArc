import csv
import math

#Read csv file
steering_reader = csv.reader(open("../test_set/steering.csv", "rb"),delimiter=',')
steering = []
wheelbase=2.730 #in mm of lincoln mkz

for element in steering_reader:
	steering.append(element)

#convert angle in degree to turning circle
#https://de.wikipedia.org/wiki/Wendekreis_(Fahrzeug)
steering[0][1]="turning_circle"
for row in steering[1:]:
	steering_angle=float(row[1])
	if(steering_angle==0):
                #magic number
		turning_circle=0
	else:
		turning_circle= 1/((2*wheelbase)/ math.sin(steering_angle/4))
	row[1]=turning_circle

#Write csv file
wtr = csv.writer(open ('../test_set/steering_turning_circle.csv', 'w'), delimiter=',', lineterminator='\n')
for x in steering : wtr.writerow (x)
