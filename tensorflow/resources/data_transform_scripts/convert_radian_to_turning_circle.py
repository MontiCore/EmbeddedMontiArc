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
		turning_circle= (2*wheelbase)/ math.sin(steering_angle/4)
	row[1]=turning_circle

max_element = max(abs(element[1]) for element in steering[1:])


#normalize
for row in steering[1:]:
    if row[1] == 0:
        row[1] = 1.0
    else:
        row[1] = row[1]/max_element

#max_element = max((element[1]) for element in steering[1:])
#min_element = min((element[1]) for element in steering[1:])

#print max_element
#print min_element

#Write csv file
wtr = csv.writer(open ('../test_set/steering_turning_circle.csv', 'w'), delimiter=',', lineterminator='\n')
for x in steering : wtr.writerow (x)
