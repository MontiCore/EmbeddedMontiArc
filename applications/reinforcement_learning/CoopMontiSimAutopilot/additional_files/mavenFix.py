import os

def change(fileName):
	homePath = os.getenv('HOME')
	old = []
	version = "3.3.3"
	f = open("{home}/.m2/repository/montisim/{a}/{b}/{a}-{b}.pom".format(a = fileName,b = version,home = homePath),'r')
	for line in f:
		old.append(line)
	f.close()
	f = open("{home}/.m2/repository/montisim/{a}/{b}/{a}-{b}.pom".format(a = fileName,b = version,home = homePath),'w')
	for i in old:
		if i == '    <version>${revision}</version>\n':
			f.write('    <version>{}</version>\n'.format(version))
		else:
			f.write("{}".format(i))

	f.close()

def main():
	change("vehicle")
	change("eesimulator")
	change("sim-commons")
	change("environment")
	change("eecomponents")
	change("simulator")

if __name__  == "__main__":
	main()
