import numpy as np
import matplotlib.pyplot as plt
import csv
import string
import seaborn as sns
import math

def Plot(path, fileName, name):
	print("Data File Path : {}".format(path))
	print("File Name : {}".format(fileName))
	print("Name ID   : {}".format(name))

	# read
	f = open(path+fileName)
	lines = f.readlines()

	binCentors = []
	contents   = []

	for line in lines:
		line = line.strip().split()

		binCentors	.append(float(line[1]))
		contents	.append(float(line[2]))
	#
	# plot
	#
	fig = plt.figure(figsize=(5,4))
	plt.plot(binCentors, contents,ls='-', linewidth=1.0,color='k', marker='s', mec='k', mfc='w')
	#plt.legend(frameon=True)
	plt.xlabel(name,fontdict={'family' : 'Times New Roman', 'size': 12})
	plt.ylabel("Counts",fontdict={'family' : 'Times New Roman', 'size': 12})
	#plt.xlim(0,30)
	#plt.ylim(0,60)
	#plt.title('Distribution of Errors')
	plt.savefig('figure-'+name+'.jpeg',dpi=300)

	return

if __name__ == '__main__':
	print("hello")

	path = "alice_step4_trainningSamples/build/"
	fileName = "data_hist_robotsSize.txt"
	Plot(path,fileName, "RobotSize")

	fileName = "data_hist_statusSize.txt"
	Plot(path,fileName, "StatusSize")

	plt.show()
