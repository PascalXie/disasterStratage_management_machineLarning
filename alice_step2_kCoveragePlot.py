import numpy as np
import matplotlib.pyplot as plt
import csv
import string
import seaborn as sns
import math

def Plot(path, fileName, GenerationID):
	print("Generation ID  : {}".format(GenerationID))
	print("Data File Path : {}".format(path))
	print("File Name : {}".format(fileName))

	# read
	f = open(path+fileName)
	lines = f.readlines()

	binCentors = []
	contents   = []

	for line in lines:
		line = line.strip().split()

		binCentors	.append(float(line[0]))
		contents	.append(float(line[1]))

	#
	# plot
	#
	fig = plt.figure(figsize=(5,4))
	plt.plot(binCentors, contents,ls='-', linewidth=1.0,color='k', marker='s', mec='k', mfc='w', label="Generation "+str(GenerationID))
	plt.legend(frameon=True)
	plt.xlabel('k-Coverage',fontdict={'family' : 'Times New Roman', 'size': 12})
	plt.ylabel("Counts",fontdict={'family' : 'Times New Roman', 'size': 12})
	#plt.xlim(0,30)
	#plt.ylim(0,60)
	#plt.title('Distribution of Errors')
	plt.savefig('figure-kCoverage-GenID'+str(GenerationID)+'.jpeg',dpi=300)

	return 

if __name__ == '__main__':
	print("hello")

	GenerationID = 9
	path = "alice_step2_UAVsCoverage_part5_unlimitedUAVs_tool_h2dCoverage/build/"
	filename = "data_kCoverage_generation_"+str(GenerationID)+".txt"
	Plot(path,filename, GenerationID)

	plt.show()
