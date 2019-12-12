import math
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

import seaborn as sns
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def drawAPlot(dataFilePath, filename):
	dataFileName = dataFilePath + filename
	print(dataFileName)

	data = open(dataFileName)
	lines = data.readlines()
	data.close()

	xs = []
	ys = []
	values = []
	values_log = []
	for line in lines:
		line = line.strip().split()
		xs.append(round(float(line[0])/1000.,1))
		ys.append(round(float(line[1])/1000.,1))
		#xs.append(float(line[0])/1000.)
		#ys.append(float(line[1])/1000.)

		value = float(line[2])

#		if(value>0.0015):
#			value=1;
#		else:
#			value=0;

		values.append(value)

	df = pd.DataFrame({'y':ys,'x': xs,'CostFunctionValues':values})
	pt = df.pivot_table(index='y', columns='x', values='CostFunctionValues', aggfunc=np.sum)

	f, ax1 = plt.subplots(figsize = (5,4),nrows=1)
	cmap = sns.diverging_palette(200,20,sep=20,as_cmap=True)
	#sns_plot = sns.heatmap(pt, linewidths = 0.00, ax = ax1, cmap="YlGnBu_r", xticklabels=12, yticklabels=12)
	sns_plot = sns.heatmap(pt, linewidths = 0.00, ax = ax1, cmap="viridis", xticklabels=12, yticklabels=12)
	sns_plot.tick_params(labelsize=10)

	ax1.invert_yaxis()
	#ax1.set_title('Obstacle (building) Distribution', fontsize=12)
	ax1.set_xlabel('X / km',fontsize=10)
	ax1.set_ylabel('Y / km',fontsize=10)

	if dataFilePath!="alice_step1_part3_obstacles/build/":
		if filename == "data_step1_part2_tool_obstacles.txt":
			plt.savefig('figure-tool-DistributionOfObstacles.jpeg',dpi=300, bbox_inches='tight')

		if filename == "data_step1_part2_tool_obstacles_binary.txt":
			plt.savefig('figure-tool-DistributionOfObstacles-binary.jpeg',dpi=300, bbox_inches='tight')

		if filename == "data_step1_part2_tool_obstacles_binary_RegionOfInterst.txt":
			plt.savefig('figure-tool-DistributionOfObstacles-binary-RegionOfInterst.jpeg',dpi=300, bbox_inches='tight')

	if dataFilePath=="alice_step1_part3_obstacles/build/":
		name_hr = "_HighResolution"
		if filename == "data_step1_part2_tool_obstacles.txt":
			plt.savefig('figure-tool-DistributionOfObstacles'+name_hr+'.jpeg',dpi=300, bbox_inches='tight')

		if filename == "data_step1_part2_tool_obstacles_binary.txt":
			plt.savefig('figure-tool-DistributionOfObstacles-binary'+name_hr+'.jpeg',dpi=300, bbox_inches='tight')

		if filename == "data_step1_part2_tool_obstacles_binary_RegionOfInterst.txt":
			plt.savefig('figure-tool-DistributionOfObstacles-binary-RegionOfInterst'+name_hr+'.jpeg',dpi=300, bbox_inches='tight')


	return

if __name__ == '__main__':
	print("hello")

#	dataFilePath = "alice_step1_part2_populationDensityGenerator_version3_aStarPathPlanning/build/"
#	filename = "data_step1_part2_tool_obstacles.txt"
#	drawAPlot(dataFilePath, filename)
#
#	dataFilePath = "alice_step1_part2_populationDensityGenerator_version3_aStarPathPlanning/build/"
#	filename = "data_step1_part2_tool_obstacles_binary.txt"
#	drawAPlot(dataFilePath, filename)
#
#	dataFilePath = "alice_step1_part2_populationDensityGenerator_version3_aStarPathPlanning/build/"
#	filename = "data_step1_part2_tool_obstacles_binary_RegionOfInterst.txt"
#	drawAPlot(dataFilePath, filename)

	dataFilePath = "alice_step1_part3_obstacles/build/"
	filename = "data_step1_part2_tool_obstacles.txt"
	drawAPlot(dataFilePath, filename)

	dataFilePath = "alice_step1_part3_obstacles/build/"
	filename = "data_step1_part2_tool_obstacles_binary.txt"
	drawAPlot(dataFilePath, filename)

	dataFilePath = "alice_step1_part3_obstacles/build/"
	filename = "data_step1_part2_tool_obstacles_binary_RegionOfInterst.txt"
	drawAPlot(dataFilePath, filename)

	plt.show()


