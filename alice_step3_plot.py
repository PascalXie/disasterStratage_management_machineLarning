import math
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

import seaborn as sns
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def drawAPlot(dataFilePath, filename, time1, time2):
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
		#xs.append(round(float(line[0]),1))
		#ys.append(round(float(line[1]),1))
		x = float(line[0])
		y = float(line[1])

		# time selecting
		if x<60.*60.*time1:
			continue
		if x>60.*60.*time2:
			break

		xs.append(x)
		ys.append(y)

		value = float(line[2])
		values.append(value)

	df = pd.DataFrame({'y':ys,'x': xs,'CostFunctionValues':values})
	pt = df.pivot_table(index='y', columns='x', values='CostFunctionValues', aggfunc=np.sum)

	f, ax1 = plt.subplots(figsize = (6,5),nrows=1)
	cmap = sns.diverging_palette(200,20,sep=20,as_cmap=True)
	#sns_plot = sns.heatmap(pt, linewidths = 0.00, ax = ax1, cmap="YlGnBu_r", xticklabels=12, yticklabels=12)
	sns_plot = sns.heatmap(pt, linewidths = 0.00, ax = ax1, cmap="viridis", xticklabels=12, yticklabels=12)
	sns_plot.tick_params(labelsize=10)

	ax1.invert_yaxis()
	#ax1.set_title('Changes of Deployment along Time between '+str(time1)+"-"+str(time2)+"(h)", fontsize=12)
	ax1.set_xlabel('Time / second',fontsize=10)
	ax1.set_ylabel('Deployment Point ID',fontsize=10)
	plt.savefig('figure-Hist2DTimeDeployment'+str(time1)+"-"+str(time2)+'.jpeg',dpi=300, bbox_inches='tight')

	# Analysis 
	uncovered_counts = 0
	for ele in values:
		if ele == 0:
			uncovered_counts += 1

	uncovered_ratio = uncovered_counts/float(len(values))
	print("time between {} - {} hours : uncovered_ratio {}".format(time1, time2, uncovered_ratio))

	return

if __name__ == '__main__':
	print("hello")

	path = "alice_step3_dynamicDeployment_version2/build/"
	filename = "data_Hist2D_time_KCoverage.txt"

	drawAPlot(path,filename,0,5)
	drawAPlot(path,filename,0,1)
	drawAPlot(path,filename,2.5,3.5)

	plt.show()
