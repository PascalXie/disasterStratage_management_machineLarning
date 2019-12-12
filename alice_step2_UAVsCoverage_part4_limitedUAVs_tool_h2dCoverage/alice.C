#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <random>

#include "Units.hh"
#include "Histogram.hh"
#include "Histogram2D.hh"

#include "ToolRobotDeploymentEvenly.hh"

ToolRobotDeploymentEvenly * dep;
vector<double> x_, y_;

int		binSizes[2];
double	mins[2];
double 	maxs[2];
double	binWidths[2];

void ComputeDeployment(string filename, int GenerationID);
bool ReadDeployment(string filename);
bool Tool_WriteHistogram2D(Histogram2D *h, string filename);

using namespace std;

int main()
{
	cout<<"Hello "<<endl;

	// step 0
	dep = new ToolRobotDeploymentEvenly("deploy", 1, 2);
	for(int i=0;i<2;i++)
	{
		binSizes[i] = dep->binSizes[i];
		mins[i]		= dep->mins[i];
		maxs[i]		= dep->maxs[i];
		binWidths[i]= dep->binWidths[i];
	}

	// step 1 
	for(int i=0;i<10;i++)
	{
		int GenerationID = i;
		string filename = "../../alice_step2_UAVsCoverage_part4_limitedUAVs/build/data_Generation_"+to_string(GenerationID)+".txt";
		ComputeDeployment(filename, GenerationID);
	}

	return 1;

}

void ComputeDeployment(string filename, int GenerationID)
{
	// step 1 
	ReadDeployment(filename);

	// step 2 : histogram 2d
	Histogram2D	* hdep= new Histogram2D("h",binSizes[0],mins[0],maxs[0],binSizes[1],mins[1],maxs[1]);
	for(int i =0;i<binSizes[0];i++)
	for(int j =0;j<binSizes[1];j++)
	{
		int xID = i;
		int yID = j;
		double x, y;
		hdep->GetBinCentor2D(xID, yID, x, y);

		int NumberCovered = 0;
		for(int k=0;k<x_.size();k++)
		{
			int rID = k;
			double rx = x_[rID];
			double ry = y_[rID];
			double d2 = (x-rx)*(x-rx) + (y-ry)*(y-ry);
			double d = sqrt(d2);
			if(d<=1.*dep->UAV_CommunicationRange_metre_)
			{
				NumberCovered ++;
			}
		}

		hdep->Fill(x,y,NumberCovered);
	}

	// step 3 : write
	Tool_WriteHistogram2D(hdep, "data_deployment_generation_"+to_string(GenerationID)+".txt");
}

bool Tool_WriteHistogram2D(Histogram2D *h, string filename)
{
	ofstream write(filename);

	for(int i=0;i<binSizes[0];i++)
	for(int j=0;j<binSizes[1];j++)
	{
		int xID = i;
		int yID = j;

		double x,y;
		h->GetBinCentor2D(xID, yID, x, y);

		double content = h->GetBinContent(xID, yID);

		write<<x<<" "<<y<<" "<<content<<endl;
	}

	write.close();

	return true;
}

bool ReadDeployment(string filename)
{
	cout<<filename<<endl;

	x_.clear();
	y_.clear();

	ifstream file(filename.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<filename<<" \""<<endl;
		return 0;
	}

	int IndividualRank = 0;
	int GeneID = 0;
	double Fitness = 0;
	double x,y;
	string name;
	while(!file.eof())
	{
		file>>IndividualRank>>GeneID>>Fitness>>x>>y>>name;

		if(file.eof()) break;

		if(IndividualRank==9)
		{
			x_.push_back(x);
			y_.push_back(y);

			cout<<"x y "<<x<<" "<<y<<endl;
		}
	}
	file.close();

	return 1;
}
