#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <random>

#include "Units.hh"
#include "Histogram.hh"
#include "Histogram2D.hh"

// A star
#include "AStarNode.hh"
#include "PathPlanningAStar.hh"

// DynamicControlCentor
#include "DynamicRobotUnit.hh"
#include "DynamicControlCentor.hh"

using namespace std;

ofstream GLOBAL_write("data_robots_path.txt",ios::app);
string FileName_RobotDeploy = "../data_OneMonitorPoint.txt";

int		binSizes[2] = {10,10};
double	binWidths[2];
double	mins[2] = {-1000, -1000};
double 	maxs[2] = {1000, 1000};

vector<string> Zones_IDs;
vector<double> Zones_xs, Zones_ys, Zones_rs; // for SafeZonesAndDangerZones
vector<double> SafeZones_xs, SafeZones_ys, SafeZones_rs; // for SafeZonesAndDangerZones
bool ReadData_SafeZonesAndDangerZones(string filename1);

bool InitiateOneRobotManagement(int ID);

int main()
{
	cout<<"Hello "<<endl;

	string filename0_1 = "../../data_safeZonesAndDangerZones.txt";
	ReadData_SafeZonesAndDangerZones(filename0_1);

	// random
	default_random_engine e(time(0));
	uniform_real_distribution<double> er(-5000*m, 5000*m);
	uniform_real_distribution<double> etheta(0,2.*PI);

	for(int i=0;i<1000;i++)
	{
		//
		// set monitor point
		//
		ofstream write1(FileName_RobotDeploy);
		double r = er(e);
		double theta = etheta(e);
		double x = 22000*m + r*cos(theta); // m
		double y = 15000*m + r*sin(theta); // m
		//cout<<"ID "<<i<<", x y: "<<x/m<<" m, "<<y/m<<" m"<<endl;
		write1<<"0 0 0 "<<x<<" "<<y<<" Individual_FirstGen0"<<endl;
		write1.close();

		//
		// conpute
		//
		InitiateOneRobotManagement(i);
		
	}

	GLOBAL_write.close();


	return 1;
}


bool InitiateOneRobotManagement(int ID)
{
	// step 0 
	string filename0 = "../../alice_step1_part2_populationDensityGenerator_version3_aStarPathPlanning/data_histogram2d_parameters.txt";

	// step 0 : read obstacle data
	string obFileName1 = "../../alice_step1_part2_populationDensityGenerator_version3_aStarPathPlanning/build/data_step1_part2_tool_obstacles.txt";
	string obFileName2 = "../../alice_step1_part2_populationDensityGenerator_version3_aStarPathPlanning/build/data_step1_part2_tool_obstacles_binary.txt";

	// step 1 : DynamicControlCentor
	DynamicControlCentor * dcc = new DynamicControlCentor("DynamicControlCentor", filename0, obFileName1, obFileName2, FileName_RobotDeploy);

	//DynamicRobotUnit * rob = new DynamicRobotUnit("robottest", 0 , 0 , 0, 15);


	// step 0 : safe zones and danger zones
	// ../data_safeZonesAndDangerZones.txt
	dcc->Tool_SetSafeZones(SafeZones_xs, SafeZones_ys);

	// set time 
	//double maximumTimeInterval = 0.5*minute; // short time
	//double maximumTimeInterval = 50*minute;  // long time
	double maximumTimeInterval = 5*hour;  // long time
	dcc->Tool_SetMaximumTimeInterval(maximumTimeInterval);
	// initiate
	dcc->InitiateDynamicControl();

	// write

	//dcc->WriteRobotPath();
	//dcc->Tool_WriteHistogram2D(dcc->hDeployment_Time_KCoverage_, "data_Hist2D_time_KCoverage.txt");

	vector<DynamicRobotUnit *> robots_ = dcc->robots_;

	for(int i=0;i<robots_.size();i++)
	{
		int rID = i;

		string status_previous = "Nothing";
		for(int j=0;j<robots_[rID]->ThePassedPath_x_.size();j++)
		{
			int pID = j;

			string status = robots_[rID]->ThePassedPath_status_[pID];
			//if(status!="Flying_Work"&&status!="Flying_Recharge")
			if(status!=status_previous)
			{
				GLOBAL_write<<ID<<" ";
				GLOBAL_write<<dcc->dep_x_[0]/m<<" ";
				GLOBAL_write<<dcc->dep_y_[0]/m<<" ";
				GLOBAL_write<<robots_[rID]->name_<<" ";
				GLOBAL_write<<robots_[rID]->ThePassedPath_timeStamp_[pID]/second<<" ";
				GLOBAL_write<<robots_[rID]->ThePassedPath_x_[pID]/m<<" ";
				GLOBAL_write<<robots_[rID]->ThePassedPath_y_[pID]/m<<" ";
				GLOBAL_write<<robots_[rID]->ThePassedPath_status_[pID]<<endl;
			}

			status_previous = status;
		}

	}

	// delete
	delete dcc;

	return true;
}

bool ReadData_SafeZonesAndDangerZones(string filename1)
{
	ifstream file1(filename1.c_str());

	if(file1.fail())
	{
		cout<<"Can not find the file \" "<<filename1<<" \""<<endl;
		return 0;
	}

	string ID;
	double x, y, r;
	while(!file1.eof())
	{
		file1>>ID>>x>>y>>r;

		if(file1.eof()) break;

		Zones_IDs.push_back(ID);
		Zones_xs.push_back(x);
		Zones_ys.push_back(y);
		Zones_rs.push_back(r);

		// for safe zones
		if(ID[4]=='S') 
		{
			//cout<<"Safe Zone : "<<ID<<endl;
			SafeZones_xs.push_back(x);
			SafeZones_ys.push_back(y);
			SafeZones_rs.push_back(r);
		}

		//cout<<ID<<" "<<x<<" "<<y<<" "<<r<<endl;
	}

	file1.close();

	/*
	// debug
	for(int i=0;i<SafeZones_xs.size();i++)
	{
		cout<<"SafeZones x y "<<SafeZones_xs[i]<<" "<<SafeZones_ys[i]<<endl;
	}
	*/

	return true;
}
