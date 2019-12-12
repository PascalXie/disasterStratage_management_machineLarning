#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <random>

#include "Units.hh"
#include "Histogram.hh"
#include "Histogram2D.hh"

using namespace std;

ofstream GLOBAL_write("data_trainningSamples.txt");
//string FileName_RobotDeploy = "../data_OneMonitorPoint.txt";

Histogram * hStatus = new Histogram("StatusSize",20,-0.5,19.5);
Histogram * hRobots = new Histogram("RobotsSize",10,-0.5,9.5);


//
bool ReadData(string filename1);

//
bool CheckStatusSequence(vector<string> status_, vector<double> time_);
string GetStatusAfterThis(string status_previous);

//
bool WriteHist(string filename, Histogram *h);

int main()
{
	cout<<"Hello "<<endl;

	// step 1 : import robot path
	string filename0_1 = "../../alice_step3_dynamicDeployment_version2/build/data_robots_path.txt";
	ReadData(filename0_1);


	// 
	GLOBAL_write.close();

	hStatus->Normalize();
    hRobots->Normalize();

	hStatus->Show();
    hRobots->Show();

	WriteHist("data_hist_statusSize.txt",hStatus);
	WriteHist("data_hist_robotsSize.txt",hRobots);

	return 1;
}

bool WriteHist(string filename, Histogram *h)
{
	ofstream write(filename);

	for(int i=0;i<h->binSize_;i++)
	{
		int xID = i;
		double x = h->GetBinCenter(xID);
		double c = h->GetBinContent(xID);

		write<<xID<<" "<<x<<" "<<c<<endl;
	}

	write.close();

	return true;
}

bool ReadData(string filename1)
{
	ifstream file1(filename1.c_str());

	if(file1.fail())
	{
		cout<<"Can not find the file \" "<<filename1<<" \""<<endl;
		return 0;
	}

	//
	vector<string> status_;
	vector<double> time_;

	//
	double ID=-1;
	double dep_x, dep_y;
	string robname;
	double robtime;
	double rob_x, rob_y;
	string status;

	double ID_pre=-2;
	string robname_pre;

	double RobotCounter_PerEvent = 0;
	double IDCounter = -1;
	while(!file1.eof())
	{
		file1>>ID>>dep_x>>dep_y>>robname>>robtime>>rob_x>>rob_y>>status;

		if(file1.eof()) break;

		dep_x *= m;
		dep_y *= m;

		robtime *= second;

		rob_x *= m;
		rob_y *= m;

		double d2 = (dep_x-rob_x)*(dep_x-rob_x) + (dep_y-rob_y)*(dep_y-rob_y);
		double d = sqrt(d2);

		//cout<<ID<<" "<<dep_x/m<<" "<<dep_y/m<<" "<<robname<<" "<<robtime/second<<" "<<rob_x/m<<" "<<rob_y/m<<" "<<status<<", d "<<d/m<<endl;

		//
		// generate trainning-smaple matrix
		//
		if(ID!=ID_pre||robname!=robname_pre)
		{
			bool IsSequqnceGood = CheckStatusSequence(status_, time_);
			if(IsSequqnceGood==false)
			{
				cout<<"!!!!!!!!!!!!"<<endl;
			}
			else if(status_.size()!=0)
			{
				GLOBAL_write<<IDCounter<<" ";
				for(int i=0;i<status_.size();i++)
				{
					GLOBAL_write<<time_[i]<<" ";
				}
				GLOBAL_write<<endl;
			}

			//
			// generate histogram of status size and robot size
			//
			if(IDCounter!=-1)
			{
				hStatus->Fill(status_.size(),1);
				RobotCounter_PerEvent ++;
			}

			//
			// clear
			//
			status_.clear();
			time_.clear();
		}

		if(ID!=ID_pre)
		{
			IDCounter ++;

			//
			// generate histogram of status size and robot size
			//
			if(IDCounter!=0)
			{
				hRobots->Fill(RobotCounter_PerEvent,1);
			}

			RobotCounter_PerEvent = 0;
		}

		status_.push_back(status);
		time_.push_back(robtime);

		
		ID_pre = ID;
		robname_pre = robname;
	}

	file1.close();

	return true;
}

bool CheckStatusSequence(vector<string> status_, vector<double> time_)
{
	bool IsSequenceGood = true;

	if(status_.size()!=0&&status_[0]!="Free")
		IsSequenceGood = false;

	for(int i=1;i<status_.size();i++)
	{
		if(status_[i]!=GetStatusAfterThis(status_[i-1]))
			IsSequenceGood = false;
	}

	return IsSequenceGood;
}

string GetStatusAfterThis(string status_previous)
{
	string status_current = "Nothing";

	if(status_previous=="Free")
		status_current = "Flying_Work";

	else if(status_previous=="Flying_Work")
		status_current = "Working";

	else if(status_previous=="Working")
		status_current = "Flying_Recharge";

	else if(status_previous=="Flying_Recharge")
		status_current = "Recharging";

	else if(status_previous=="Recharging")
		status_current = "Free";


	return status_current;
}
