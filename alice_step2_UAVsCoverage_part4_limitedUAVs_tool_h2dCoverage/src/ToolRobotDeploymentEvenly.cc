#include "ToolRobotDeploymentEvenly.hh"

//-------------------------
// Constructor
//-------------------------
ToolRobotDeploymentEvenly::ToolRobotDeploymentEvenly(string name, int CoverageMode, int CrowdPathTimeID)
:	name_(name),
	CoverageMode_(CoverageMode)
{
	// debug
	cout<<"----------------------------------------------------------------"<<endl;
	cout<<"----"<<endl;
	cout<<"debug ToolRobotDeploymentEvenly::ToolRobotDeploymentEvenly"<<endl;
	cout<<"name "<<name_<<endl;
	cout<<"CoverageMode_ "<<CoverageMode_<<"; 1 for localization, 2 for coverage"<<endl;
	cout<<"----"<<endl;
	cout<<"----------------------------------------------------------------"<<endl;
	// !debug

	//  step 1 - 1 : import histogram data for buildings, which are obstacles
	binSizes[0] = 10;
	binSizes[1] = 10;
	mins[0]		= -1000;
	mins[1]		= -1000;
	maxs[0]		= 1000;
	maxs[1]		= 1000;

	// step 1 - 3 : import Robots Parameters
	RobotSize_ = 1;
	UAV_CommunicationRange_metre_ = 100*m;

	// 
	// step 1 : Import data
	// 
	// data file path : ../../alice_step1_part2_populationDensityGenerator_version3_aStarPathPlanning/
	string FolderPath_step1 = "../../alice_step1_part2_populationDensityGenerator_version3_aStarPathPlanning/";

	// step 1 - 0 : import histogram parameters
	bool isHistogramParaGood = Tool_ReadHistogramsParameters_ForBuildings(FolderPath_step1+"data_histogram2d_parameters.txt");

	//  step 1 - 1 : import histogram data for buildings, which are obstacles
	bool isBuildingsGood = Tool_ReadDistributionObstacles(FolderPath_step1+"build/data_step1_part2_tool_obstacles.txt");
	bool isBuildingsGood2 = Tool_ReadDistributionObstacles_binary(FolderPath_step1+"build/data_step1_part2_tool_obstacles_binary.txt");

	// step 1 - 2 : import data for crowds
	string filename = FolderPath_step1+"build/data_step1_part2_populationMaps_roundID_" + to_string(CrowdPathTimeID) + ".txt";
	bool isCrowdsGood = Tool_ReadPathForCrowds(filename);

	// step 1 - 3 : import Robots Parameters
	bool isRobotParametersGood = Tool_ReadRobotParameters("../../alice_step2_UAVsCoverage_part4_limitedUAVs/data_Robots_parameters.txt");

	/*
	// 
	// step 2 : compute coverage
	// 

	// step 2 - 1 : deploy the Robots (UAVs) evenly
	bool isDeployEvenlyGood = Deploy_step1_evenly();

	for(int i =0;i<robotCoor_x_1_all_.size();i++)
	{
		bool isRobotGood = Tool_IsRobotPositoinGood(robotCoor_x_1_all_[i],robotCoor_y_1_all_[i]); 

		if(isRobotGood)
		{
			robotCoor_x_2_.push_back(robotCoor_x_1_all_[i]);
			robotCoor_y_2_.push_back(robotCoor_y_1_all_[i]);
			//cout<<"Good : outside the buildings"<<endl;
		}
	}

	// compute histogram of coverage
	ComputeCoverage_DeployStep1();

	// write
	bool isWritingGood = Tool_WriteHistogram2D(Coverage_step1_, "data_distributionCoverage_coverStep1.txt");
	*/
}

//-------------------------
// Destructor
//-------------------------
ToolRobotDeploymentEvenly::~ToolRobotDeploymentEvenly()
{}

//-------------------------
// Private
// step 1 - 0 : import histogram parameters
//-------------------------
bool ToolRobotDeploymentEvenly::Tool_ReadHistogramsParameters_ForBuildings(string filename)
{
	ifstream file(filename.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<filename<<" \""<<endl;
		return 0;
	}

	string temp;
	file>>temp>>binSizes[0]>>mins[0]>>maxs[0];
	file>>temp>>binSizes[1]>>mins[1]>>maxs[1];

	mins[0] *= m;
    mins[1] *= m;
	maxs[0] *= m;
    maxs[1] *= m;

	binWidths[0] = (maxs[0]-mins[0])/double(binSizes[0]);
	binWidths[1] = (maxs[1]-mins[1])/double(binSizes[1]);

	cout<<"----------------------------------------------------------------"<<endl;
	cout<<"----"<<endl;
	cout<<"alice main : Tool_ReadHistogramsParameters_ForBuildings"<<endl;
	cout<<"x , bin "<<binSizes[0]<<", min "<<mins[0]/m<<" m, max "<<maxs[0]/m<<" m, binWidths "<<binWidths[0]/m<<" m"<<endl;
	cout<<"y , bin "<<binSizes[1]<<", min "<<mins[1]/m<<" m, max "<<maxs[1]/m<<" m, binWidths "<<binWidths[1]/m<<" m"<<endl;
	cout<<"----"<<endl;
	cout<<"----------------------------------------------------------------"<<endl;

	return 1;
}

//-------------------------
// Private
// step 1 - 1 : import histogram data for buildings, which are obstacles
//-------------------------
bool ToolRobotDeploymentEvenly::Tool_ReadDistributionObstacles(string filename)
{
	hObstacles_			= new Histogram2D("h",binSizes[0],mins[0],maxs[0],binSizes[1],mins[1],maxs[1]);

	ifstream file(filename.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<filename<<" \""<<endl;
		return 0;
	}

	double x,y;
	double content;
	while(!file.eof())
	{
		file>>x>>y>>content;

		if(file.eof()) break;

		hObstacles_->Fill(x,y,content);

		//cout<<"x y "<<x<<" "<<y<<" building "<<content<<endl;
	}

	file.close();

	return 1;
}

//-------------------------
// Private
// step 1 - 1 : import histogram data for buildings, which are obstacles
//-------------------------
bool ToolRobotDeploymentEvenly::Tool_ReadDistributionObstacles_binary(string filename)
{
	hObstacles_binary_	= new Histogram2D("h",binSizes[0],mins[0],maxs[0],binSizes[1],mins[1],maxs[1]);

	ifstream file(filename.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<filename<<" \""<<endl;
		return 0;
	}

	double x,y;
	double content;
	while(!file.eof())
	{
		file>>x>>y>>content;

		if(file.eof()) break;

		hObstacles_binary_->Fill(x,y,content);

		//cout<<"x y "<<x<<" "<<y<<" building (binary) "<<content<<endl;
	}

	file.close();

	return 1;
}

//-------------------------
// Private
// step 1 - 2 : import data for crowds
//-------------------------
bool ToolRobotDeploymentEvenly::Tool_ReadPathForCrowds(string filename)
{
	//debug
	cout<<"debug class ToolRobotDeploymentEvenly::Tool_ReadPathForCrowds"<<endl;
	cout<<filename<<endl;

	Histogram2D * pathMap = new Histogram2D("h",binSizes[0],mins[0],maxs[0],binSizes[1],mins[1],maxs[1]);

	ifstream file(filename.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<filename<<" \""<<endl;
		return 0;
	}

	double x,y;
	double content;
	while(!file.eof())
	{
		file>>x>>y>>content;

		if(file.eof()) break;

		pathMap->Fill(x,y,content);

		//cout<<"x y "<<x<<" "<<y<<" content "<<content<<endl;
	}

	file.close();

	//
	Paths_.push_back(pathMap);

	// find the maximum population per bin
	double contentMax = pathMap->GetBinContent(0,0);
	for(int i=0;i<binSizes[0];i++)
	for(int j=0;j<binSizes[1];j++)
	{
		int xID = i;
		int yID = j;

		double content = pathMap->GetBinContent(xID,yID);

		if(content>contentMax) contentMax=content;
	}
	MaximumPopulationPerBin_Paths_.push_back(contentMax);

	return 1;
}

//-------------------------
// Private
// step 1 - 3 : import Robots Parameters
//-------------------------
bool ToolRobotDeploymentEvenly::Tool_ReadRobotParameters(string filename)
{
	ifstream file(filename.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<filename<<" \""<<endl;
		return 0;
	}

	string name;
	double value;
	while(!file.eof())
	{
		file>>name>>value;

		if(file.eof()) break;

		if(name=="RobotSize") RobotSize_ = value;
		if(name=="UAV_CommunicationRange_metre") UAV_CommunicationRange_metre_ = value*m;

		//cout<<"name "<<name<<", value "<<value<<endl;
	}

	file.close();

	//
	cout<<"----------------------------------------------------------------"<<endl;
	cout<<"----"<<endl;
	cout<<"debug ToolRobotDeploymentEvenly::Tool_ReadRobotParameters"<<endl;
	cout<<"RobotSize_ "<<RobotSize_<<endl;
	cout<<"UAV_CommunicationRange_metre_ "<<UAV_CommunicationRange_metre_/m<<" m"<<endl;
	cout<<"----"<<endl;
	cout<<"----------------------------------------------------------------"<<endl;

	return 1;
}

//-------------------------
// Private
// step 2 - 1 : deploy the Robots (UAVs) evenly
//-------------------------
bool ToolRobotDeploymentEvenly::Deploy_step1_evenly()
{
	cout<<"debug ToolRobotDeploymentEvenly::Deploy_step1_evenly Begin..."<<endl;

	// 
	double RobotWidths[2] = {1,1};

	if(CoverageMode_==1)
	{
		// mode 1 : coverage for localization
		RobotWidths[0] = 1.7	*UAV_CommunicationRange_metre_/2.;  // x, sqrt(3)
		RobotWidths[1] = 1.5*2.	*UAV_CommunicationRange_metre_/2.;  // y, 1.5*2
	}

	if(CoverageMode_==2)
	{
		// mode 2 : maximum coverage
		RobotWidths[0] = 1.7	*UAV_CommunicationRange_metre_;  // x, sqrt(3)
		RobotWidths[1] = 1.5*2.	*UAV_CommunicationRange_metre_;  // y, 1.5*2
	}

	int RobotSizes[2] = {1,1};
	RobotSizes[0] = (maxs[0]-mins[0])/RobotWidths[0];
	RobotSizes[1] = (maxs[1]-mins[1])/RobotWidths[1];

	for(int i =0;i<RobotSizes[0]+1;i++)
	for(int j =0;j<RobotSizes[1]+1;j++)
	{
		int xID = i;
		int yID = j;

		// line 1 and line 2
		double rx1 = mins[0] +  double(xID)			*RobotWidths[0];
		double rx2 = mins[0] + (double(xID)+0.5)	*RobotWidths[0];

		double ry1 = mins[1] +  double(yID)			*RobotWidths[1];
		double ry2 = mins[1] + (double(yID)+0.5)	*RobotWidths[1];

		// line 1
		robotCoor_x_1_all_.push_back(rx1);
		robotCoor_y_1_all_.push_back(ry1);

		// line 2
		robotCoor_x_1_all_.push_back(rx2);
		robotCoor_y_1_all_.push_back(ry2);
	}

	//
	cout<<"x : RobotWidths "<<RobotWidths[0]<<", RobotSizes "<<RobotSizes[0]<<endl;
	cout<<"y : RobotWidths "<<RobotWidths[1]<<", RobotSizes "<<RobotSizes[1]<<endl;

	/*
	// debug
	for(int i =0;i<robotCoor_x_1_all_.size();i++)
	{
		cout<<"Robot ID : "<<i<<"; x y "<<robotCoor_x_1_all_[i]/m<<" m, "<<robotCoor_y_1_all_[i]/m<<" m"<<endl;
	}
	// !debug
	*/

	cout<<"debug ToolRobotDeploymentEvenly::Deploy_step1_evenly end."<<endl;

	return 1;
}

//-------------------------
// Private
// step 2 - 1 : deploy the Robots (UAVs) evenly
//-------------------------
bool ToolRobotDeploymentEvenly::ComputeCoverage_DeployStep1()
{
	//
	Coverage_step1_	= new Histogram2D("h",binSizes[0],mins[0],maxs[0],binSizes[1],mins[1],maxs[1]);

	for(int i=0;i<binSizes[0];i++)
	for(int j=0;j<binSizes[1];j++)
	{
		int xID = i;
		int yID = j;
		double x, y;
		Coverage_step1_->GetBinCentor2D(xID, yID, x, y);

		int NumberCovered = 0;
		for(int k=0;k<robotCoor_x_2_.size();k++)
		{
			int rID = k;
			double rx = robotCoor_x_2_[rID];
			double ry = robotCoor_y_2_[rID];

			double d2 = (x-rx)*(x-rx) + (y-ry)*(y-ry);
			double d = sqrt(d2);
			if(d<=1.*UAV_CommunicationRange_metre_)
			{
				NumberCovered ++;
			}
		}

		Coverage_step1_->Fill(x,y,NumberCovered);
	}

	//Coverage_step1_->Show();
	//
	return 1;
}

//-------------------------
// Private
// Tools
//-------------------------
bool ToolRobotDeploymentEvenly::Tool_IsRobotPositoinGood(double x, double y)
{
	// 1 : is inside the map
	if(x<mins[0]||x>=maxs[0]) return false;
	if(y<mins[1]||y>=maxs[1]) return false;

	// 2 : is outside the buildings
	int xID = hObstacles_binary_->GetBinIDX(x);
	int yID = hObstacles_binary_->GetBinIDY(y);

	if(xID==-1||yID==-1) return false; // is inside the map
	double content = hObstacles_binary_->GetBinContent(xID,yID);

	if(content==1)
	{
		//cout<<"Inside the building"<<endl;
		return false;
	}

	return true;
}

//-------------------------
// Private
// Tools
//-------------------------
bool ToolRobotDeploymentEvenly::Tool_WriteHistogram2D(Histogram2D *h, string filename)
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
