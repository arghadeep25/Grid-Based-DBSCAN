#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <limits>
#include <cmath>
#include <stack>
#include <bits/stdc++.h>
#include <algorithm>
#include <string>
#include <cctype>
#include <time.h>
#include <chrono>

using namespace std;


class region
{
  public:
	float x_coord, y_coord;
	int cluster_id = 0, region_type = 1, reg_minPts = 0;
	vector<int> mainCorePt;
	int traced = 0;
	region() {}
	region(float a_coord, float b_coord, int id)
	{
		x_coord = a_coord;
		y_coord = b_coord;
		cluster_id = id;
	}
};


struct BBox
{
	double xMin;
	double xMax;
	double yMin;
	double yMax;
	int cluster;
};


struct Mat
{
	double covXX;
	double covXY;
	double covYX;
	double covYY;
	double meanX;
	double meanY;
};
