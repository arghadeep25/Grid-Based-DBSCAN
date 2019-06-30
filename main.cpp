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
#include "dbscan.h"

using namespace std;



float stringToFloat(string i)
{
	stringstream sf;
	float score = 0;
	sf << i;
	sf >> score;
	return score;
}


vector<region> openFile(const char *pointCloud)
{
	fstream file;
	file.open(pointCloud, ios::in);
	if (!file)
	{
		cout << "Open File Failed!" << endl;
		vector<region> a;
		return a;
	}
	vector<region> data;
	int i = 1;
	while (!file.eof())
	{
		string temp;
		file >> temp;
		int split = temp.find(',', 0);
		region p(stringToFloat(temp.substr(0, split)),
				stringToFloat(temp.substr(split + 1, temp.length() - 1)), i++);
		data.push_back(p);
	}
	file.close();
	cout << "successful!" << endl;
	return data;
}


float sq_dist(region a, region b)
{
	return sqrt((a.x_coord - b.x_coord) * (a.x_coord - b.x_coord) +
  (a.y_coord - b.y_coord) * (a.y_coord - b.y_coord));
}


vector<region> gridBasedDBSCAN(vector<region> pointCloud, float eps, int minPts)
{
	vector<region> results;
	int pcdSize = pointCloud.size();
	//calculate pts
	cout << "calculate pts" << endl;
	for (int i = 0; i < pcdSize; i++)
	{
		for (int j = i + 1; j < pcdSize; j++)
		{
			if (sq_dist(pointCloud[i], pointCloud[j]) < eps)
				pointCloud[i].reg_minPts++;
			pointCloud[j].reg_minPts++;
		}
	}
	//core region
	cout << "core region " << endl;
	vector<region> coreregion;
	for (int i = 0; i < pcdSize; i++)
	{
		if (pointCloud[i].reg_minPts >= minPts)
		{
			pointCloud[i].region_type = 3;
			coreregion.push_back(pointCloud[i]);
		}
	}
	cout << "joint core region" << endl;
	//joint core region
	for (int i = 0; i < coreregion.size(); i++)
	{
		for (int j = i + 1; j < coreregion.size(); j++)
		{
			if (sq_dist(coreregion[i], coreregion[j]) < eps)
			{
				coreregion[i].mainCorePt.push_back(j);
				coreregion[j].mainCorePt.push_back(i);
			}
		}
	}
	for (int i = 0; i < coreregion.size(); i++)
	{
		stack<region *> pointsource;
		if (coreregion[i].traced == 1)
			continue;
		pointsource.push(&coreregion[i]);
		region * temp;
		while (!pointsource.empty())
		{
			temp = pointsource.top();
			temp->traced = 1;
			pointsource.pop();
			for (int j = 0; j < temp->mainCorePt.size(); j++)
			{
				if (coreregion[temp->mainCorePt[j]].traced == 1)
					continue;
				coreregion[temp->mainCorePt[j]].cluster_id = coreregion[i].cluster_id;
				coreregion[temp->mainCorePt[j]].traced = 1;
				pointsource.push(&coreregion[temp->mainCorePt[j]]);
			}
		}
	}
	cout << "border region,joint border region to core region" << endl;
	//border region,joint border region to core region
	for (int i = 0; i < pcdSize; i++)
	{
		if (pointCloud[i].region_type == 3)
			continue;
		for (int j = 0; j < coreregion.size(); j++)
		{
			if (sq_dist(pointCloud[i], coreregion[j]) < eps)
			{
				pointCloud[i].region_type = 2;
				pointCloud[i].cluster_id = coreregion[j].cluster_id;
				break;
			}
		}
	}
	cout << "output" << endl;
	//output
	fstream clustering;
	clustering.open("clustered_result.txt", ios::out);
	for (int i = 0; i < pcdSize; i++)
	{
		if (pointCloud[i].region_type == 2)
			clustering << pointCloud[i].x_coord << "," <<
      pointCloud[i].y_coord << "," << pointCloud[i].cluster_id << "\n";
	}
	for (int i = 0; i < coreregion.size(); i++)
	{
		clustering << coreregion[i].x_coord << "," <<
    coreregion[i].y_coord << "," << coreregion[i].cluster_id << "\n";
	}
	clustering.close();
	return coreregion;
}


vector<region> noiseRemoval(vector<region> clusteredData, int threshold)
{
	vector<region> results;
	vector<int> clusters;
	vector<int> clusterscopy;
	for (int i = 0; i < clusteredData.size(); i++)
	{
		clusters.push_back(clusteredData[i].cluster_id);
	}
	clusterscopy = clusters;
	sort(clusters.begin(), clusters.end());
	auto last = std::unique(clusters.begin(), clusters.end());
	clusters.erase(last, clusters.end());

	for (int i = 0; i < clusteredData.size(); i++)
	{
		for (int j = 0; j < clusters.size(); j++)
		{
			if (clusteredData[i].cluster_id == clusters[j] &&
				count(clusterscopy.begin(), clusterscopy.end(), clusters[j]) > threshold)
			{
				results.push_back(clusteredData[i]);
			}
		}
	}
	fstream result;
	result.open("noise_removed_clustered_2.txt", ios::out);
	for (int j = 0; j < results.size(); j++)
	{
		result << results[j].x_coord << "," <<
    results[j].y_coord << "," << results[j].cluster_id << "\n";
	}
	result.close();
	return results;
}


// Mean Value
double mean(vector<double> mat){
	// vector<double> mat;
	double sum = 0;
	for(int i = 0; i < mat.size(); i++){
		sum = sum + mat[i];
	}
  	return sum/mat.size();
}


// Covariance Matrix
double covariance(vector<double> mat1, vector<double> mat2){
  double sum = 0;
  for(int i = 0; i < mat1.size(); i++){
    sum = sum + (mat1[i] - mean(mat1))*(mat2[i] - mean(mat2));
  }
//   cout << "Sum: " << sum << endl;
  return sum / (mat1.size()-1);
}


// change in function also.
void boundingBox(vector<region> noisedRemovedResults,
                 vector<BBox> &clusteredBoundingBox,
                 vector<Mat> &covMatrix)
{
	// vector<BBox> clusteredBoundingBox;
	vector<int> clusters;
	vector<double> valX;
	vector<double> valY;
	double meanX, meanY;
	double covXX, covXY, covYX, covYY;
	//vector<vector<double>> covMatrix;

	// Sort and extract the number of clusters
	// Clusters val - Start

	covMatrix.clear();
	for(int i = 0; i < noisedRemovedResults.size(); i++)
	{
		clusters.push_back(noisedRemovedResults[i].cluster_id);
	}
	sort(clusters.begin(), clusters.end());
	auto last = std::unique(clusters.begin(), clusters.end());
	clusters.erase(last, clusters.end());
	// Clusters val - End
	for(int i = 0; i < clusters.size(); i++){
		cout << clusters[i] << endl;
	}
	for(int i = 0; i < clusters.size(); i++)
	{
		valX.clear();
		valY.clear();
		for(int j =0; j < noisedRemovedResults.size(); j++)
		{	// Matching the region cloud with cluster vector
			if(noisedRemovedResults[j].cluster_id == clusters[i])
			{
				valX.push_back(noisedRemovedResults[j].x_coord);
				valY.push_back(noisedRemovedResults[j].y_coord);
			}

		}

		sort(valX.begin(), valX.end());
		sort(valY.begin(), valY.end());
		int sizeX = valX.size();
		int sizeY = valY.size();

		cout << "valX size: " << valX.size() << endl;
		covXX = covariance(valX, valX);
		covXY = covariance(valX, valY);
		covYX = covariance(valY, valX);
		covYY = covariance(valY, valY);
		meanX = mean(valX);
		meanY = mean(valY);

		//lastest change
		Mat val{covXX, covXY, covYX, covYY, meanX, meanY};
		covMatrix.push_back(val);
		// until here


		cout << "cov(XX): " << covXX << "  cov(XY): " << covXY << endl;
		cout << "cov(YX): " << covYX << "  cov(YY): " << covYY << endl;


		cout << "Min X: "<<valX[0] << "& Max X: " << valX[sizeX-1]<<endl;
		cout << "Min Y: "<<valY[0] << "& Max Y: " << valY[sizeY-1]<<endl;

		clusteredBoundingBox.push_back(BBox());
		clusteredBoundingBox[i].xMin = valX[0];
		clusteredBoundingBox[i].xMax = valX[sizeX - 1];
		clusteredBoundingBox[i].yMin = valY[0];
		clusteredBoundingBox[i].yMax = valY[sizeY - 1];
		clusteredBoundingBox[i].cluster = clusters[i];
	}
}


int main(int argc, char **argv)
{
	auto start = chrono::steady_clock::now();
	const char *input = argv[1];
	vector<region> results;
	vector<region> noisedRemoved_results;
	vector<BBox> final_results;
	vector<Mat> covMatrix;

	vector<region> pointCloud = openFile(input);
	results = gridBasedDBSCAN(pointCloud, 5, 15);
	auto end = chrono::steady_clock::now();
	cout << results.size() << endl;
	noisedRemoved_results = noiseRemoval(results, 5);
	cout << "Result size " << noisedRemoved_results.size() << endl;
	boundingBox(noisedRemoved_results, final_results, covMatrix);
	cout << "No of Bounding Boxes" << final_results.size()<< endl<<endl;
	// for(int i = 0; i < covMatrix.size(); i++){
	// 	for(int j = 0; j < covMatrix[i].size(); j++){
	// 		cout << covMatrix[i][j] << " ";
	// 	}
	// 	cout << endl;
	// }
	for(int i = 0; i < final_results.size(); i++){
		fstream bbox;
		string num = to_string(i);
		string filename = "bbox_" + num + string(".txt");
		const char* name = filename.c_str();
		bbox.open(name, ios::out);
		bbox << final_results[i].xMin << "," << final_results[i].yMin<<endl;
		bbox << final_results[i].xMax << "," << final_results[i].yMax<<endl;
		bbox.close();
	}
	auto diff = end - start;
	cout << chrono::duration <double, milli> (diff).count() << " ms" << endl;
	return 0;
}
