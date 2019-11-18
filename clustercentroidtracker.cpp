#include "clustercentroidtracker.h"

#include "iostream"
#include <numeric>
#include <algorithm>

#include "Eigen/Dense"
#include "pcl/common/centroid.h"

CentroidTracker::CentroidTracker()
{
	maxDisappeared_ = 10;
}

CentroidTracker::CentroidTracker(int maxDisappeared)
{
	maxDisappeared_ = maxDisappeared;
}

void CentroidTracker::Update(std::vector<XYZcloudPtr> inputClusters)
{
	// list of centroids is empty increment through all objects and 
	// add to their dissapeared frame count
	// return early as there is nothing to update
	if (inputClusters.size() == 0) {
		std::map<int, int>::iterator it = disappeared.begin();
		while(it != disappeared.end()) {
			int objectID = it->first;
			UpdateDisappeared(objectID);
		}
		return;
	}

	// calculate centroids
	std::vector<Eigen::Vector3f> inputCentroids;
	inputCentroids.reserve(inputClusters.size());
	for (XYZcloudPtr cl : inputClusters) {
		Eigen::Vector4f center;
		pcl::compute3DCentroid(*cl, center);
		inputCentroids.emplace_back(center(0), center(1), center(2));
	}

	// if we are not tracking any objects register all centroids, 
	// otherwise we are tracking objects, so need to match the input centroids
	// with existing centroids
	if (objects.size() == 0) {
		for (size_t i = 0; i < inputClusters.size(); i++) {
			//Register(inputClusters[i], inputCentroids[i]);
			Eigen::Vector3f zeroVec(0, 0, 0);
			RegisterWithDirection(inputClusters[i], inputCentroids[i], zeroVec);
		}
	}
	else {
		Eigen::MatrixXf D = GetDistanceMatrix(inputCentroids);
		//std::cout << "\n" << D << "\n";

		// convert rowwise minimum values to tuples <distance, row, col>, sort by distance
		std::vector<std::tuple<float, int, int>> tuples;
		Eigen::MatrixXf::Index rowIdx, colIdx;
		float minDist = 0;

		for (size_t i = 0; i < D.rows(); i++) {
			minDist = D.block(i, 0, 1, D.cols()).minCoeff(&rowIdx, &colIdx);
			tuples.push_back(std::make_tuple(minDist, i, colIdx));
		}
		std::sort(tuples.begin(), tuples.end());

		// loop over tuples, check if used, get object ID from row,
		// update centroid, reset dissapeeared, add to used
		std::vector<int> usedRows;
		std::vector<int> usedCols;

		for (std::tuple<float, int, int> t : tuples) {
			int row = std::get<1>(t);
			int col = std::get<2>(t);

			bool inUsedRows = std::find(usedRows.begin(), usedRows.end(), row) != usedRows.end();
			bool inUsedCols = std::find(usedCols.begin(), usedCols.end(), col) != usedCols.end();

			if (inUsedRows || inUsedCols) {
				continue;
			}
			else {
				std::map<int, Eigen::Vector3f>::iterator it = objects.begin();
				std::advance(it, row);
				int objectID = it->first;

				// if object doesnt have direction vector yet or if previous direction
				// is consistent with the new direction, update point 
				Eigen::Vector3f newDir = inputCentroids[col] - objects[objectID];

				if ((direction[objectID](0) == 0.0) || (newDir.norm() < 3)) {
					objects[objectID] = inputCentroids[col];
					clusters[objectID] = inputClusters[col];
					direction[objectID] = newDir;
					disappeared[objectID] = 0;
					usedRows.push_back(row);
					usedCols.push_back(col);
				}


			}
		}
		// find unused rows / columns, 
		std::vector<int> unusedRows;
		std::vector<int> unusedCols;

		std::vector<int> rowSeq(D.rows());
		std::vector<int> colSeq(D.cols());

		std::iota(rowSeq.begin(), rowSeq.end(), 0);
		std::iota(colSeq.begin(), colSeq.end(), 0);

		std::sort(usedRows.begin(), usedRows.end());
		std::sort(usedCols.begin(), usedCols.end());

		std::set_difference(rowSeq.begin(), rowSeq.end(),
			usedRows.begin(), usedRows.end(), std::back_inserter(unusedRows));
		std::set_difference(colSeq.begin(), colSeq.end(),
			usedCols.begin(), usedCols.end(), std::back_inserter(unusedCols));
		
		// if distance matrix has more rows than columns, objects at unused rows have 
		// dissapeared, otherwise unused columns represent new objects are new
		if (D.rows() >= D.cols()) {
			for (int r : unusedRows) {
				std::map<int, Eigen::Vector3f>::iterator it = objects.begin();
				std::advance(it, r);
				int objectID = it->first;
				UpdateDisappeared(objectID);
			}
		}
		else {
			for (int c : unusedCols) {
				//Register(inputClusters[c], inputCentroids[c]);
				Eigen::Vector3f zeroVec(0, 0, 0);
				RegisterWithDirection(inputClusters[c], inputCentroids[c], zeroVec);
			}
		}
		
	}
}

void CentroidTracker::Register(XYZcloudPtr cluster, Eigen::Vector3f &centroid)
{
	// save centroid to a map under latest ID, write that it has
	// dissapeared for 0 frames, increment object ID for next centroid
	objects[nextObjectID_] = centroid;
	clusters[nextObjectID_] = cluster;
	disappeared[nextObjectID_] = 0;
	nextObjectID_++;
}

void CentroidTracker::RegisterWithDirection(XYZcloudPtr cluster, 
	Eigen::Vector3f &centroid, Eigen::Vector3f &dir)
{
	// save centroid to a map under latest ID, write that it has
	// dissapeared for 0 frames, increment object ID for next centroid
	objects[nextObjectID_] = centroid;
	clusters[nextObjectID_] = cluster;
	direction[nextObjectID_] = dir;
	disappeared[nextObjectID_] = 0;
	nextObjectID_++;
}

void CentroidTracker::UpdateDisappeared(int objectID)
{
	// increment dissapeared count for object ID if object 
	// dissapeared for too long remove it from the objects
	disappeared[objectID]++;
	if (disappeared[objectID] > maxDisappeared_)
		Deregister(objectID);
}

void CentroidTracker::PrintObjects()
{
	if (objects.size() == 0) {
		std::cout << "tracker is empty" << std::endl;
	}
	else {
		for (auto it = objects.begin(); it != objects.end(); it++) {
			int objectID = it->first;
			std::cout << "id_" << objectID <<
				" direction: " << direction[objectID].transpose() <<
				" n. pts: " << clusters[objectID]->points.size() <<
				" disap. count: " << disappeared[it->first] << std::endl;
		}
	}
	std::cout << "\n";
}



void CentroidTracker::Deregister(int objectID)
{
	objects.erase(objectID);
	clusters.erase(objectID);
	disappeared.erase(objectID);
	direction.erase(objectID);
}


Eigen::MatrixXf CentroidTracker::GetDistanceMatrix(std::vector<Eigen::Vector3f>& centroids)
{
	int nObjects = objects.size();
	int nCentroids = centroids.size();
	Eigen::ArrayX3f objArray(nObjects, 3), centArray(nCentroids, 3);
	
	// convert object map and centroids vector into ArrayX3f
	std::map<int, Eigen::Vector3f>::iterator it = objects.begin();
	for (size_t i = 0; i < nObjects; i++) {
		objArray.row(i) = it->second;
		it++;
	}
	
	for (size_t i = 0; i < nCentroids; i++) {
		centArray.row(i) = centroids[i];
	}

	// compose euclidean distance matrix
	Eigen::ArrayXXf dist(nObjects, nCentroids);
	for (int s = 0; s < nCentroids; s++)
	{
		dist.col(s) =
			(objArray.rowwise() - centArray.row(s)).matrix().rowwise().norm();
	}

	//std::cout << objArray;
	//std::cout << "\n\n" << centArray;
	//std::cout << "\n\n" << dist;

	return dist;
}
