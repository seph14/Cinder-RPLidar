/*
 Copyright (c) 2018-2019, Seph Li - All rights reserved.
 This code is intended for use with the Cinder C++ library: http://libcinder.org
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and
 the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 the following disclaimer in the documentation and/or other materials provided with the distribution.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 */

#include "KMeans.h"
#include "cinder/Rand.h"
#include "cinder/app/App.h"

using namespace std;
using namespace ci;
using namespace ci::app;

Point::Point(int id_point, ci::vec2 values) {
	this->id_point = id_point;
	this->values.x = values.x;
	this->values.y = values.y;
	id_cluster = -1;
}

PointRef Point::create(int id, ci::vec2 val) {
	return PointRef( new Point(id,val) );
}

const int Point::getID() const {
	return id_point;
}

void Point::setCluster(const int id_cluster) {
	this->id_cluster = id_cluster;
}

void Point::setPosition(ci::vec2 val) {
	setPosition(val.x,val.y);
}

void Point::setPosition(float x, float y) {
	this->values.x = x;
	this->values.y = y;
}

const int Point::getCluster() const {
	return id_cluster;
}

const ci::vec2 Point::getValues() const {
	return values;
}

Cluster::Cluster(int id_cluster, PointRef point) {
	this->id_cluster = id_cluster;
	points = { point };
	central_values = point->getValues();
	point->setCluster(id_cluster);
	needRecalculate = false;
}

ClusterRef Cluster::create(int id_cluster, PointRef point) {
	return ClusterRef(new Cluster( id_cluster, point ));
}

void Cluster::addPoint(PointRef point) {
	needRecalculate = true;
	points.push_back(point);
}

bool Cluster::removePoint(const int id_point) {
	for (auto it = points.begin(); it != points.end(); it++) {
		if ((*it)->getID() == id_point) {
			points.erase(it);
			needRecalculate = true;
			return true;
		}
	}
	return false;
}

const ci::vec2 Cluster::getCentralValue() const {
	return central_values;
}

void Cluster::setCentralValue(const vec2 val) {
	central_values = val;
}

PointRef Cluster::getPoint(int index) const {
	return points[index];
}

const int Cluster::getTotalPoints() const {
	return points.size();
}

const int Cluster::getID() const {
	return id_cluster;
}

// return ID of nearest center (uses euclidean distance)
const int KMeans::getIDNearestCenter(vector<ClusterRef> clusters, PointRef point, float *distance) {
	int id_cluster_center = 0;
	float min_dist = glm::distance2(clusters[0]->getCentralValue(), point->getValues());
	for (int i = 1; i < clusters.size(); i++) {
		float dist = glm::distance2(clusters[i]->getCentralValue(), point->getValues());
		if (dist < min_dist) {
			min_dist = dist;
			id_cluster_center = i;
		}
	}
	*distance = min_dist;
	return id_cluster_center;
}

KMeans::KMeans(int K, int max_iterations) {
	mK = K;
	this->max_iterations = max_iterations;
}

KMeans::KMeans() {
	mK = 16;
	max_iterations = 6;
}

std::vector<ClusterRef> KMeans::run(vector<PointRef> points, const int pointLength, const float threshold) {
	int randIdx = ci::randInt(0, pointLength);
	std::vector<ClusterRef> clusters = { Cluster::create(0, points[randIdx]) };

	//return clusters;
	int iter = 1;
	float distance = 0.f;
	static const float distanceThreshold = threshold * threshold;

	while (true) {
		bool done = true;
		// associates each point to the nearest center
		for (size_t i = 0; i < pointLength; i++) {
			auto point = points[i];
			int id_old_cluster = point->getCluster();
			int id_nearest_center = getIDNearestCenter(clusters, point, &distance);

			if (point->getCluster() == -1 && clusters.size() < mK && distance > distanceThreshold) {
				//create a new cluster
				int idx = clusters.size();
				clusters.push_back(Cluster::create(idx, point));
				done = false;
			} else if (id_old_cluster != id_nearest_center) {
				//reassign to a new cluster
				if (id_old_cluster != -1 && id_old_cluster < clusters.size())
					clusters[id_old_cluster]->removePoint(point->getID());
				if (id_nearest_center < clusters.size()) {
					point->setCluster(id_nearest_center);
					clusters[id_nearest_center]->addPoint(point);
					done = false;
				}
			}
		}

		// recalculating the center of each cluster
		for (auto it = clusters.begin(); it != clusters.end(); ) {
			size_t total_points_cluster = (*it)->getTotalPoints();
			if ((*it)->shouldRecalculate() && total_points_cluster > 0) {
				vec2 sum = vec2(0.f);
				for (int p = 0; p < total_points_cluster; p++)
					sum += (*it)->getPoint(p)->getValues();
				(*it)->setCentralValue(vec2(
					sum.x / total_points_cluster,
					sum.y / total_points_cluster
				));
				it++;
			} else if (total_points_cluster <= 0) {
				it = clusters.erase(it);
			} else it++;
		}

		if (done == true || iter >= max_iterations) {
			break;
		}
		iter++;
	}

	return clusters;
}
