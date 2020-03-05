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

#pragma once

#include "cinder/Cinder.h"
#include "cinder/gl/gl.h"
#include <string>

class Point;
class Cluster;
typedef std::shared_ptr<Point>   PointRef;
typedef std::shared_ptr<Cluster> ClusterRef;

class Point {
private:
	int id_point, id_cluster;
	ci::vec2 values;

public:
	Point(int id_point, ci::vec2 values);
	static PointRef create(int id, ci::vec2 val);
	const int getID() const;
	void  setCluster (const int id_cluster);
	void  setPosition(float x, float y);
	void  setPosition(ci::vec2 val);
	const int getCluster() const;
	const ci::vec2 getValues() const;
};

class Cluster{
private:
	int id_cluster;
	ci::vec2 central_values;
	std::vector<PointRef> points;
	bool needRecalculate;

public:
	Cluster(int id_cluster, PointRef point);
	static ClusterRef create(int id_cluster, PointRef point);
	void addPoint(PointRef point);
	bool removePoint(int id_point);
	const ci::vec2 getCentralValue() const;
	void setCentralValue(const ci::vec2 val);
	PointRef getPoint(int index) const;
	const int getTotalPoints() const;
	const int getID() const;
	const bool shouldRecalculate() { bool val = needRecalculate; needRecalculate = false; return val; }
};

class KMeans
{
private:
	int mK = 16; // number of clusters
	int max_iterations = 6;

	// return ID of nearest center (uses euclidean distance)
	const int getIDNearestCenter(std::vector<ClusterRef> clusters, const PointRef point, float* distance);

public:
	KMeans();
	KMeans(int K, int max_iterations);

	void setK(const int k) { mK = k; }
	void setMaxIteration(const int interation) { max_iterations = interation; }

	std::vector<ClusterRef> run(std::vector<PointRef> points, const int pointLength, const float threshold = 30.f);
};
