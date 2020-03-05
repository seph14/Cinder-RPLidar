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

#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Resources.h"
#include "cinder/Xml.h"
#include "cinder/osc/Osc.h"
#include "cinder/Rand.h"
#include "cinder/Log.h"

#include "rplidar.h" 
#include "KMeans.h"

#include <iostream>

#define PRODUCTION 0
#define GROUP_MSG  1

using namespace rp::standalone::rplidar;
using namespace ci;
using namespace ci::app;
using namespace std;

const float distanceScale		= 1.f / 16.f;
using Sender					= osc::SenderUdp;
const uint16_t destinationPort	= 10001;
const uint16_t localPort		= 9999;
using SenderRef					= std::shared_ptr<Sender>;

static const int MAX_NODES		= 8192;
static const int MAX_POINTS		= 2048;
static const int MAX_CLUSTER	= 64;

class SampleApp : public App {
protected:
	// Kmeans section
	KMeans								mKmeans;
	bool								mDrawPoint, mDrawCluster, mUseRender, mActive;
	// Rendering section
	vector<vec2>						mPoints, mClusters;
	vector<vec3>						mFilters;
	int									mClusterCount, mHour, mMinute, NUM_THRESHOLD;
	gl::BufferTextureRef				mPointBuffer, mClusterBuffer;
	gl::VboRef							mInstanceDataVbo, mPointVbo, mClusterVbo;
	gl::BatchRef						mPointBatch, mClusterBatch;
	// OSC sender
	SenderRef							mSender;
	// lidar stuff
	vector<PointRef>					mPointData;
	shared_ptr<RPlidarDriver>			mDriver;
	rplidar_response_measurement_node_t nodes[MAX_NODES];
	size_t								count;
	float								mRotation, mSlope, mDirection;
	vec2								mPosition;
	vec4								mBoundary;

	bool isTimeup	 ();
	void turnoff	 ();
	bool grabScanData();
	bool checkRPLIDARHealth(shared_ptr<RPlidarDriver> drv);
	void initBatch	 ();
	void onSendError (asio::error_code error);

public:
	void setup() override;
	void keyDown( KeyEvent event ) override;
	void update() override;
	void draw() override;
	void cleanup() override;
};

void SampleApp::onSendError(asio::error_code error) {
	if (error) {
		CI_LOG_E("Error sending: " << error.message() << " val: " << error.value());
		try {
			mSender->close();
		} catch (const osc::Exception &ex) {
			CI_LOG_E("Cleaning up socket: val -" << ex.value());
		}
		quit();
	}
}

void SampleApp::initBatch() {
	geom::BufferLayout instanceDataLayout;
	instanceDataLayout.append(
		geom::Attrib::CUSTOM_0,
		geom::DataType::INTEGER, 1, 0, 0, 1);

	std::vector<int> positions(MAX_POINTS);
	for (size_t i = 0; i < MAX_POINTS; i++)
		positions[i] = i;

	mInstanceDataVbo = gl::Vbo::create(GL_ARRAY_BUFFER,
		positions.size() * sizeof(int), positions.data(), GL_STATIC_DRAW);

	mPoints		 = vector<vec2>(MAX_POINTS, vec2(65535.f));
	mPointVbo	 = gl::Vbo::create(GL_ARRAY_BUFFER,
		mPoints.size() * sizeof(vec2), mPoints.data(), GL_DYNAMIC_DRAW);
	mPointBuffer = gl::BufferTexture::create(mPointVbo, GL_RG32F);

	mClusters		= vector<vec2>(MAX_POINTS, vec2(65535.f));
	mClusterVbo		= gl::Vbo::create(GL_ARRAY_BUFFER,
		mClusters.size() * sizeof(vec2), mClusters.data(), GL_DYNAMIC_DRAW);
	mClusterBuffer	= gl::BufferTexture::create(mClusterVbo, GL_RG32F);

	auto glsl = gl::GlslProg::create(loadResource(SHADER_VERT), loadResource(SHADER_FRAG));
	glsl->uniform("uBuffer", 0);
	glsl->uniform("uScale", 1.f);

	auto plane = gl::VboMesh::create(geom::Circle());
	plane->appendVbo(instanceDataLayout, mInstanceDataVbo);
	mPointBatch = gl::Batch::create(plane, glsl,
		{ { geom::Attrib::CUSTOM_0, "vInstanceIdx" } });

	auto circle = gl::VboMesh::create(geom::Circle().radius(8.f));
	circle->appendVbo(instanceDataLayout, mInstanceDataVbo);
	mClusterBatch = gl::Batch::create(plane, glsl,
		{ { geom::Attrib::CUSTOM_0, "vInstanceIdx" } });
}

bool SampleApp::grabScanData() {
	u_result op_result = mDriver->grabScanData(nodes, count);

	if (IS_OK(op_result)) {
		std::fill(mPoints.begin(), mPoints.end(), vec2(655350.f));
		mDriver->ascendScanData(nodes, count);

		int idx = 0;
		for (int pos = 0; pos < (int)count; ++pos) {
			float a = mRotation + mDirection *
				glm::radians((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.f);
			float d = .1f * nodes[pos].distance_q2 / 4.f;

			if (d > 0.f) {
				vec2 p = mPosition + d * vec2(glm::cos(a), glm::sin(a));
				float rt = glm::clamp((p.x - mSlope) / (mBoundary.z - mSlope), 0.f, 1.f);
				float threshold = glm::lerp(mBoundary.y, mBoundary.w, rt);

				//slope check
				if (p.x >= mBoundary.x && p.x <= mBoundary.z &&
					p.y >= threshold && p.y <= mBoundary.w) {

					bool shouldAdd = true;
					//filter check
					for (auto filter : mFilters) {
						if (glm::distance2(p, vec2(filter.x, filter.y)) <
							filter.z * filter.z) {
							shouldAdd = false;
							break;
						}
					}

					if (shouldAdd) mPointData[idx++]->setPosition(p);
				}
			}
		}

		for (int pos = idx; pos < MAX_NODES; pos++) {
			mPointData[pos]->setPosition(65535.f, 65535.f);
		}
		return true;
	}
	return false;
}

bool SampleApp::checkRPLIDARHealth(shared_ptr<RPlidarDriver> drv) {
	u_result						 op_result;
	rplidar_response_device_health_t healthinfo;
	op_result = drv->getHealth(healthinfo);

	if (IS_OK(op_result)) {
		CI_LOG_I( "RPLidar health status: " << int(healthinfo.status) );
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
			CI_LOG_I("Error, rplidar internal error detected. Please reboot the device to retry.");
			drv->reset();
			return false;
		}
		else if (healthinfo.status == RPLIDAR_STATUS_OK) {
			CI_LOG_I("OK");
			return true;
		}
		else if (healthinfo.status == RPLIDAR_STATUS_WARNING) {
			CI_LOG_I("WARNING");
			return true;
		}
	} else {
		CI_LOG_I("Error, cannot retrieve the lidar health code: " << op_result);
		return false;
	}
}

bool SampleApp::isTimeup() {
	time_t now = time(0);
	struct tm tstruct;
	tstruct = *localtime(&now);
	return (tstruct.tm_hour >= mHour && tstruct.tm_min >= mMinute);
}

void SampleApp::turnoff() {
	if (!mActive) return;
	mActive = false;
	mDriver->stop();
	mDriver->stopMotor();
	mDriver->disconnect();
	mDriver.reset();
}

void SampleApp::cleanup() {
	turnoff();
}

void SampleApp::setup() {
	auto path = getAppPath();
	addAssetDirectory(path);
	mKmeans.setK(MAX_CLUSTER);

	mActive = false;
	mHour	= 20;
	mMinute = 0;
	string lidarPort = "\\\\.\\COM3";

	try {
		auto filepath = getAssetPath("") / "settings.xml";;
		auto file = new XmlTree(loadFile(filepath));
		auto params = file->getChild("params");

		float frameRate = glm::max(1.f, params.getChild("frameRate").getValue<float>());
		CI_LOG_V( "set frame rate to: " << frameRate );
		setFrameRate(frameRate);

		bool showview = (params.getChild("showView").getValue<string>() == "true");

		auto host = params.getChild("oscReceiver").getValue<string>();
		auto port = params.getChild("oscPort").getValue<uint16_t>();
		CI_LOG_V("setting OSC receiver to " << host);
		mSender = SenderRef(new Sender(port, host, destinationPort));
		try { mSender->bind(); }
		catch (const osc::Exception &ex) {
			CI_LOG_E("Error binding: " << ex.what() << " val: " << ex.value());
			quit();
		}

		auto lidar	= params.getChild("lidar");
		lidarPort	= "\\\\.\\" + lidar.getChild("port").getValue<string>();
		mRotation	= glm::radians(lidar.getChild("angle").getValue<float>());
		mPosition.x = lidar.getChild("position").getChild("x").getValue<float>();
		mPosition.y = lidar.getChild("position").getChild("y").getValue<float>();
		mBoundary.x = lidar.getChild("min").getChild("x").getValue<float>();
		mBoundary.y = lidar.getChild("min").getChild("y").getValue<float>();
		mBoundary.z = lidar.getChild("max").getChild("x").getValue<float>();
		mBoundary.w = lidar.getChild("max").getChild("y").getValue<float>();
		mSlope			= lidar.getChild("slope").getValue<float>();
		mDirection		= (lidar.getChild("topdown").getValue<string>() == "true") ? -1.f : +1.f;
		NUM_THRESHOLD	= lidar.getChild("threshold").getValue<int>();

		auto filters = params.getChild("filter");
		for (auto dot : filters) {
			float xx = dot.getChild("x").getValue<float>();
			float yy = dot.getChild("y").getValue<float>();
			float rr = dot.getChild("r").getValue<float>();
			CI_LOG_V("add filter: " << xx << "-" << yy << "-" << rr);
			mFilters.push_back(vec3(xx, yy, rr));
		}

		if (showview) {
			setWindowSize(mBoundary.z - mBoundary.x, mBoundary.w - mBoundary.y);
			mDrawPoint	 = true;
			mDrawCluster = true;
			mUseRender	 = true;
			initBatch();
		} else {
			setWindowSize(5, 5);
			mDrawPoint	 = false;
			mDrawCluster = false;
			mUseRender   = false;
		}

		auto time	= params.getChild("time");
		mHour		= time.getChild("hour").getValue<int>();
		mMinute		= time.getChild("minute").getValue<int>();
	} catch (ci::Exception ex) {
		console() << "error: " << ex.what() << endl;
	}

	isTimeup();

	const _u32 opt_com_baudrate		= 115200;
	std::wstring wPort				= std::wstring(lidarPort.begin(), lidarPort.end());
	const wchar_t * opt_com_path	= wPort.c_str();
	count	= _countof(nodes);
	mDriver = shared_ptr<RPlidarDriver>(RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT));

	mPointData.resize(count);
	for (int i = 0; i < count; i++)
		mPointData[i] = Point::create(i, vec2(65535.f, 65535.f));

	u_result op_result;
	rplidar_response_device_info_t devinfo;

	if (IS_OK(mDriver->connect(opt_com_path, opt_com_baudrate))) {
		op_result = mDriver->getDeviceInfo(devinfo);
		if (IS_OK(op_result)) {
			/*
			console() << "RPLIDAR S/N: ";
			for (int pos = 0; pos < 16; ++pos)
				console() << devinfo.serialnum[pos];
			console() << endl << "Firmware Ver: "
				<< (devinfo.firmware_version >> 8)
				<< (devinfo.firmware_version & 0xFF)
				<< endl << "Hardware Rev: "
				<< (int)devinfo.hardware_version << endl;
			*/
			checkRPLIDARHealth(mDriver);
			mDriver->startMotor();
			mDriver->startScan(false, true);
			mActive = true;
		}
	} else {
		mActive = false;
		quit();
	}

	gl::enableDepth(false);
}

void SampleApp::keyDown(KeyEvent event) {
	if (event.getCode() == KeyEvent::KEY_p) {
		mDrawPoint = !mDrawPoint;
	} else if (event.getCode() == KeyEvent::KEY_c) {
		mDrawCluster = !mDrawCluster;
	}
}

void SampleApp::update() {
	if (mActive && isTimeup())
		turnoff();

	if (!mActive) return;

	int pointSize = 0;
	vector<PointRef> copypoint;

	if (grabScanData()) {
		pointSize = count;
		for (size_t i = 0; i < count; i++) {
			auto pp = mPointData[i];
			copypoint.push_back(Point::create(pp->getID(), pp->getValues()));
		}
	}

	// if rendering debug view, update buffer
	if (mUseRender) {
		if (pointSize > 0) {
			std::fill(mPoints.begin(), mPoints.end(), vec2(655350.f));
			size_t min = ci::math<size_t>::min(mPoints.size(), pointSize);
			for (size_t dataCnt = 0; dataCnt < min; dataCnt++)
				mPoints[dataCnt] = copypoint[dataCnt]->getValues();
			mPointVbo->bufferData(mPoints.size() * sizeof(vec2), mPoints.data(), GL_DYNAMIC_DRAW);
		}
	}

	// kmeans pass
	if (pointSize > 0) {
		auto clusters = mKmeans.run(copypoint, pointSize);
		mClusterCount = clusters.size();
		if (mClusterCount > 0) {
			vector<vec2> data;

#if GROUP_MSG
			osc::Message dataMsg("/data/process");
			dataMsg.appendCurrentTime();

			static const int NUM_THRESHOLD = 0;

			int cnt = 0;
			for (int i = 0; i < mClusterCount; i++) {
				const auto clu = clusters[i];
				cnt += (clu->getTotalPoints() > NUM_THRESHOLD) ? 1 : 0;
			}
			dataMsg.append(cnt);

			for (int i = 0; i < mClusterCount; i++) {
				const auto clu = clusters[i];
				if (clu->getTotalPoints() > NUM_THRESHOLD) {
					vec2 pos = clu->getCentralValue();
					vec2 target = vec2((pos.x - mBoundary.x) / (mBoundary.z - mBoundary.x),
						(pos.y - mBoundary.y) / (mBoundary.w - mBoundary.y));
					dataMsg.append(target.x);
					dataMsg.append(target.y);
					if (mUseRender) data.push_back(pos);
				}
			}
			if (mUseRender)
				mClusterVbo->bufferData(data.size() * sizeof(vec2), data.data(), GL_DYNAMIC_DRAW);
			mSender->send(dataMsg, std::bind(&SampleApp::onSendError, this, std::placeholders::_1));
#else
			for (int i = 0; i < mClusterCount; i++) {
				const auto clu = clusters[i];
				if (clu->getTotalPoints() > NUM_THRESHOLD) {
					vec2 pos = clu->getCentralValue();
					vec2 target = vec2((pos.x - mBoundary.x) / (mBoundary.z - mBoundary.x),
						(pos.y - mBoundary.y) / (mBoundary.w - mBoundary.y));
					osc::Message msg("/data/0");
					msg.append(target.x);
					msg.append(target.y);
					mSender->send(msg, std::bind(&RPLidarApp::onSendError, this, std::placeholders::_1));
					if (mUseRender) data.push_back(pos);
				}
			}
			if (mUseRender)
				mClusterVbo->bufferData(data.size() * sizeof(vec2), data.data(), GL_DYNAMIC_DRAW);
#endif
		}
	}
}

void SampleApp::draw() {
	gl::clear(Color(0, 0, 0));
	if (!mUseRender || !mActive) return;

	gl::viewport(getWindowSize());
	gl::setMatricesWindow(getWindowSize());

	{
		gl::ScopedColor scpColor(Color(0, 0, 1));
		gl::drawSolidCircle(mPosition, 16.f);
	}

	if (mDrawPoint) {
		mPointBuffer->bindTexture();
		mPointBatch->getGlslProg()->uniform("uColor", vec4(1, 0, 0, 1));
		mPointBatch->getGlslProg()->uniform("uSize", 2.f);
		mPointBatch->drawInstanced(MAX_POINTS);
		mPointBuffer->unbindTexture();
	}

	if (mDrawCluster && mClusterCount > 0) {
		mClusterBuffer->bindTexture();
		mClusterBatch->getGlslProg()->uniform("uColor", vec4(0, 1, 0, 1));
		mClusterBatch->getGlslProg()->uniform("uSize", 8.f);
		mClusterBatch->drawInstanced(mClusterCount);
		mClusterBuffer->unbindTexture();
	}
}

#if PRODUCTION
CINDER_APP(SampleApp, RendererGl)
#else
CINDER_APP(SampleApp, RendererGl, [](App::Settings *settings) {
	settings->setConsoleWindowEnabled(true);
})
#endif
