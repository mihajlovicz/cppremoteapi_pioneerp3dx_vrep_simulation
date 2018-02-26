/*
	Client of V-REP simulation server (remoteApi)
	Copyright (C) 2015  Rafael Alceste Berri rafaelberri@gmail.com

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Habilite o server antes na simulação V-REP com o comando lua:
// simExtRemoteApiStart(portNumber) -- inicia servidor remoteAPI do V-REP

extern "C" {
#include "remoteApi/extApi.h"
}

#include <iostream>
#include <string>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <math.h> 
#include "EuclideanClusterExtraction.h"
#include "mass_center.h"
#include "RVOSimulator.h"
#include "differential_drive.h"
#include <future>

using namespace std;

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> detekcija_ciljeva(pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_point_cloud);
Eigen::Vector3f centar_mase_prepreke(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
std::tuple<float, float> brzine_tockova(RVO::Vector3 zeljena_brzina);

int main(int argc, char **argv)
{
	string serverIP = "127.0.0.1";
	int serverPort = 19999;
	int leftMotorHandle = 0;
	float vLeft = 0;
	int rightMotorHandle = 0;
	float vRight = 0;
	string sensorNome[16];
	int sensorHandle[16];

	int kinectSensorHandle;

	float* depthBuffer;//float** depthBuffer  // = float[64][48];// [3072];
	
	int resolution[] = { 64,48 };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ciljevi;
	std::vector<Eigen::Vector3f> centri_mase_ciljeva; //ubaciti prvo sopstvenu lokaciju
	RVO::RVOSimulator* sim = new RVO::RVOSimulator();

	float xAngle;
	float yAngle;
	float camXHalfAngle = 57;
	float camYHalfAngle = 57;
	float depthValue;
	float xCoord;
	float yCoord;
	float zCoord;
	float nearClippingPlane = 1;
	float depthAmplitude = 1;

	
	float v0 = 0;


	int clientID = simxStart((simxChar*)serverIP.c_str(), serverPort, true, true, 2000, 5);

	if (clientID != -1)
	{
		cout << "Server connected!" << std::endl;

		if (simxGetObjectHandle(clientID, (const simxChar*) "kinect_visionSensor", (simxInt *)&kinectSensorHandle, (simxInt)simx_opmode_oneshot_wait) == simx_return_ok)
		{
			//depthBuffer = simxGetVisionSensorDepthBuffer(kinectSensorHandle);  //dodati simReleaseBuffer 
			// dodati simHandleVisionSensor 
			int res = simxGetVisionSensorDepthBuffer(clientID, kinectSensorHandle, resolution, &depthBuffer, simx_opmode_streaming); //simx_return_ok

			if (simxGetVisionSensorDepthBuffer(clientID, kinectSensorHandle, resolution, &depthBuffer, simx_opmode_buffer) == simx_error_noerror)
			  {			
				for (int i = 0; i < resolution[0]; i++) {   
					xAngle = ((32 - i - 0.5) / 32)*camXHalfAngle;
					for (int j = 0; j < resolution[0]; j++) {
						yAngle = ((j - 24 + 0.5) / 24)*camYHalfAngle;
						depthValue = depthBuffer[i*resolution[0] + j];
						zCoord = nearClippingPlane + depthAmplitude*depthValue;
						xCoord = tan(xAngle)*zCoord;
						yCoord = tan(yAngle)*zCoord;
						cloud->points.emplace_back(xCoord, yCoord, zCoord);
					}
				  }
		      }
		}

		
		ciljevi = detekcija_ciljeva(cloud);

		std::vector<std::future<Eigen::Vector3f>> results;

		// poseban thread za centar_mase_prepreke(elem), packaged task
		for (const auto& elem : ciljevi) {
			results.push_back(std::async(&centar_mase_prepreke, elem));
		}


		// Boost::wait_for_any() get future result as they finish  // doraditi 
		std::vector<std::future<Eigen::Vector3f>>::iterator iter;
		for (iter = results.begin(); iter != results.end(); ++iter) {
			centri_mase_ciljeva.push_back((*iter).get());
		}


		if (simxGetObjectHandle(clientID, (const simxChar*) "Pioneer_p3dx_leftMotor", (simxInt *)&leftMotorHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
			cout << "left engine handle not found!" << std::endl;
		else
			cout << "Connected to left motor!" << std::endl;

		if (simxGetObjectHandle(clientID, (const simxChar*) "Pioneer_p3dx_rightMotor", (simxInt *)&rightMotorHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
			cout << "Right hand motor not found!" << std::endl;
		else
			cout << "Connected to right motor!" << std::endl;

		while (simxGetConnectionId(clientID) != -1) 
		{

			vLeft = v0;
			vRight = v0;

			//sim->setTimeStep(0.25f); 

			for(const auto& cilj : centri_mase_ciljeva){
				sim->addAgent(RVO::Vector3(cilj[0], cilj[1], cilj[2]));
			}

			RVO::Vector3 zeljena_brz = sim->getAgentPrefVelocity(0);
			//sim->doStep();

			std::tuple<float, float> vright_vleft = brzine_tockova(zeljena_brz);


			simxSetJointTargetVelocity(clientID, leftMotorHandle, (simxFloat)vLeft, simx_opmode_streaming);
			simxSetJointTargetVelocity(clientID, rightMotorHandle, (simxFloat)vRight, simx_opmode_streaming);

			extApi_sleepMs(5);
		}

		simxFinish(clientID); 
		cout << "Closed connection!" << std::endl;
	}
	else
		cout << "Problems connecting or server!" << std::endl;
	return 0;
}
