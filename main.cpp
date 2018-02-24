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
	//float depthBuffer_a[64][48];//[64][48];
	//float(**depthBuffer) = &depthBuffer_a;
	//int* resolution; // [64 x 48] ????
	int resolution[] = { 64,48 };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ciljevi;
	std::vector<Eigen::Vector3f> centri_mase_ciljeva; //ubaciti prvo sopstvenu lokaciju ???
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

	// variaveis de cena e movimentação do pioneer
	/*float noDetectionDist = 0.5;
	float maxDetectionDist = 0.2;
	float detect[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	float braitenbergL[16] = { -0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	float braitenbergR[16] = { -1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	*/
	float v0 = 0;


	// pcl::PointCloud<pcl::PointXYZ> cloud;//radi
	// cloud.points.size();  //radi

	int clientID = simxStart((simxChar*)serverIP.c_str(), serverPort, true, true, 2000, 5);

	if (clientID != -1)
	{
		cout << "Server connected!" << std::endl;

		if (simxGetObjectHandle(clientID, (const simxChar*) "kinect_visionSensor", (simxInt *)&kinectSensorHandle, (simxInt)simx_opmode_oneshot_wait) == simx_return_ok)
		{
			//depthBuffer = simxGetVisionSensorDepthBuffer(kinectSensorHandle);  //dodati simReleaseBuffer ???

			// dodati simHandleVisionSensor ???
			int res = simxGetVisionSensorDepthBuffer(clientID, kinectSensorHandle, resolution, &depthBuffer, simx_opmode_streaming); //simx_return_ok

			if (simxGetVisionSensorDepthBuffer(clientID, kinectSensorHandle, resolution, &depthBuffer, simx_opmode_buffer) == simx_error_noerror)
			  {			
				for (int i = 0; i < resolution[0]; i++) {    // da li i ide od 0 do 63 ili od 1 do 64???
					xAngle = ((32 - i - 0.5) / 32)*camXHalfAngle;
					for (int j = 0; j < resolution[0]; j++) {
						yAngle = ((j - 24 + 0.5) / 24)*camYHalfAngle;
						//depthValue = depthBuffer[i][j];// +(j - 1) * 64] //[i + (j - 1) * 63]?????? //depthBuffer[i + (j - 1) * 64]
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

		// poseban thread za centar_mase_prepreke(elem)??? mozda packaged task
		for (const auto& elem : ciljevi) {
			results.push_back(std::async(&centar_mase_prepreke, elem));
		}
		
		//for (const auto& elem : results) {
		//	 // Boost::wait_for_any() get future result as they finish
		//	//centri_mase_ciljeva.push_back(elem.get()); // kompajler prijavljuje neku gresku ???
		//}
		

		// Boost::wait_for_any() get future result as they finish  // doraditi !!!
		std::vector<std::future<Eigen::Vector3f>>::iterator iter;
		for (iter = results.begin(); iter != results.end(); ++iter) {
			centri_mase_ciljeva.push_back((*iter).get());
		}


		// inicialização dos motores
		if (simxGetObjectHandle(clientID, (const simxChar*) "Pioneer_p3dx_leftMotor", (simxInt *)&leftMotorHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
			cout << "Handle do motor esquerdo nao encontrado!" << std::endl;
		else
			cout << "Conectado ao motor esquerdo!" << std::endl;

		if (simxGetObjectHandle(clientID, (const simxChar*) "Pioneer_p3dx_rightMotor", (simxInt *)&rightMotorHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
			cout << "Handle do motor direito nao encontrado!" << std::endl;
		else
			cout << "Conectado ao motor direito!" << std::endl;

		//// inicialização dos sensores (remoteApi)
		//for (int i = 0; i < 16; i++)
		//{
		//	sensorNome[i] = "Pioneer_p3dx_ultrasonicSensor" + to_string(i + 1);

		//	if (simxGetObjectHandle(clientID, (const simxChar*)sensorNome[i].c_str(), (simxInt *)&sensorHandle[i], (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
		//		cout << "Handle do sensor " << sensorNome[i] << " nao encontrado!" << std::endl;
		//	else
		//	{
		//		cout << "Conectado ao sensor " << sensorNome[i] << std::endl;
		//		simxReadProximitySensor(clientID, sensorHandle[i], NULL, NULL, NULL, NULL, simx_opmode_streaming);
		//	}
		//}

		// desvio e velocidade do robô
		while (simxGetConnectionId(clientID) != -1) // enquanto a simulação estiver ativa
		{
			/*for (int i = 0; i < 16; i++)
			{
				simxUChar state;
				simxFloat coord[3];

				if (simxReadProximitySensor(clientID, sensorHandle[i], &state, coord, NULL, NULL, simx_opmode_buffer) == simx_return_ok)
				{
					float dist = coord[2];
					if (state > 0 && (dist < noDetectionDist))
					{
						if (dist < maxDetectionDist)
						{
							dist = maxDetectionDist;
						}

						detect[i] = 1 - ((dist - maxDetectionDist) / (noDetectionDist - maxDetectionDist));
					}
					else
						detect[i] = 0;
				}
				else
					detect[i] = 0;
			}*/


			vLeft = v0;
			vRight = v0;

			//sim->setTimeStep(0.25f); ??? da li je potrebno

			for(const auto& cilj : centri_mase_ciljeva){
				sim->addAgent(RVO::Vector3(cilj[0], cilj[1], cilj[2]));
			}

			RVO::Vector3 zeljena_brz = sim->getAgentPrefVelocity(0);
			//sim->doStep(); ??? da li je potrebno

			std::tuple<float, float> vright_vleft = brzine_tockova(zeljena_brz);


			/*for (int i = 0; i < 16; i++)
			{
				vLeft = vLeft + braitenbergL[i] * detect[i];
				vRight = vRight + braitenbergR[i] * detect[i];
			}*/

			// atualiza velocidades dos motores

			//simxSetJointTargetVelocity(clientID, leftMotorHandle, (simxFloat)std::get<0>(vright_vleft), simx_opmode_streaming);
			//simxSetJointTargetVelocity(clientID, rightMotorHandle, (simxFloat)std::get<1>(vright_vleft), simx_opmode_streaming);
			simxSetJointTargetVelocity(clientID, leftMotorHandle, (simxFloat)vLeft, simx_opmode_streaming);
			simxSetJointTargetVelocity(clientID, rightMotorHandle, (simxFloat)vRight, simx_opmode_streaming);

			// espera um pouco antes de reiniciar a leitura dos sensores
			extApi_sleepMs(5);
		}

		simxFinish(clientID); // fechando conexao com o servidor
		cout << "Conexao fechada!" << std::endl;
	}
	else
		cout << "Problemas para conectar o servidor!" << std::endl;
	return 0;
}
