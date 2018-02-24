#pragma once
#include <iostream>
#include <tuple>
#include <Eigen/Dense>
#include "Vector3.h"
#include <cmath>

std::tuple<float, float> brzine_tockova(RVO::Vector3 zeljena_brzina) {

	//L R dimenzije robota, vr i vl brzine tockova robota
	float L = 50.0, R = 5.0, vr, vl, v = 0.0, omega = 0.0;
	//Matrix<float, 2, 2> m;
	float fi = std::atan(zeljena_brzina[1] / zeljena_brzina[0]);
	std::cout << "fi = " << fi << std::endl;
	//rotaciona matrica
	Eigen::Matrix2f R_fi;
	R_fi(0, 0) = std::cos(fi);
	R_fi(0, 1) = -std::sin(fi);
	R_fi(1, 0) = std::sin(fi);
	R_fi(1, 1) = std::cos(fi);

	// matrica [1 0 0 1/l] l=0.2 naprimer ??? 0.2 je mnogo malo
	Eigen::Matrix2f M_l;
	/*M_l(0, 0) = 1;
	M_l(0, 1) = 0;
	M_l(1, 0) = 0;
	M_l(1, 1) = 5;*/

	M_l << 1, 1,
		0, 1000000;

	Eigen::Vector2f v_omega;
	v_omega(0) = v;
	v_omega(1) = omega;

	Eigen::Vector2f zeljena_brzina_komponente;
	zeljena_brzina_komponente(0) = zeljena_brzina[0];
	zeljena_brzina_komponente(1) = zeljena_brzina[1];

	v_omega = M_l * R_fi.inverse() * zeljena_brzina_komponente;

	std::cout << "v = " << v_omega(0) << std::endl;
	std::cout << "omega = " << v_omega(1) << std::endl;

	vr = (2 * v_omega(0) + v_omega(1)*L) / (2 * R);
	vl = (2 * v_omega(0) - v_omega(1)*L) / (2 * R);

	return std::make_tuple(vr, vl);
}