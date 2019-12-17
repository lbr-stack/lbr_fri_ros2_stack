/* 
* example code that utilizes kuka's KinematicsLib
* download KinematicsLib at https://drive.google.com/file/d/1Fbw4M2O0LrA0M5-qaaoc9FbFnwLovNA7/view?usp=sharing
* find setup of KinematicsLib https://docs.google.com/document/d/1JWkPyFDort5hLL4CVQddASvoFPW9zGWo90L2t5IxZHY/edit?usp=sharing
* find kuka's documentation at https://drive.google.com/file/d/1_krvyozw_d8P1_l5XstOoVm72jHa50lZ/view?usp=sharing
*/

#include <iostream>
#include <KinematicsLib.h>
using namespace std;

auto forward() -> void {
	double axesInput[7] = { 0., 0., 0., 0., 0., 0., 0. }; // in units of rad
	double cartOutput[7] = { 0., 0., 0., 0., 0., 0., 0. }; // in units of mm
	int status;
	int turn;
	if (!getForward(axesInput, cartOutput, &status, &turn, NULL, NULL)) {
		printf("could not forward\n");
	}
	else {
		printf("forward kinematics cartesian coordinates:\n");
		printf("status: %d\n", status);
		printf("turn: %d\n", turn);
		for (auto const& i : cartOutput) {
			cout << i << endl;
		}
	}
	system("pause");
};

auto inverse() -> void {
	double cartInput[7] = { 0., 0., 0., 0., 0., 0., 0. };
	double axesOutput[7] = { 0., 0., 0., 0., 0., 0., 0. };
	int status = 0;
	int turn = 0;
	if (!getInverse(cartInput, status, turn, axesOutput, axesOutput, NULL, NULL)) {
		printf( "could not invert\n");
	}
	else {
		printf("inverse kinematics joint angles:\n");
		for (auto const& i : axesOutput) {
			cout << i << endl;
		}
	}
	system("pause");
};

auto main() -> int {
	init(Device::LBR7KG);

	forward();
	inverse();
}
