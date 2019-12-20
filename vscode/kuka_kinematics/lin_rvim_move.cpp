#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <tuple>

#include <KinematicsLib.h>

#include <LIN.h>

const int N_JOINTS = 7;
const int DOF = 3;
const double dt = 0.005; // s
double q_init[N_JOINTS] = { 0., 0., 0., 0., 0., 0., 0. };

// no singularity at 0. 60. 0. 60. 0. 60. 0.
// create point cloud for rvim_logo
using point = std::vector<double>;
using point_cloud = std::vector<point>;
double scaling = 20;

point_cloud rvim_logo = { // in x-y-z plane
	{ 0., 0., 0.}, //  1
	{-2., 0., 0.}, //  2
	{-2., 1., 0.}, //  3
	{-1., 1., 0.}, //  4
	{-1., 0., 0.}, //  5
	{ 0., 1., 0.}, //  6

	{ 0., 1., 1.}, //  transition
	{-2., 2., 1.}, //  transition

	{-2., 2., 0.}, //  7
	{ 0., 3., 0.}, //  8
	{-2., 4., 0.}, //  9

	{-2., 4., 1.}, //  transition
	{-2., 5., 1.}, //  transition

	{-2., 5., 0.}, // 10
	{ 0., 5., 0.}, // 11

	{ 0., 5., 1.}, // transition
	{ 0., 6., 1.}, // transition

	{ 0., 6., 0.}, // 12
	{-2., 6., 0.}, // 13
	{-1., 7., 0.}, // 14
	{-2., 8., 0.}, // 15
	{ 0., 8., 0.}, // 16

	{ 0., 8., 1.}, // up move
}; // in units of mm, defined in initial plane

auto main(int argc, char** argv) -> int {

	if (argc == 8) {
		for (int i = 0; i < N_JOINTS; i++) {
			q_init[i] = std::stoi(argv[i + 1]);
		}
	}

	// init LBR
	init(Device::LBR7KG);

	// compute initial cartesian position
	double x[7] = { 0., 0., 0., 0., 0., 0., 0. };
	int status;
	int turn;
	if (!getForward(q_init, x, &status, &turn, NULL, NULL)) {
		printf("forward kinematics did not converge\n");
		std::exit(EXIT_FAILURE);
	}

	printf("turn: %d, status: %d\n", turn, status);

	for (int i = 0; i < 7; i++) {
		printf("%f\n", x[i]);
	}
	system("pause");

	// move by A in mm
	double A[7] = { 0., 0., 0., 0., 0., 0., 0. };
	double B[7] = { 0., 0., 0., 0., 0., 0., 0. };
	for (int i = 0; i < 7; i++) {
		A[i] = x[i];
		B[i] = x[i];
	}
	for (int i = 0; i < 7; i++) {
		printf("A: %f, B: %f\n", A[i], B[i]);
	}
	system("pause");

	std::ofstream out;
	out.open("trajectory.csv");

	// perform a linear motion from A to B
	for (int i = 0; i < rvim_logo.size() - 1; i++) {
		// update points in x-y plane
		A[0] = x[0] + rvim_logo[i][0]*scaling;
		A[1] = x[1] + rvim_logo[i][1]*scaling;
		A[2] = x[2] + rvim_logo[i][2]*scaling;
		B[0] = x[0] + rvim_logo[i + 1][0]*scaling;
		B[1] = x[1] + rvim_logo[i + 1][1]*scaling;
		B[2] = x[2] + rvim_logo[i + 1][2]*scaling;


		LIN lin(A, B, q_init, N_JOINTS, status, turn);
		std::vector<std::vector<double>> trajectory;

		for (int i = 0; i < 7; i++) {
			printf("A: %f, B: %f\n", A[i], B[i]);
		}

		// compute trajectory
		trajectory = lin.compute();
		for (int j = 0; j < N_JOINTS; j++) {
			q_init[j] = trajectory[trajectory.size() - 1][j];
		}

		// save results as .csv file
		printf("length %d\n", trajectory.size());

		for (const auto& row : trajectory) {
			out << row[0] << ", "
				<< row[1] << ", "
				<< row[2] << ", "
				<< row[3] << ", "
				<< row[4] << ", "
				<< row[5] << ", "
				<< row[6] << "\n";
		}
	}

	out.close();

	return 0;
};
