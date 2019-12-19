#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

#include <KinematicsLib.h>

#include <LIN.h>

const int N_JOINTS = 7;
const int DOF = 3;
const double dt = 0.005; // s
double q_init[N_JOINTS] = { 0., 0., 0., 0., 0., 0., 0. };

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

	for (int i = 0; i < 7; i++) {
		printf("%f\n", x[i]);
	}
	system("pause");

	// move by A in mm
	double A[7] = { 0., 0., -100., 0., 0., 0., 0. };
	double B[7] = { 0., 0., 0., 0., 0., 0., 0. };
	for (int i = 0; i < DOF; i++) {
		B[i] = x[i] + A[i];
		A[i] = x[i];
	}
	for (int i = DOF; i < 7; i++) {
		A[i] = x[i];
		B[i] = x[i];
	}
	for (int i = 0; i < 7; i++) {
		printf("A: %f, B: %f\n", A[i], B[i]);
	}
	system("pause");


	// perform a linear motion from A to B
	LIN lin(A, B, q_init, N_JOINTS);
	auto trajectory = lin.compute();

	// save results as .csv file
	printf("length %d\n", trajectory.size());

	std::ofstream out;
	out.open("trajectory.csv");
	for (const auto& row : trajectory) {
		for (const auto& col : row) {
			out << col << ", ";
		}
		out << "\n";
	}
	out.close();
	system("pause");

	return 0;
};
