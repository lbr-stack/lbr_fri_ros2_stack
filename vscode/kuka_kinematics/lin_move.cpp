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
	double A[7] = { 0., 0., 0., 0., 0., 0., 0. };
	int status;
	int turn;
	if (!getForward(q_init, A, &status, &turn, NULL, NULL)) {
		printf("forward kinematics did not converge\n");
		std::exit(EXIT_FAILURE);
	}

	for (int i = 0; i < 7; i++) {
		printf("%f\n", A[i]);
	}
	system("pause");

	// move from A to B by x and ensure same orientation
	double B[7] = { 0., 0., 0., 0., 0., 0., 0. };
	double x[7] = { 0., 0., -100., 0., 0., 0., 0. };
	for (int i = 0; i < 7; i++) {
		B[i] = A[i] + x[i];
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
			out << row[0] << ", "
				<< row[1] << ", "
				<< row[2] << ", "
				<< row[3] << ", "
				<< row[4] << ", "
				<< row[5] << ", "
				<< row[6] << "\n";
	}
	out.close();
	system("pause");

	return 0;
};
