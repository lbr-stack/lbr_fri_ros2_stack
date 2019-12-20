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
// create point cloud for santas house
using point = std::tuple<double, double>;
using point_cloud = std::vector<point>;

point_cloud santas_house = {
	std::make_tuple<double, double>(0.,  0.),// 1
	std::make_tuple<double, double>(20.,  0.),// 2
	std::make_tuple<double, double>(20., 40.),// 3
	std::make_tuple<double, double>(0.,  0.),// 4
	std::make_tuple<double, double>(0., 40.),// 5
	std::make_tuple<double, double>(20., 40.),// 6
	std::make_tuple<double, double>(10., 50.),// 7
	std::make_tuple<double, double>(0., 40.),// 8
	std::make_tuple<double, double>(20.,  0.),// 9
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
	for (int i = 0; i < santas_house.size() - 1; i++) {
		// update points in y-z plane
		A[1] = x[1] + std::get<0>(santas_house[i]);
		A[2] = x[2] + std::get<1>(santas_house[i]);
		B[1] = x[1] + std::get<0>(santas_house[i + 1]);
		B[2] = x[2] + std::get<1>(santas_house[i + 1]);


		LIN lin(A, B, q_init, N_JOINTS);
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
