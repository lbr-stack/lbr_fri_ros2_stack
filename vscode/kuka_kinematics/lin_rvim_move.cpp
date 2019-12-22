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
double scaling = 1;

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


point_cloud merry_christmas = { // in x-y-z plane
	// M
	{-  0.0,  0.0, 0.0},   //  1
	{- 31.0,  0.0, 0.0},  //  2
	{- 11.5, 16.5, 0.0}, //  3
	{- 31.0, 33.0, 0.0}, //  4
	{   0.0, 33.0, 0.0},  //  5

	// transition
	{- 0.0, 33.0, 5.0},
	{- 8.0, 39.5, 5.0},

	// e
	{-  8.0, 39.5, 0.0},  //  6
	{-  8.0, 52.5, 0.0},  //  7
	{- 19.0, 52.5, 0.0}, //  8
	{- 19.0, 39.5, 0.0}, //  9
	{   0.0, 39.5, 0.0},  // 10
	{   0.0, 52.5, 0.0},  // 11

	// transition
	{0.0, 52.5, 5.0},
	{0.0, 59.0, 5.0},

	// r
	{-  0.0, 59.0, 0.0},   // 12
	{- 19.0, 59.0, 0.0},  // 13
	{- 19.0, 59.0, 5.0},  // transition
	{- 11.0, 59.0, 5.0},  // transition
	{- 11.0, 59.0, 0.0},  // 14
	{- 19.0, 66.0, 0.0},  // 15
	{- 19.0, 72.0, 0.0},  // 16

	// transition
	{- 19.0, 72.0, 5.0},
	{   0.0, 79.0, 5.0},

	// r
	{   0.0, 79.0, 0.0},   // 17
	{- 19.0, 79.0, 0.0},  // 18
	{- 19.0, 79.0, 5.0},  // transition
	{- 11.0, 79.0, 5.0},  // transition
	{- 11.0, 79.0, 0.0},  // 19
	{- 19.0, 86.0, 0.0},  // 20
	{- 19.0, 92.0, 0.0},  // 21

	// transition
	{- 19.0, 92.0, 5.0}, 
	{- 19.0, 99.0, 5.0}, 

	// y
	{- 19.0,  99.0, 0.0},    // 22
	{   0.0,  99.0, 0.0},     // 23
	{   0.0, 113.0, 0.0},    // 24
	{   0.0, 113.0, 5.0},    // transition
	{- 19.0, 113.0, 5.0},   // transition
	{- 19.0, 113.0, 0.0},   // 25
	{  11.0, 113.0, 0.0}, // 26
	{  11.0,  99.0, 0.0},  // 27

	// transition
	{11.0, 99.0, 5.0},
	{23.5, 38.0, 5.0},

	// C
	{23.5, 38.0, 0.0}, // 28
	{23.5, 18.0, 0.0}, // 29
	{54.0, 18.0, 0.0}, // 30
	{54.0, 38.0, 0.0}, // 31

	// transition
	{54.0, 38.0, 5.0},
	{23.5, 44.5, 5.0},

	// h
	{23.5, 44.5, 0.0}, // 32
	{54.0, 44.5, 0.0}, // 33
	{54.0, 44.5, 5.0}, // transition 
	{35.0, 44.5, 5.0}, // transition 
	{35.0, 44.5, 0.0}, // 34 
	{35.0, 58.0, 0.0}, // 35
	{54.0, 58.0, 0.0}, // 36

	// transition
	{54.0, 58.0, 5.0},
	{54.0, 64.5, 5.0},

	// r
	{54.0, 64.5, 0.0}, // 37
	{35.0, 64.5, 0.0}, // 38
	{35.0, 64.5, 5.0},  // transition
	{43.0, 64.5, 5.0},  // transition
	{43.0, 64.5, 0.0}, // 39
	{35.0, 71.5, 0.0}, // 40
	{35.0, 77.5, 0.0}, // 41

	// transition
	{35.0, 77.5, 5.0},
	{23.5, 84.5, 5.0},

	// i
	{23.5, 84.5, 0.0},   // 42
	{25.5, 84.5, 0.0},  // 43
	{25.5, 84.5, 5.0 }, // transition
	{35.0, 84.5, 5.0 }, // transition
	{35.0, 84.5, 0.0},  // 44
	{54.0, 84.5, 0.0},  // 45

	// transition
	{54.0,  84.5, 5.0 },
	{35.0, 104.5, 5.0 },

	// s
	{35.0, 104.5, 0.0},  // 46
	{35.0,  91.5, 0.0},  // 47
	{46.5,  91.5, 0.0},  // 48
	{46.5, 104.5, 0.0}, // 49
	{54.0, 104.5, 0.0}, // 50
	{54.0,  91.5, 0.0},  // 51

	// transition
	{54.0,  91.5, 5.0 },
	{23.5, 111.0, 5.0 },

	// t
	{23.5, 111.0, 0.0},   // 52
	{54.0, 111.0, 0.0},  // 53
	{54.0, 125.0, 0.0},  // 54
	{54.0, 125.0, 5.0 }, // transition
	{35.0, 111.0, 5.0 }, // transition
	{35.0, 111.0, 0.0},  // 55
	{35.0, 118.0, 0.0},  // 56

	// transition
	{35.0, 118.0, 5.0 },
	{35.0, 131.0, 5.0 },

	// m
	{35.0, 131.0, 0.0},     // 57
	{54.0, 131.0, 0.0},    // 58
	{54.0, 131.0, 5.0 },   // transition
	{43.0, 131.0, 5.0 },   // transition
	{43.0, 131.0, 0.0},    // 59
	{35.0, 138.0, 0.0},    // 60
	{35.0, 144.0, 0.0},    // 61
	{54.0, 144.0, 0.0},    // 62
	{54.0, 144.0, 5.0 },   // transition
	{43.0, 144.0, 5.0 },   // transition
	{43.0, 144.0, 0.0},    // 63
	{35.0, 151.0, 0.0},    // 64
	{35.0, 157.0, 0.0},    // 65
	{54.0, 157.0, 0.0},    // 66

	// transition
	{54.0, 157.0, 5.0 }, 
	{35.0, 164.0, 5.0 },

	// a
	{35.0, 164.0, 0.0},  // 67
	{35.0, 177.0, 0.0}, // 68
	{54.0, 177.0, 0.0}, // 69
	{54.0, 164.0, 0.0}, // 70
	{46.5, 164.0, 0.0}, // 71
	{46.5, 177.0, 0.0}, // 72

	// transition
	{46.5, 177.0, 5.0 },
	{35.0, 197.0, 5.0 },

	// s
	{35.0, 197.0, 0.0 }, // 73
	{35.0, 184.0, 0.0},  // 74
	{46.5, 184.0, 0.0},  // 75
	{46.5, 197.0, 0.0},  // 76
	{54.0, 197.0, 0.0},  // 77
	{54.0, 184.0, 0.0},  // 78

	// lift
	{54.0, 184.0, 5.0 },  // 78
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
	for (int i = 0; i < merry_christmas.size() - 1; i++) {
		// update points in x-y plane
		A[0] = x[0] + merry_christmas[i][0]*scaling;
		A[1] = x[1] + merry_christmas[i][1]*scaling;
		A[2] = x[2] + merry_christmas[i][2]*scaling;
		B[0] = x[0] + merry_christmas[i + 1][0]*scaling;
		B[1] = x[1] + merry_christmas[i + 1][1]*scaling;
		B[2] = x[2] + merry_christmas[i + 1][2]*scaling;


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
