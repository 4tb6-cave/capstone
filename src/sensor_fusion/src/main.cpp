
#include <sstream>
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <regex>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <CLI11.hpp>

using namespace gtsam;

/**
 * Function to load matrix from csv file
 * Returns true on success
 */
bool loadMatrix4d(const std::string &filename, Eigen::Matrix4d &matrix)
{
	std::ifstream file(filename);
	if (!file.is_open())
	{
		std::cerr << "Failed to open " << filename << std::endl;
		return false;
	}

	std::string line;
	int row = 0;

	while (std::getline(file, line) && row < 4)
	{
		std::stringstream ss(line);
		std::string value;
		int col = 0;

		while (std::getline(ss, value, ',') && col < 4)
		{
			matrix(row, col) = std::stof(value);
			col++;
		}

		if (col != 4)
		{
			std::cerr << "Invalid data in " << filename << ": too many columns" << std::endl;
			return false;
		}
		row++;
	}

	if (row != 4)
	{
		std::cerr << "Invalid data in " << filename << ": too many rows" << std::endl;
		return false;
	}
	return true;
}

/**
 * Function to save matrix to csv file
 * Returns true on success
 */
bool saveMatrix4d(std::string filename, const Eigen::Matrix4d &matrix)
{
	const static Eigen::IOFormat csv_format(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
	std::ofstream file(filename);
	if (file.is_open())
	{
		file << matrix.format(csv_format);
		file.close();
		return true;
	}
	else
	{
		std::cerr << "Failed to open and save to " << filename << std::endl;
		return false;
	}
}

int main(int argc, char **argv)
{
	CLI::App app{"GTSAM sensor fusion using ICP results and IMU"};

	std::string transform_path = "transforms";
	std::string pose_path = "poses";
	int starting_frame = 0;
	int ending_frame = 0;

	app.add_option("--transform_path", transform_path, "Set directory to read ICP transformation results")
		->check(CLI::ExistingDirectory)->required();
	app.add_option("--pose_path", pose_path, "Set directory to save output (4x4 transformation matrix representing pose)")
		->default_val("poses");
	app.add_option("--start", starting_frame, "Starting frame number")
		->default_val(0);
	app.add_option("--end", ending_frame, "Ending frame number")
		->required();

	CLI11_PARSE(app, argc, argv);

	// Create output directory in case it does not already exist
	std::filesystem::create_directory(pose_path);

	// Create a factor graph container and add factors to it
	NonlinearFactorGraph graph;

	// Add a prior on the first pose, setting it to the origin
	// A prior factor consists of a mean and a noise model (covariance matrix)
	noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector6(0.01, 0.01, 0.01, 0.01, 0.01, 0.01)); // TODO what values?
	graph.addPrior(starting_frame, Pose3(Rot3::Identity(), Point3(0, 0, 0)), priorNoise);

	// TODO: We really need a covariance matrix for each ICP result! this is just a wild guess
	noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector6(0.1, 0.1, 0.1, 0.1, 0.1, 0.1));


	// TODO: it would be nice if the files were sorted, for debugging, but it really doesn't matter
	for (const auto& file : std::filesystem::directory_iterator(transform_path))
	{
		if (!file.is_regular_file())
			continue;

		std::string filename = file.path().filename().string();
		// extract frame numbers and check file name
		const std::regex pattern("icp(\\d+)_(\\d+)\\.csv");
		std::smatch match;
		if (std::regex_match(filename, match, pattern))
		{
			int node1 = std::stoi(match[1]);
			int node2 = std::stoi(match[2]);
			if (node1 < starting_frame || node1 > ending_frame || node2 < starting_frame || node2 > ending_frame)
			{
				std::cout << "Skipping " << file.path().string() << ", out of range" << std::endl;
				continue;
			}
			std::cout << "Reading from " << file.path().string() << ", " << node1 << " --> " << node2 << std::endl;

			// Load transformation matrix from file and place in graph
			Eigen::Matrix4d transform_matrix;
			if (loadMatrix4d(file.path().string(), transform_matrix))
			{
				graph.emplace_shared<BetweenFactor<Pose3>>(node1, node2, Pose3(transform_matrix), model);
			}
		}
	}

	graph.print("\nFactor Graph:\n"); // print complete graph

	// Create the data structure to hold the initialEstimate estimate to the solution
	Values initialEstimate;
	for (int i = starting_frame; i <= ending_frame; i++)
	{
		initialEstimate.insert(i, Pose3(Rot3::Identity(), Point3(0, 0, 0)));
	}
	// initialEstimate.print("\nInitial Estimate:\n"); // print

	// 4. Optimize the initial values using a Gauss-Newton nonlinear optimizer
	// The optimizer accepts an optional set of configuration parameters,
	// controlling things like convergence criteria, the type of linear
	// system solver to use, and the amount of information displayed during
	// optimization. We will set a few parameters as a demonstration.
	GaussNewtonParams parameters;
	// Stop iterating once the change in error between steps is less than this value
	parameters.relativeErrorTol = 1e-5;
	// Do not perform more than N iteration steps
	parameters.maxIterations = 100;
	// Create the optimizer ...
	GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
	// ... and optimize
	Values result = optimizer.optimize();
	result.print("Final Result:\n");

	// 5. Calculate and print marginal covariances for all variables
	// std::cout.precision(3);
	// Marginals marginals(graph, result);
	// for (int i = starting_frame; i < ending_frame; i++)
	// {
	// 	std::cout << i << " covariance:\n" << marginals.marginalCovariance(i) << std::endl;
	// }

	// Save poses to files
	for (int i = starting_frame; i <= ending_frame; i++)
	{
		Eigen::Matrix4d transform = result.at<Pose3>(i).matrix();
		std::ostringstream transform_file;
		transform_file << pose_path << "/pose" << std::setw(5) << std::setfill('0') << i << ".csv";
		saveMatrix4d(transform_file.str(), transform);
	}
	return 0;
}