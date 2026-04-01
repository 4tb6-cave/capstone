
#include <sstream>
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <regex>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseRotationPrior.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>

#include <CLI11.hpp>

using namespace gtsam;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)


// TODO:
// - figure out covariance values for IMU and for ICP
// - Add an option to use precomputed quaternions from IMU, include as PoseRotationPrior factors
// - Add translation from IMU frame to ToF frame


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

bool loadTimestamps(std::string filename, std::vector<double> &timestamps)
{
	timestamps.clear();
	std::ifstream file(filename);
	if (!file.is_open())
	{
		std::cerr << "Failed to open " << filename << std::endl;
		return false;
	}

	std::string line;
	int frame = 0;
	std::getline(file, line); // discard first line
	while (std::getline(file, line))
	{
		std::stringstream ss(line);
		std::string value;	
		if (!std::getline(ss, value, ','))
		{
			std::cerr << "Invalid data in " << filename << ": failed to read id" << std::endl;
			return false;
		}
		if (std::stoi(value) != frame)
		{
			continue; // only save if frame number makes sense
		}
		if (!std::getline(ss, value, ','))
		{
			std::cerr << "Invalid data in " << filename << ": failed to read timestamp" << std::endl;
			return false;
		}
		timestamps.push_back(std::stod(value));
		frame++;
	}
	return true;
}

boost::shared_ptr<PreintegrationParams> imuParams()
{
	// We use the sensor specs to build the noise model for the IMU factor.
	// TODO: add correct values!! or make this configurable for different sensors with an external file?
	double accel_noise_sigma = 0.1;
	double gyro_noise_sigma = 0.1;
	double accel_bias_rw_sigma = 0.001;
	double gyro_bias_rw_sigma = 0.001;
	Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
	Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
	Matrix33 integration_error_cov =
		I_3x3 * 1e-4; // error committed in integrating position from velocities
	Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
	Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);

	auto p = PreintegrationParams::MakeSharedU(9.81); // gravity vector points in -z
	// PreintegrationBase params:
	p->accelerometerCovariance =
		measured_acc_cov; // acc white noise in continuous
	p->integrationCovariance =
		integration_error_cov; // integration uncertainty continuous
	// should be using 2nd order integration
	// PreintegratedRotation params:
	p->gyroscopeCovariance =
		measured_omega_cov; // gyro white noise in continuous

	p->use2ndOrderCoriolis = false;

	return p;
}

// Add IMU to factor graph
bool imu_preintegration(NonlinearFactorGraph &graph, std::string imu_filename,
						std::vector<double> frame_time, int first_frame, int last_frame)
{
	std::ifstream file(imu_filename);
	if (!file.is_open())
	{
		std::cerr << "Failed to open " << imu_filename << std::endl;
		return false;
	}

	// add priors to graph for V and B
	auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
	auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);
	graph.addPrior(V(first_frame), Vector3(0, 0, 0), velocity_noise_model);
	graph.addPrior(B(first_frame), imuBias::ConstantBias(Vector6(0, 0, 0, 0, 0, 0)), bias_noise_model); // should the covariance for this first one be greater than for between the others?


	imuBias::ConstantBias prior_imu_bias;  // assume zero initial bias
	auto p = imuParams();
	PreintegratedImuMeasurements preintegrated(p, prior_imu_bias);

	std::string line;
	int row = 0;
	const int num_cols = 11;
	double values[num_cols]; // time,o_x,o_y,o_z,o_w,av_x,av_y,av_z,la_x,la_y,la_z

	double cur_time;
	double prev_time = -1;

	int frame = first_frame;

	// IMU and ToF sensor are probably out of sync, so for now it assumed that the ToF sensor measurement
	// happened at the exact same time as the IMU measurement immediately preceding it. Probably good enough for now.

	// TODO: what if IMU starts recording after several frames of ToF data are already saved??

	std::getline(file, line); // discard first line

	while (std::getline(file, line))
	{
		// Read IMU data
		std::stringstream ss(line);
		std::string value;
		int col = 0;
		while (std::getline(ss, value, ',') && col < num_cols)
		{
			values[col] = std::stod(value);
			col++;
		}
		Eigen::Vector3d acc, omega;
		acc << values[8], values[9], values[10];
		omega << values[5], values[6], values[7];
		cur_time = values[0];

		// Use IMU data
		if (values[0] < frame_time[frame]) // check if time is in the right range
		{
			if (frame == first_frame)
				continue; // values before first frame are discarded
		}
		else
		{
			if (frame != first_frame)
			{
				// add factor into graph
				ImuFactor imu_factor(X(frame - 1), V(frame - 1), X(frame), V(frame), B(frame - 1), preintegrated);
				graph.add(imu_factor);
			}

			frame++;
			if (frame > last_frame)
			{
				// complete
				break;
			}

			// reset integration
			preintegrated.resetIntegration();

		}

		// preintegrate IMU
		double dt = 0.01; //(prev_time != -1) ? (cur_time - prev_time) : 0.01; // default 100 Hz (TODO improve this)
		preintegrated.integrateMeasurement(acc, omega, dt);
		prev_time = cur_time;

	}


	imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
	for (int f = first_frame; f < last_frame; f++)
	{
		// add bias factors
		graph.add(BetweenFactor<imuBias::ConstantBias>(B(f), B(f + 1), zero_bias, bias_noise_model));
	}

	return true;
}

// Add IMU to factor graph using precomputed orientation
bool imu_rotation_prior(NonlinearFactorGraph &graph, std::string imu_filename,
						std::vector<double> frame_time, int first_frame, int last_frame)
{
	std::ifstream file(imu_filename);
	if (!file.is_open())
	{
		std::cerr << "Failed to open " << imu_filename << std::endl;
		return false;
	}

	auto rot_noise_model = noiseModel::Isotropic::Sigma(3, 0.01); // TODO determine value

	std::string line;
	int row = 0;
	const int num_cols = 11;
	double values[num_cols]; // time,o_x,o_y,o_z,o_w,av_x,av_y,av_z,la_x,la_y,la_z

	double cur_time, prev_time;
	Eigen::Quaterniond q, prev_q;
	bool prev_imu_flag = false;

	Eigen::Quaterniond inv_q;
	bool got_first_q = false;

	int frame = first_frame;

	std::getline(file, line); // discard first line

	while (std::getline(file, line) && frame <= last_frame)
	{
		// Read IMU data
		std::stringstream ss(line);
		std::string value;
		int col = 0;
		while (std::getline(ss, value, ',') && col < num_cols)
		{
			values[col] = std::stod(value);
			col++;
		}
		q = Eigen::Quaterniond(values[4], values[1], values[2], values[3]);
		cur_time = values[0];

		if (values[0] > frame_time[frame])
		{
			if (prev_imu_flag)
			{
				float t = (frame_time[frame] - prev_time) / (cur_time - prev_time); // interpolation factor
				prev_q.normalize();
				q.normalize(); //ensure both are normalized
				Eigen::Quaterniond q_new = prev_q.slerp(t, q);
				// std::cout << "Interpolated q for frame " << frame << " time " << std::fixed << std::setprecision(3) << 
				// 	frame_time[frame] << ": " << q_new << std::endl;

				if (!got_first_q)
				{
					inv_q = q_new.inverse();
					got_first_q = true;
				}
				q_new = q_new * inv_q;

				// add factor into graph
				PoseRotationPrior<Pose3> rot_factor(X(frame), Rot3(q_new), rot_noise_model);
				graph.add(rot_factor);

				frame++;
			}
			else
			{
				// IMU must have started after ToF, we need to skip ToF frames until they match up again
				while (values[0] > frame_time[frame])
					frame++;
				prev_imu_flag = true;
			}
		}
		else
		{
			// imu measurement before frame
			prev_imu_flag = true;
		}

		prev_q = q;
		prev_time = cur_time;
	}

	return true;
}

int main(int argc, char **argv)
{
	CLI::App app{"GTSAM sensor fusion using ICP results and IMU"};

	std::string data_dir;
	int starting_frame = 0;
	int ending_frame = 0;
	bool enable_preintegration = false;
	bool enable_rotation_prior = false;

	app.add_option("data_dir", data_dir, "Directory which contains input data and where results will be stored")
		->check(CLI::ExistingDirectory)->required();
	app.add_option("-s,--start", starting_frame, "Starting frame number")
		->default_val(0);
	app.add_option("-e,--end", ending_frame, "Ending frame number")
		->required();
	app.add_flag("--enable_imu_preint", enable_preintegration, "Enable IMU preintegration")
		->default_val(false);
	app.add_flag("--enable_imu_rot", enable_rotation_prior, "Enable IMU rotation prior from precomputed quaternions")
		->default_val(false);

	CLI11_PARSE(app, argc, argv);

	std::string transform_path = data_dir + "/transforms";
	std::string pose_path = data_dir + "/poses";
	std::string imu_filename = data_dir + "/imu.csv";
	std::string frame_timestamps_filename = data_dir + "/time_stamps.csv";

	//load timestamps
	std::vector<double> frame_timestamps;
	if (!loadTimestamps(frame_timestamps_filename, frame_timestamps) || frame_timestamps.size() < ending_frame)
	{
		std::cerr << "Failed to load timestamps from " << frame_timestamps_filename << std::endl;
	}

	// Create output directory in case it does not already exist
	std::filesystem::create_directory(pose_path);

	// Create a factor graph container and add factors to it
	NonlinearFactorGraph graph;

	// Add a prior on the first pose, setting it to the origin
	// A prior factor consists of a mean and a noise model (covariance matrix)
	if (enable_rotation_prior)
	{
		// only constrain translation, because we have a prior for rotation elsewhere
		noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001)); // TODO what values?
		graph.add(PoseTranslationPrior<Pose3>(X(starting_frame), Pose3(Rot3::Identity(), Point3(0, 0, 0)), priorNoise));
	}
	else
	{
		noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector6(0.001, 0.001, 0.001, 0.001, 0.001, 0.001)); // TODO what values?
		graph.addPrior(X(starting_frame), Pose3(Rot3::Identity(), Point3(0, 0, 0)), priorNoise);
	}

	// TODO: We really need a covariance matrix for each ICP result! this is just a wild guess
	noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector6(0.001, 0.001, 0.001, 0.001, 0.001, 0.001));


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
				graph.emplace_shared<BetweenFactor<Pose3>>(X(node1), X(node2), Pose3(transform_matrix), model);
			}
		}
	}

	// Add IMU preintegration factors
	if (enable_preintegration)
		imu_preintegration(graph, imu_filename, frame_timestamps, starting_frame, ending_frame);

	if (enable_rotation_prior)
		imu_rotation_prior(graph, imu_filename, frame_timestamps, starting_frame, ending_frame);

	graph.print("\nFactor Graph:\n"); // print complete graph

	// Create the data structure to hold the initialEstimate estimate to the solution
	Values initialEstimate;
	for (int i = starting_frame; i <= ending_frame; i++)
	{
		initialEstimate.insert(X(i), Pose3(Rot3::Identity(), Point3(0, 0, 0))); // use icp estimates?

		// for IMU
		if (enable_preintegration)
		{
			initialEstimate.insert(V(i), Vector3(0, 0, 0));
			initialEstimate.insert(B(i), imuBias::ConstantBias(Vector6(0, 0, 0, 0, 0, 0)));
		}
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
		Eigen::Matrix4d transform = result.at<Pose3>(X(i)).matrix();
		std::ostringstream transform_file;
		transform_file << pose_path << "/pose" << std::setw(5) << std::setfill('0') << i << ".csv";
		saveMatrix4d(transform_file.str(), transform);
	}
	return 0;
}