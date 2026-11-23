#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

struct Options {
  std::string input_pcd;
  std::string output_pcd;
  std::string transform_yaml;
  double distance_threshold = 0.08;
  double low_z_percentile = 45.0;
  double voxel_size = 0.10;
  double max_tilt_deg = 30.0;
  int max_iterations = 5000;
  bool binary_output = false;
  bool zero_ground = false;
};

void printUsage(const char * program)
{
  std::cerr
    << "Usage:\n"
    << "  " << program << " <input.pcd> <output.pcd> [options]\n\n"
    << "Options:\n"
    << "  --transform-yaml <path>       Save transform metadata YAML. Default: <output>.yaml\n"
    << "  --distance-threshold <meters> RANSAC inlier threshold. Default: 0.08\n"
    << "  --low-z-percentile <0..100>   Estimate ground only from low-Z points. Default: 45\n"
    << "  --voxel-size <meters>         Downsample before plane fitting, 0 disables. Default: 0.10\n"
    << "  --max-tilt-deg <degrees>      Reject planes farther from Z than this. Default: 30\n"
    << "  --max-iterations <count>      RANSAC iterations. Default: 5000\n"
    << "  --binary                      Save output PCD in binary format. Default: ascii\n"
    << "  --zero-ground                 Also translate fitted ground plane to z=0\n";
}

double parseDouble(const std::string & value, const std::string & name)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid value for " + name + ": " + value);
  }
}

int parseInt(const std::string & value, const std::string & name)
{
  try {
    return std::stoi(value);
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid value for " + name + ": " + value);
  }
}

Options parseArgs(int argc, char ** argv)
{
  if (argc < 3) {
    printUsage(argv[0]);
    throw std::runtime_error("Missing input/output PCD path");
  }

  Options options;
  options.input_pcd = argv[1];
  options.output_pcd = argv[2];

  for (int i = 3; i < argc; ++i) {
    const std::string arg = argv[i];
    auto requireValue = [&](const std::string & name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value after " + name);
      }
      return argv[++i];
    };

    if (arg == "--transform-yaml") {
      options.transform_yaml = requireValue(arg);
    } else if (arg == "--distance-threshold") {
      options.distance_threshold = parseDouble(requireValue(arg), arg);
    } else if (arg == "--low-z-percentile") {
      options.low_z_percentile = parseDouble(requireValue(arg), arg);
    } else if (arg == "--voxel-size") {
      options.voxel_size = parseDouble(requireValue(arg), arg);
    } else if (arg == "--max-tilt-deg") {
      options.max_tilt_deg = parseDouble(requireValue(arg), arg);
    } else if (arg == "--max-iterations") {
      options.max_iterations = parseInt(requireValue(arg), arg);
    } else if (arg == "--binary") {
      options.binary_output = true;
    } else if (arg == "--zero-ground") {
      options.zero_ground = true;
    } else if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown option: " + arg);
    }
  }

  if (options.transform_yaml.empty()) {
    options.transform_yaml = options.output_pcd + ".yaml";
  }
  if (options.low_z_percentile < 0.0 || options.low_z_percentile > 100.0) {
    throw std::runtime_error("--low-z-percentile must be within [0, 100]");
  }
  if (options.distance_threshold <= 0.0) {
    throw std::runtime_error("--distance-threshold must be positive");
  }
  if (options.max_iterations <= 0) {
    throw std::runtime_error("--max-iterations must be positive");
  }

  return options;
}

double percentile(std::vector<float> values, const double pct)
{
  if (values.empty()) {
    throw std::runtime_error("Cannot compute percentile of empty cloud");
  }
  const double rank = pct / 100.0 * static_cast<double>(values.size() - 1);
  const auto low_idx = static_cast<std::size_t>(std::floor(rank));
  const auto high_idx = static_cast<std::size_t>(std::ceil(rank));
  std::nth_element(values.begin(), values.begin() + low_idx, values.end());
  const double low = values[low_idx];
  if (high_idx == low_idx) {
    return low;
  }
  std::nth_element(values.begin(), values.begin() + high_idx, values.end());
  const double high = values[high_idx];
  return low + (high - low) * (rank - static_cast<double>(low_idx));
}

CloudT::Ptr makeGroundCandidateCloud(const CloudT::ConstPtr & input, const Options & options)
{
  std::vector<float> z_values;
  z_values.reserve(input->size());
  for (const auto & point : input->points) {
    if (pcl::isFinite(point)) {
      z_values.push_back(point.z);
    }
  }

  const double z_limit = percentile(z_values, options.low_z_percentile);
  CloudT::Ptr candidates(new CloudT);
  candidates->reserve(input->size());
  for (const auto & point : input->points) {
    if (pcl::isFinite(point) && point.z <= z_limit) {
      candidates->push_back(point);
    }
  }

  if (options.voxel_size > 0.0) {
    CloudT::Ptr downsampled(new CloudT);
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(candidates);
    voxel_grid.setLeafSize(
      static_cast<float>(options.voxel_size),
      static_cast<float>(options.voxel_size),
      static_cast<float>(options.voxel_size));
    voxel_grid.filter(*downsampled);
    return downsampled;
  }

  return candidates;
}

Eigen::Vector4d estimateGroundPlane(const CloudT::ConstPtr & candidates, const Options & options)
{
  if (candidates->size() < 3) {
    throw std::runtime_error("Not enough candidate points for ground plane fitting");
  }

  pcl::SACSegmentation<PointT> segmentation;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setAxis(Eigen::Vector3f::UnitZ());
  segmentation.setEpsAngle(static_cast<float>(options.max_tilt_deg * M_PI / 180.0));
  segmentation.setDistanceThreshold(options.distance_threshold);
  segmentation.setMaxIterations(options.max_iterations);
  segmentation.setInputCloud(candidates);
  segmentation.segment(*inliers, *coefficients);

  if (inliers->indices.empty() || coefficients->values.size() < 4) {
    throw std::runtime_error("Ground plane fitting failed; try increasing --low-z-percentile or --max-tilt-deg");
  }

  Eigen::Vector4d plane(
    coefficients->values[0], coefficients->values[1], coefficients->values[2],
    coefficients->values[3]);
  const double norm = plane.head<3>().norm();
  if (norm < std::numeric_limits<double>::epsilon()) {
    throw std::runtime_error("Ground plane normal has zero length");
  }
  plane /= norm;
  if (plane.z() < 0.0) {
    plane *= -1.0;
  }

  std::cout << "Ground candidate points: " << candidates->size() << "\n";
  std::cout << "RANSAC inliers: " << inliers->indices.size() << "\n";
  return plane;
}

void writeTransformYaml(
  const Options & options, const Eigen::Vector4d & plane, const Eigen::Matrix4d & transform,
  const double angle_deg)
{
  std::ofstream out(options.transform_yaml);
  if (!out) {
    throw std::runtime_error("Cannot write transform YAML: " + options.transform_yaml);
  }

  out << std::setprecision(12);
  out << "input_pcd: " << options.input_pcd << "\n";
  out << "output_pcd: " << options.output_pcd << "\n";
  out << "method: low_z_ransac_perpendicular_plane\n";
  out << "ransac_distance_threshold_m: " << options.distance_threshold << "\n";
  out << "low_z_percentile: " << options.low_z_percentile << "\n";
  out << "voxel_size_m: " << options.voxel_size << "\n";
  out << "max_tilt_deg: " << options.max_tilt_deg << "\n";
  out << "zero_ground: " << (options.zero_ground ? "true" : "false") << "\n";
  out << "ground_plane_original:\n";
  out << "  normal: [" << plane.x() << ", " << plane.y() << ", " << plane.z() << "]\n";
  out << "  d: " << plane.w() << "\n";
  out << "transform_original_to_levelled:\n";
  out << "  angle_deg: " << angle_deg << "\n";
  out << "  matrix:\n";
  for (int r = 0; r < 4; ++r) {
    out << "    - [" << transform(r, 0) << ", " << transform(r, 1) << ", "
        << transform(r, 2) << ", " << transform(r, 3) << "]\n";
  }
}

int main(int argc, char ** argv)
{
  try {
    const Options options = parseArgs(argc, argv);

    CloudT::Ptr cloud(new CloudT);
    if (pcl::io::loadPCDFile<PointT>(options.input_pcd, *cloud) < 0) {
      throw std::runtime_error("Cannot read input PCD: " + options.input_pcd);
    }
    if (cloud->empty()) {
      throw std::runtime_error("Input PCD is empty: " + options.input_pcd);
    }

    std::cout << "Input points: " << cloud->size() << "\n";

    const CloudT::Ptr candidates = makeGroundCandidateCloud(cloud, options);
    const Eigen::Vector4d plane = estimateGroundPlane(candidates, options);
    const Eigen::Vector3d normal = plane.head<3>().normalized();
    const Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
    const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(normal, z_axis);

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation.normalized().toRotationMatrix();
    if (options.zero_ground) {
      transform(2, 3) = plane.w();
    }

    CloudT levelled;
    pcl::transformPointCloud(*cloud, levelled, transform.cast<float>());

    const auto output_parent = std::filesystem::path(options.output_pcd).parent_path();
    if (!output_parent.empty()) {
      std::filesystem::create_directories(output_parent);
    }
    const int save_result = options.binary_output
      ? pcl::io::savePCDFileBinary(options.output_pcd, levelled)
      : pcl::io::savePCDFileASCII(options.output_pcd, levelled);
    if (save_result < 0) {
      throw std::runtime_error("Cannot write output PCD: " + options.output_pcd);
    }

    const double angle_deg = std::acos(std::clamp(normal.dot(z_axis), -1.0, 1.0)) * 180.0 / M_PI;
    writeTransformYaml(options, plane, transform, angle_deg);

    const Eigen::Vector3d rotated_normal = transform.block<3, 3>(0, 0) * normal;
    std::cout << std::setprecision(8);
    std::cout << "Ground normal: [" << normal.x() << ", " << normal.y() << ", " << normal.z()
              << "]\n";
    std::cout << "Tilt corrected deg: " << angle_deg << "\n";
    std::cout << "Rotated normal check: [" << rotated_normal.x() << ", " << rotated_normal.y()
              << ", " << rotated_normal.z() << "]\n";
    std::cout << "Wrote: " << options.output_pcd << "\n";
    std::cout << "Wrote: " << options.transform_yaml << "\n";
  } catch (const std::exception & error) {
    std::cerr << "level_pcd_map error: " << error.what() << "\n";
    return 1;
  }

  return 0;
}
