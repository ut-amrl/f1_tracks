#include <gflags/gflags.h>
#include <glog/logging.h>

#include <boost/filesystem.hpp>
#include <boost/range.hpp>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <vector>

#include "vector_map/vector_map.h"

DEFINE_string(racetracks_path, "racetrack-database/tracks",
              "path to the database of racetrack csv files");
DEFINE_string(scans_path, "scans", "path to output scan data");
DEFINE_bool(export_vectormaps, true, "whether to export vectormaps");
DEFINE_string(vectormap_path, "vectormaps",
              "path to output vectormaps of racetracks");

using Eigen::Vector2f;
using Eigen::Vector3f;
using std::vector;
using vector_map::VectorMap;

const float FACTOR = 20;
const float ANGLE_MIN = -2.25147461891;
const float ANGLE_MAX = 2.25147461891;
const float RANGE_MIN = 0.019999999553;
const float RANGE_MAX = 30.0;
const int NUM_SCANS = 1080;

vector<float> split(const std::string& line) {
  vector<float> values;
  std::string value;
  std::istringstream v_stream(line);
  while (std::getline(v_stream, value, ',')) {
    values.push_back(std::stof(value));
  }

  return values;
}

vector<Vector2f> read_track_file(const std::string& filename) {
  vector<Vector2f> centerline;
  std::ifstream track_file(filename);
  if (!track_file.is_open()) {
    LOG(FATAL) << "Couldn't open track file: " << filename;
    exit(1);
  }

  std::string line;
  while (std::getline(track_file, line)) {
    if (line[0] == '#') continue;
    vector<float> values = split(line);
    centerline.push_back({values[0], values[1]});
  }

  return centerline;
}

std::pair<Vector2f, Vector2f> lr_points(const Vector2f& p, const Vector2f& c,
                                        const Vector2f& n) {
  static const Eigen::Matrix2f rot =
      (Eigen::Matrix2f() << 0, 1, -1, 0).finished();
  auto v = (n - p) / (n - p).norm();
  auto perp = rot * v;

  return {c - perp, c + perp};
}

std::pair<vector<Vector2f>, vector<Vector2f>> wall_points(
    const vector<Vector2f>& centerline) {
  vector<Vector2f> left, right;

  for (size_t i = 0; i < centerline.size(); i++) {
    std::pair<Vector2f, Vector2f> lr;
    if (i == 0) {
      lr = lr_points(centerline.back(), centerline[i], centerline[i + 1]);
    } else if (i == centerline.size() - 1) {
      lr = lr_points(centerline[i - 1], centerline[i], centerline.front());
    } else {
      lr = lr_points(centerline[i - 1], centerline[i], centerline[i + 1]);
    }
    left.push_back(lr.first);
    right.push_back(lr.second);
  }

  return {left, right};
}

void scale(vector<Vector2f>& points) {
  for (auto& p : points) {
    p /= FACTOR;
  }
}

vector<geometry::Line2f> make_lines(const vector<Vector2f>& left,
                                    const vector<Vector2f>& right) {
  vector<geometry::Line2f> lines;

  for (size_t i = 0; i < left.size() - 1; i++) {
    lines.push_back(geometry::Line2f(left[i], left[i + 1]));
  }
  lines.push_back(geometry::Line2f(left.back(), left.front()));

  for (size_t i = 0; i < right.size() - 1; i++) {
    lines.push_back(geometry::Line2f(right[i], right[i + 1]));
  }
  lines.push_back(geometry::Line2f(right.back(), right.front()));

  return lines;
}

void export_vectormap(const VectorMap& vmap, const std::string& filename) {
  std::ofstream file;
  file.open(filename);

  file << "[\n";
  for (size_t i = 0; i < vmap.lines.size(); i++) {
    auto l = vmap.lines[i];
    file << "{\"p0\":{\"x\": " << l.p0.x() << ", \"y\": " << l.p0.y()
         << "}, \"p1\":{\"x\": " << l.p1.x() << ", \"y\": " << l.p1.y() << "}}";
    if (i < vmap.lines.size() - 1) {
      file << ",\n";
    }
  }
  file << "]";
  file.close();
}

void export_scans(const vector<std::pair<Vector3f, vector<float>>>& scans,
                  const std::string& filename) {
  std::ofstream file;
  file.open(filename);

  // add first line as column names
  file << "x, y, theta, ";
  for (int i = 0; i < NUM_SCANS; i++) {
    file << "scan" << std::setfill('0') << std::setw(4) << i;
    if (i != NUM_SCANS - 1) file << ", ";
  }
  file << "\n";

  for (auto p : scans) {
    Vector3f loc = p.first;
    vector<float> scan = p.second;
    file << loc.x() << ", " << loc.y() << ", " << loc.z() << ", ";
    for (int i = 0; i < NUM_SCANS; i++) {
      file << scan[i];
      if (i != NUM_SCANS - 1) file << ", ";
    }
    file << "\n";
  }

  file.close();
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  // for each track file:
  //   - read/process string
  //   - create walls -> create vectormap object
  //   - for each point on the centerline, predict scans
  //   - store location + predicted scans as element of data structure
  // export all of this to a csv (one csv per track?)

  // every iteration of this loop is independent -> very parallelizable
  for (auto& track_file_entry : boost::make_iterator_range(
           boost::filesystem::directory_iterator(FLAGS_racetracks_path), {})) {
    auto track_path = track_file_entry.path();
    LOG(INFO) << "Processing " << track_path.filename();

    vector<Vector2f> centerline = read_track_file(track_path.string());
    scale(centerline);  // combine this with ^

    auto walls = wall_points(centerline);
    auto left = walls.first;
    auto right = walls.second;
    VectorMap map(make_lines(left, right));

    if (FLAGS_export_vectormaps) {
      std::string vmap_filename = FLAGS_vectormap_path + "/" +
                                  track_path.stem().string() +
                                  ".vectormap.json";
      LOG(INFO) << "Saving " << vmap_filename;
      export_vectormap(map, vmap_filename);
    }

    vector<std::pair<Vector3f, vector<float>>> track_scans;
    for (size_t i = 0; i < centerline.size(); i++) {
      vector<float> scan;
      float ang_d;
      if (i == 0) {
        ang_d = atan2(centerline[i + 1].y() - centerline.back().y(),
                      centerline[i + 1].x() - centerline.back().x());
      } else if (i == centerline.size() - 1) {
        ang_d = atan2(centerline.front().y() - centerline[i - 1].y(),
                      centerline.front().x() - centerline[i - 1].x());
      } else {
        ang_d = atan2(centerline[i + 1].y() - centerline[i - 1].y(),
                      centerline[i + 1].x() - centerline[i - 1].x());
      }
      map.GetPredictedScan(centerline[i], RANGE_MIN, RANGE_MAX,
                           ANGLE_MIN + ang_d, ANGLE_MAX + ang_d, NUM_SCANS,
                           &scan);
      track_scans.push_back(
          {{centerline[i].x(), centerline[i].y(), ang_d}, scan});
    }
    std::string scans_filename =
        FLAGS_scans_path + "/" + track_path.stem().string() + ".scans.csv";
    LOG(INFO) << "Saving " << scans_filename;
    export_scans(track_scans, scans_filename);
  }

  return 0;
}
