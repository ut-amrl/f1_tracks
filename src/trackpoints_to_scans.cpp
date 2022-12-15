#include "vector_map/vector_map.h"

#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <opencv4/opencv2/core.hpp>
#include <vector>

using Eigen::Vector2f;
using std::vector;
using vector_map::VectorMap;

const float FACTOR = 20;

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
        printf("Couldn't open track file: %s\n", filename.c_str());
        exit(1);
    }

    std::string line;
    while (std::getline(track_file, line)) {
        if (line[0] == '#')
            continue;
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

std::pair<vector<Vector2f>, vector<Vector2f>>
wall_points(const vector<Vector2f>& centerline) {
    vector<Vector2f> left, right;

    for (size_t i = 0; i < centerline.size(); i++) {
        std::pair<Vector2f, Vector2f> lr;
        if (i == 0) {
            lr = lr_points(centerline.back(), centerline[i], centerline[i + 1]);
        } else if (i == centerline.size() - 1) {
            lr =
                lr_points(centerline[i - 1], centerline[i], centerline.front());
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

int main() {
    vector<Vector2f> centerline =
        read_track_file("/home/rmeno12/lidar_prediction/data/"
                        "racetrack-database/tracks/Austin.csv");
    scale(centerline);
    auto walls = wall_points(centerline);
    auto left = walls.first;
    auto right = walls.second;
    VectorMap map(make_lines(left, right));

    vector<float> scans;
    float angle_min = -2.25147461891;
    float angle_max = 2.25147461891;
    float range_min = 0.019999999553;
    float range_max = 30.0;
    map.GetPredictedScan(centerline[0], range_min, range_max, angle_min,
                         angle_max, 1080, &scans);

    printf("[");
    for (float f : scans) {
        printf("%f,", f);
    }
    printf("]");

    return 0;
}