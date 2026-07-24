#include "freedom/freedom.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace {

struct Record {
    std::uint64_t frame = 0;
    std::int64_t timestampNs = 0;
    Eigen::Isometry3d worldFromLidar = Eigen::Isometry3d::Identity();
    pcl::PointXYZ point;
};

std::vector<std::string> split(const std::string& line)
{
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

Record parseRecord(const std::string& line)
{
    const std::vector<std::string> fields = split(line);
    if (fields.size() != 12) {
        throw std::runtime_error("Each replay row must contain 12 comma-separated fields.");
    }
    Record record;
    record.frame = std::stoull(fields[0]);
    record.timestampNs = std::stoll(fields[1]);
    record.worldFromLidar.translation() = Eigen::Vector3d(
        std::stod(fields[2]), std::stod(fields[3]), std::stod(fields[4]));
    Eigen::Quaterniond rotation(
        std::stod(fields[8]), std::stod(fields[5]),
        std::stod(fields[6]), std::stod(fields[7]));
    rotation.normalize();
    record.worldFromLidar.linear() = rotation.toRotationMatrix();
    record.point.x = float(std::stod(fields[9]));
    record.point.y = float(std::stod(fields[10]));
    record.point.z = float(std::stod(fields[11]));
    return record;
}

freedom::FreeDOM::Config mid360Config()
{
    const double pi = std::acos(-1.0);
    return freedom::FreeDOM::Config{
        0.3, 100.0, -20.0, 20.0,
        0.1, 2, 5,
        false, 100.0, -20.0, 20.0,
        100.0, -20.0, 20.0,
        6, 20,
        26, 124,
        true,
        2.0 * pi, 52.0 * pi / 180.0, -7.0 * pi / 180.0, 64,
        0.3, 100.0, 0.2,
        3, 0, 0.0, 0.0,
        false, false, std::string(),
        4};
}

using QuantizedPoint = std::tuple<std::int64_t, std::int64_t, std::int64_t>;

std::uint64_t canonicalHash(const freedom::Points& points)
{
    std::vector<QuantizedPoint> quantized;
    quantized.reserve(points.size());
    for (const freedom::Point& point : points) {
        quantized.emplace_back(
            std::llround(point.x() * 1000000.0),
            std::llround(point.y() * 1000000.0),
            std::llround(point.z() * 1000000.0));
    }
    std::sort(quantized.begin(), quantized.end());
    std::uint64_t hash = 1469598103934665603ULL;
    for (const QuantizedPoint& point : quantized) {
        for (const std::int64_t coordinate :
             {std::get<0>(point), std::get<1>(point), std::get<2>(point)}) {
            const std::uint64_t bits = static_cast<std::uint64_t>(coordinate);
            for (unsigned int byte = 0; byte < 8; ++byte) {
                hash ^= (bits >> (byte * 8)) & 0xffULL;
                hash *= 1099511628211ULL;
            }
        }
    }
    return hash;
}

void writeHeader(std::ostream& output)
{
    output << "frame,timestamp_ns,input,valid,scan_voxels,aggressive,moderate,conservative,"
              "new_free_voxels,free_blocks,free_voxels,static_blocks,static_voxels,"
              "static_subvoxels,static_map_points,static_map_voxels,point_hash,voxel_hash\n";
}

void processFrame(freedom::FreeDOM& engine,
                  const std::vector<Record>& records,
                  std::ostream& output)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.reserve(records.size());
    for (const Record& record : records) {
        cloud.points.push_back(record.point);
    }
    cloud.width = std::uint32_t(cloud.points.size());
    cloud.height = 1;
    engine.pointcloud_integrate(cloud, records.front().worldFromLidar);
    const freedom::FreeDOM::FrameStats& stats = engine.last_frame_stats();
    freedom::FreeDOM::StaticMapSnapshot map;
    engine.build_static_map_snapshot(map);
    output << records.front().frame << ','
           << records.front().timestampNs << ','
           << stats.input_point_count << ','
           << stats.valid_point_count << ','
           << stats.scan_voxel_count << ','
           << stats.aggressive_point_count << ','
           << stats.moderate_point_count << ','
           << stats.conservative_point_count << ','
           << stats.new_free_voxel_count << ','
           << stats.free_block_count << ','
           << stats.free_voxel_count << ','
           << stats.static_block_count << ','
           << stats.static_voxel_count << ','
           << stats.static_subvoxel_count << ','
           << map.point_map.size() << ','
           << map.voxel_map.size() << ','
           << canonicalHash(map.point_map) << ','
           << canonicalHash(map.voxel_map) << '\n';
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: FreeDomEquivalenceTool <replay.csv> [metrics.csv]\n";
        return 2;
    }
    try {
        std::ifstream input(argv[1]);
        if (!input) {
            throw std::runtime_error("Cannot open replay input file.");
        }
        std::ofstream fileOutput;
        std::ostream* output = &std::cout;
        if (argc == 3) {
            fileOutput.open(argv[2], std::ios::binary | std::ios::trunc);
            if (!fileOutput) {
                throw std::runtime_error("Cannot open metrics output file.");
            }
            output = &fileOutput;
        }

        freedom::FreeDOM engine;
        engine.set_params(mid360Config());
        writeHeader(*output);
        std::vector<Record> frameRecords;
        std::string line;
        while (std::getline(input, line)) {
            if (line.empty() || line.front() == '#') {
                continue;
            }
            if (line.rfind("frame,", 0) == 0) {
                continue;
            }
            Record record = parseRecord(line);
            if (!frameRecords.empty() && record.frame != frameRecords.front().frame) {
                processFrame(engine, frameRecords, *output);
                frameRecords.clear();
            }
            frameRecords.push_back(std::move(record));
        }
        if (!frameRecords.empty()) {
            processFrame(engine, frameRecords, *output);
        }
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }
    return 0;
}
