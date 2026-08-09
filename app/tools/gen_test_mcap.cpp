// Dev/regression-test tool: synthesizes a small ROS1-encoded mcap file
// (Velodyne-style sensor_msgs/PointCloud2 on /velodyne_points + sensor_msgs/
// Imu on /imu/data) for exercising fastlio_app's mcap decode path
// without needing a real recording. Not part of the production tool.
//
// Usage: fastlio_gen_test_mcap <output.mcap>
// Then:  fastlio_app --bag <output.mcap> --config <a config.yaml
//        with preprocess/timestamp_unit: 0 (SEC), matching this generator's
//        second-denominated point .time field> --headless
#include <mcap/mcap.hpp>

#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

namespace {

void writeU8(std::vector<std::byte> &buf, uint8_t v) { buf.push_back(std::byte(v)); }
void writeBool(std::vector<std::byte> &buf, bool v) { writeU8(buf, v ? 1 : 0); }
template <typename T>
void writePod(std::vector<std::byte> &buf, T v) {
  size_t off = buf.size();
  buf.resize(off + sizeof(T));
  std::memcpy(buf.data() + off, &v, sizeof(T));
}
void writeU32(std::vector<std::byte> &buf, uint32_t v) { writePod(buf, v); }
void writeF64(std::vector<std::byte> &buf, double v) { writePod(buf, v); }
void writeString(std::vector<std::byte> &buf, const std::string &s) {
  writeU32(buf, static_cast<uint32_t>(s.size()));
  size_t off = buf.size();
  buf.resize(off + s.size());
  std::memcpy(buf.data() + off, s.data(), s.size());
}
void writeHeader(std::vector<std::byte> &buf, uint32_t seq, uint32_t sec, uint32_t nsec) {
  writeU32(buf, seq);
  writeU32(buf, sec);
  writeU32(buf, nsec);
  writeString(buf, "sensor_frame");
}

// Field layout matches velodyne_ros::Point: x,y,z (f32) intensity(f32) time(f32) ring(u16)
struct SynthPoint { float x, y, z, intensity, time; uint16_t ring; };

std::vector<std::byte> encodePointCloud2(uint32_t seq, uint32_t sec, uint32_t nsec,
                                          const std::vector<SynthPoint> &pts) {
  std::vector<std::byte> buf;
  writeHeader(buf, seq, sec, nsec);
  writeU32(buf, 1);                       // height
  writeU32(buf, (uint32_t)pts.size());    // width

  struct FieldSpec { const char *name; uint32_t offset; uint8_t datatype; };
  const uint32_t pointStep = 4 + 4 + 4 + 4 + 4 + 2; // x,y,z,intensity(f32),time(f32),ring(u16)
  std::vector<FieldSpec> fields = {
    {"x", 0, 7}, {"y", 4, 7}, {"z", 8, 7}, {"intensity", 12, 7}, {"time", 16, 7}, {"ring", 20, 4},
  };
  writeU32(buf, (uint32_t)fields.size());
  for (auto &f : fields) {
    writeString(buf, f.name);
    writeU32(buf, f.offset);
    writeU8(buf, f.datatype);
    writeU32(buf, 1);
  }
  writeBool(buf, false);                            // is_bigendian
  writeU32(buf, pointStep);                         // point_step
  writeU32(buf, pointStep * (uint32_t)pts.size());  // row_step

  std::vector<std::byte> data(pointStep * pts.size());
  for (size_t i = 0; i < pts.size(); i++) {
    std::byte *base = data.data() + i * pointStep;
    std::memcpy(base + 0, &pts[i].x, 4);
    std::memcpy(base + 4, &pts[i].y, 4);
    std::memcpy(base + 8, &pts[i].z, 4);
    std::memcpy(base + 12, &pts[i].intensity, 4);
    std::memcpy(base + 16, &pts[i].time, 4);
    std::memcpy(base + 20, &pts[i].ring, 2);
  }
  writeU32(buf, (uint32_t)data.size());
  buf.insert(buf.end(), data.begin(), data.end());

  writeBool(buf, true);  // is_dense
  return buf;
}

std::vector<std::byte> encodeImu(uint32_t seq, uint32_t sec, uint32_t nsec,
                                  double gx, double gy, double gz, double ax, double ay, double az) {
  std::vector<std::byte> buf;
  writeHeader(buf, seq, sec, nsec);
  for (int i = 0; i < 4; i++) writeF64(buf, 0.0);  // orientation x,y,z,w
  for (int i = 0; i < 9; i++) writeF64(buf, 0.0);  // orientation_covariance
  writeF64(buf, gx); writeF64(buf, gy); writeF64(buf, gz);
  for (int i = 0; i < 9; i++) writeF64(buf, 0.0);  // angular_velocity_covariance
  writeF64(buf, ax); writeF64(buf, ay); writeF64(buf, az);
  for (int i = 0; i < 9; i++) writeF64(buf, 0.0);  // linear_acceleration_covariance
  return buf;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " <output.mcap>\n";
    return 1;
  }

  mcap::McapWriter writer;
  mcap::McapWriterOptions options("ros1");
  auto status = writer.open(argv[1], options);
  if (!status.ok()) {
    std::cerr << "open failed: " << status.message << "\n";
    return 1;
  }

  mcap::Schema pcSchema("sensor_msgs/PointCloud2", "ros1msg", "# synthetic test schema, not parsed");
  writer.addSchema(pcSchema);
  mcap::Schema imuSchema("sensor_msgs/Imu", "ros1msg", "# synthetic test schema, not parsed");
  writer.addSchema(imuSchema);

  mcap::Channel pcChannel("/velodyne_points", "ros1", pcSchema.id);
  writer.addChannel(pcChannel);
  mcap::Channel imuChannel("/imu/data", "ros1", imuSchema.id);
  writer.addChannel(imuChannel);

  const double lidarRateHz = 5.0;
  const double imuRateHz = 200.0;
  const int numScans = 12;
  const double durationSec = numScans / lidarRateHz;

  auto toStamp = [](double t, uint32_t &sec, uint32_t &nsec) {
    sec = (uint32_t)t;
    nsec = (uint32_t)((t - sec) * 1e9);
  };

  // Synthetic geometry: a floor plane at z=-1 and a wall at y=5, in the
  // sensor's body frame -- enough plane structure for point-to-plane ICP.
  // This is a decode/wiring smoke test, not a trajectory-accuracy test.
  std::vector<SynthPoint> scanTemplate;
  for (double x = -5.0; x <= 5.0; x += 0.25) {
    for (double y = -5.0; y <= 5.0; y += 0.25) {
      scanTemplate.push_back(SynthPoint{(float)x, (float)y, -1.f, 100.f, 0.f, (uint16_t)0});
    }
  }
  for (double x = -5.0; x <= 5.0; x += 0.25) {
    for (double z = -1.0; z <= 2.0; z += 0.25) {
      scanTemplate.push_back(SynthPoint{(float)x, 5.f, (float)z, 100.f, 0.f, (uint16_t)1});
    }
  }
  for (size_t i = 0; i < scanTemplate.size(); i++) {
    scanTemplate[i].time = (float)i / (float)scanTemplate.size() * (1.0f / (float)lidarRateHz);
  }

  uint32_t imuSeq = 0, lidarSeq = 0;
  double t = 0.0;
  double nextLidar = 0.0;
  const double imuDt = 1.0 / imuRateHz;

  while (t <= durationSec + 1e-9) {
    uint32_t sec, nsec;
    toStamp(t, sec, nsec);

    auto imuBytes = encodeImu(imuSeq++, sec, nsec, 0.001, -0.001, 0.0005, 0.02, -0.01, 9.81);
    mcap::Message imuMsg;
    imuMsg.channelId = imuChannel.id;
    imuMsg.sequence = imuSeq;
    imuMsg.logTime = (mcap::Timestamp)(t * 1e9);
    imuMsg.publishTime = imuMsg.logTime;
    imuMsg.data = imuBytes.data();
    imuMsg.dataSize = imuBytes.size();
    (void)writer.write(imuMsg);

    if (t + 1e-9 >= nextLidar && lidarSeq < (uint32_t)numScans) {
      uint32_t lsec, lnsec;
      toStamp(t, lsec, lnsec);
      auto pcBytes = encodePointCloud2(lidarSeq, lsec, lnsec, scanTemplate);
      mcap::Message pcMsg;
      pcMsg.channelId = pcChannel.id;
      pcMsg.sequence = lidarSeq;
      pcMsg.logTime = (mcap::Timestamp)(t * 1e9);
      pcMsg.publishTime = pcMsg.logTime;
      pcMsg.data = pcBytes.data();
      pcMsg.dataSize = pcBytes.size();
      (void)writer.write(pcMsg);
      lidarSeq++;
      nextLidar += 1.0 / lidarRateHz;
    }

    t += imuDt;
  }

  writer.close();
  std::cout << "wrote " << lidarSeq << " lidar scans, " << imuSeq << " imu samples to " << argv[1] << "\n";
  return 0;
}
