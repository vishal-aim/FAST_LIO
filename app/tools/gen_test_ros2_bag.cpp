// Dev/regression-test tool: synthesizes a small rosbag2 sqlite3 (.db3) file
// (Velodyne-style sensor_msgs/msg/PointCloud2 on /velodyne_points +
// sensor_msgs/msg/Imu on /imu/data, CDR-encoded) for exercising
// fastlio_app's ROS2 decode path without needing a real recording.
// Not part of the production tool.
//
// Usage: fastlio_gen_test_ros2_bag <output.db3>
// Then:  fastlio_app --bag <output.db3> --config <a config.yaml with
//        preprocess/timestamp_unit: 0 (SEC), matching this generator's
//        second-denominated point .time field>
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>
#include <sqlite3.h>

namespace {

// Minimal CDR writer mirroring ros2_cdr_deserialize.h's Reader -- same
// encapsulation header and alignment rules, just writing instead of reading.
class CdrWriter {
 public:
  CdrWriter() {
    buf_.push_back(std::byte{0x00});
    buf_.push_back(std::byte{0x01});  // CDR_LE
    buf_.push_back(std::byte{0x00});
    buf_.push_back(std::byte{0x00});
    payloadStart_ = 4;
  }

  void writeU8(uint8_t v) { writePod(v); }
  void writeBool(bool v) { writeU8(v ? 1 : 0); }
  void writeI32(int32_t v) { writePod(v); }
  void writeU32(uint32_t v) { writePod(v); }
  void writeF64(double v) { writePod(v); }

  void writeString(const std::string &s) {
    writeU32((uint32_t)s.size() + 1);  // CDR string length includes the null terminator
    for (char c : s) buf_.push_back(std::byte(c));
    buf_.push_back(std::byte{0});
  }

  void writeBytesSeq(const void *data, size_t n) {
    writeU32((uint32_t)n);
    const std::byte *p = reinterpret_cast<const std::byte *>(data);
    buf_.insert(buf_.end(), p, p + n);
  }

  const std::vector<std::byte> &data() const { return buf_; }

 private:
  void align(size_t alignment) {
    size_t rel = buf_.size() - payloadStart_;
    size_t rem = rel % alignment;
    if (rem != 0) {
      for (size_t i = 0; i < alignment - rem; i++) buf_.push_back(std::byte{0});
    }
  }

  template <typename T>
  void writePod(T v) {
    align(sizeof(T));
    const std::byte *p = reinterpret_cast<const std::byte *>(&v);
    buf_.insert(buf_.end(), p, p + sizeof(T));
  }

  std::vector<std::byte> buf_;
  size_t payloadStart_ = 4;
};

void writeHeader(CdrWriter &w, int32_t sec, uint32_t nanosec, const std::string &frameId) {
  w.writeI32(sec);
  w.writeU32(nanosec);
  w.writeString(frameId);
}

// Field layout matches velodyne_ros::Point: x,y,z (f32) intensity(f32) time(f32) ring(u16)
struct SynthPoint { float x, y, z, intensity, time; uint16_t ring; };

std::vector<std::byte> encodePointCloud2(int32_t sec, uint32_t nanosec, const std::vector<SynthPoint> &pts) {
  CdrWriter w;
  writeHeader(w, sec, nanosec, "sensor_frame");
  w.writeU32(1);                    // height
  w.writeU32((uint32_t)pts.size()); // width

  struct FieldSpec { const char *name; uint32_t offset; uint8_t datatype; };
  const uint32_t pointStep = 4 + 4 + 4 + 4 + 4 + 2; // x,y,z,intensity(f32),time(f32),ring(u16)
  std::vector<FieldSpec> fields = {
    {"x", 0, 7}, {"y", 4, 7}, {"z", 8, 7}, {"intensity", 12, 7}, {"time", 16, 7}, {"ring", 20, 4},
  };
  w.writeU32((uint32_t)fields.size());
  for (auto &f : fields) {
    w.writeString(f.name);
    w.writeU32(f.offset);
    w.writeU8(f.datatype);
    w.writeU32(1);  // PointField.count
  }

  w.writeBool(false);       // is_bigendian
  w.writeU32(pointStep);    // point_step
  w.writeU32(pointStep * (uint32_t)pts.size());  // row_step

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
  w.writeBytesSeq(data.data(), data.size());

  w.writeBool(true);  // is_dense
  return w.data();
}

std::vector<std::byte> encodeImu(int32_t sec, uint32_t nanosec,
                                  double gx, double gy, double gz, double ax, double ay, double az) {
  CdrWriter w;
  writeHeader(w, sec, nanosec, "imu_frame");
  for (int i = 0; i < 4; i++) w.writeF64(0.0);  // orientation x,y,z,w
  for (int i = 0; i < 9; i++) w.writeF64(0.0);  // orientation_covariance
  w.writeF64(gx); w.writeF64(gy); w.writeF64(gz);
  for (int i = 0; i < 9; i++) w.writeF64(0.0);  // angular_velocity_covariance
  w.writeF64(ax); w.writeF64(ay); w.writeF64(az);
  for (int i = 0; i < 9; i++) w.writeF64(0.0);  // linear_acceleration_covariance
  return w.data();
}

void checkOk(int rc, sqlite3 *db, const std::string &what) {
  if (rc != SQLITE_OK) {
    std::cerr << what << ": " << sqlite3_errmsg(db) << "\n";
    std::exit(1);
  }
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " <output.db3>\n";
    return 1;
  }

  std::remove(argv[1]);  // rosbag2 (and sqlite3_open_v2 with CREATE) won't overwrite a stale file's schema
  sqlite3 *db = nullptr;
  checkOk(sqlite3_open_v2(argv[1], &db, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, nullptr), db, "open");

  const char *schema =
      "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, "
      "serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL, type_description_hash TEXT NOT NULL);"
      "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, "
      "data BLOB NOT NULL);"
      "CREATE INDEX timestamp_idx ON messages (timestamp ASC);";
  char *errMsg = nullptr;
  if (sqlite3_exec(db, schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
    std::cerr << "schema creation failed: " << errMsg << "\n";
    return 1;
  }

  sqlite3_exec(db,
               "INSERT INTO topics (id,name,type,serialization_format,offered_qos_profiles,type_description_hash) "
               "VALUES (1,'/velodyne_points','sensor_msgs/msg/PointCloud2','cdr','',''), "
               "(2,'/imu/data','sensor_msgs/msg/Imu','cdr','','')",
               nullptr, nullptr, &errMsg);

  sqlite3_stmt *insertStmt = nullptr;
  checkOk(sqlite3_prepare_v2(db, "INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)", -1,
                              &insertStmt, nullptr),
          db, "prepare insert");

  const double lidarRateHz = 5.0;
  const double imuRateHz = 200.0;
  const int numScans = 12;
  const double durationSec = numScans / lidarRateHz;

  // Synthetic geometry: a floor plane at z=-1 and a wall at y=5, in the
  // sensor's body frame -- enough plane structure for point-to-plane ICP.
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

  auto insert = [&](int64_t topicId, int64_t timestampNs, const std::vector<std::byte> &bytes) {
    sqlite3_reset(insertStmt);
    sqlite3_bind_int64(insertStmt, 1, topicId);
    sqlite3_bind_int64(insertStmt, 2, timestampNs);
    sqlite3_bind_blob(insertStmt, 3, bytes.data(), (int)bytes.size(), SQLITE_TRANSIENT);
    if (sqlite3_step(insertStmt) != SQLITE_DONE) {
      std::cerr << "insert failed: " << sqlite3_errmsg(db) << "\n";
      std::exit(1);
    }
  };

  sqlite3_exec(db, "BEGIN", nullptr, nullptr, nullptr);

  uint32_t lidarSeq = 0, imuSeq = 0;
  double t = 0.0;
  double nextLidar = 0.0;
  const double imuDt = 1.0 / imuRateHz;

  while (t <= durationSec + 1e-9) {
    int32_t sec = (int32_t)t;
    uint32_t nanosec = (uint32_t)((t - sec) * 1e9);
    int64_t timestampNs = (int64_t)(t * 1e9);

    auto imuBytes = encodeImu(sec, nanosec, 0.001, -0.001, 0.0005, 0.02, -0.01, 9.81);
    insert(2, timestampNs, imuBytes);
    imuSeq++;

    if (t + 1e-9 >= nextLidar && lidarSeq < (uint32_t)numScans) {
      auto pcBytes = encodePointCloud2(sec, nanosec, scanTemplate);
      insert(1, timestampNs, pcBytes);
      lidarSeq++;
      nextLidar += 1.0 / lidarRateHz;
    }

    t += imuDt;
  }

  sqlite3_exec(db, "COMMIT", nullptr, nullptr, nullptr);
  sqlite3_finalize(insertStmt);
  sqlite3_close(db);

  std::cout << "wrote " << lidarSeq << " lidar scans, " << imuSeq << " imu samples to " << argv[1] << "\n";
  return 0;
}
