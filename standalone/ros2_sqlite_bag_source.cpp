#include "ros2_sqlite_bag_source.h"

#include <algorithm>
#include <map>
#include <stdexcept>
#include <sqlite3.h>

namespace fastlio_standalone
{

namespace
{

void checkStep(int rc, sqlite3 *db, sqlite3_stmt *stmt, const std::string &what)
{
  if (rc != SQLITE_ROW && rc != SQLITE_DONE)
  {
    std::string msg = sqlite3_errmsg(db);
    sqlite3_finalize(stmt);
    throw std::runtime_error("sqlite3: " + what + ": " + msg);
  }
}

}  // namespace

Ros2SqliteBagSource::Ros2SqliteBagSource(const std::string &path) : path_(path) {}

void Ros2SqliteBagSource::forEachMessage(const std::vector<std::string> &topics,
                                          const std::function<void(const RawMessage &)> &cb)
{
  sqlite3 *db = nullptr;
  if (sqlite3_open_v2(path_.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK)
  {
    std::string err = db ? sqlite3_errmsg(db) : "unknown error";
    if (db) sqlite3_close(db);
    throw std::runtime_error("failed to open ros2 bag '" + path_ + "': " + err);
  }
  struct DbGuard
  {
    sqlite3 *db;
    ~DbGuard() { sqlite3_close(db); }
  } dbGuard{db};

  // Resolve the requested topic names to (id, serialization_format).
  std::map<int64_t, std::string> topicIdToName;
  std::map<int64_t, std::string> topicIdToEncoding;
  {
    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(db, "SELECT id, name, serialization_format FROM topics", -1, &stmt, nullptr) != SQLITE_OK)
    {
      throw std::runtime_error(std::string("sqlite3: preparing topics query: ") + sqlite3_errmsg(db));
    }
    while (true)
    {
      int rc = sqlite3_step(stmt);
      if (rc == SQLITE_DONE) break;
      checkStep(rc, db, stmt, "stepping topics query");
      int64_t id = sqlite3_column_int64(stmt, 0);
      std::string name = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
      std::string encoding = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));
      if (std::find(topics.begin(), topics.end(), name) != topics.end())
      {
        topicIdToName[id] = name;
        topicIdToEncoding[id] = encoding;
      }
    }
    sqlite3_finalize(stmt);
  }

  if (topicIdToName.empty())
  {
    throw std::runtime_error("none of the requested topics were found in ros2 bag '" + path_ + "'");
  }

  std::string placeholders;
  for (size_t i = 0; i < topicIdToName.size(); i++)
  {
    if (i) placeholders += ",";
    placeholders += "?";
  }
  // messages.timestamp already has an index (timestamp_idx) in rosbag2's
  // schema, so this is an indexed scan, not a full-table sort, even on a
  // multi-GB bag.
  std::string sql = "SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN (" + placeholders +
                     ") ORDER BY timestamp ASC";

  sqlite3_stmt *stmt = nullptr;
  if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK)
  {
    throw std::runtime_error(std::string("sqlite3: preparing messages query: ") + sqlite3_errmsg(db));
  }
  int bindIdx = 1;
  for (const auto &[id, name] : topicIdToName)
  {
    sqlite3_bind_int64(stmt, bindIdx++, id);
  }

  while (true)
  {
    int rc = sqlite3_step(stmt);
    if (rc == SQLITE_DONE) break;
    checkStep(rc, db, stmt, "stepping messages query");

    int64_t topicId = sqlite3_column_int64(stmt, 0);
    int64_t timestampNs = sqlite3_column_int64(stmt, 1);
    const void *blob = sqlite3_column_blob(stmt, 2);
    int blobSize = sqlite3_column_bytes(stmt, 2);

    RawMessage msg;
    msg.topic = topicIdToName[topicId];
    msg.encoding = topicIdToEncoding[topicId];
    msg.timestampSec = static_cast<double>(timestampNs) * 1e-9;
    msg.data = reinterpret_cast<const std::byte *>(blob);
    msg.size = static_cast<size_t>(blobSize);
    cb(msg);  // msg.data is only valid until the next sqlite3_step below
  }
  sqlite3_finalize(stmt);
}

}  // namespace fastlio_standalone
