#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace control_center {

struct NpzRow {
  uint32_t seq{};
  float x{};
  float y{};
  float z{};
  float a{};
  float b{};
  float c{};
  float e{};
  int32_t tool_id{};
  uint8_t move_type{};
  std::string src_line;
  uint8_t event_flag{};
  uint8_t event_type{};
  std::string payload;
  int32_t trigger_seq{};
};

struct NpzChunk {
  std::vector<uint32_t> seq;
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> z;
  std::vector<float> a;
  std::vector<float> b;
  std::vector<float> c;
  std::vector<float> e;
  std::vector<int32_t> tool_id;
  std::vector<uint8_t> move_type;
  std::vector<std::string> src_line;
  std::vector<uint8_t> event_flag;
  std::vector<uint8_t> event_type;
  std::vector<std::string> payload;
  std::vector<int32_t> trigger_seq;
  size_t size{0};
};

class NpzLoader {
public:
  explicit NpzLoader(const std::string& path, size_t preload_chunks = 2);

  bool ok() const { return ok_; }
  const std::string& error() const { return error_; }

  bool has_next() const;
  bool next_row(NpzRow& out);

  const std::unordered_map<uint8_t, std::string>& move_type_vocab() const {
    return move_type_vocab_;
  }
  const std::unordered_map<uint8_t, std::string>& event_type_vocab() const {
    return event_type_vocab_;
  }

private:
  void load_initial();
  void ensure_preload();
  bool load_next_chunk();

  std::vector<std::string> resolve_files(const std::string& path) const;
  NpzChunk load_chunk(const std::string& file);

  std::unordered_map<uint8_t, std::string> move_type_vocab_;
  std::unordered_map<uint8_t, std::string> event_type_vocab_;

  std::vector<std::string> files_;
  size_t next_file_idx_{0};
  size_t preload_chunks_{2};
  bool ok_{false};
  std::string error_;

  std::deque<NpzChunk> cache_;
  size_t chunk_row_idx_{0};
};

}  // namespace control_center
