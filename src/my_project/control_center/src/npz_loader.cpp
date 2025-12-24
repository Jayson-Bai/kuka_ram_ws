#include "control_center/npz_loader.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>

#include "cnpy.h"

namespace fs = std::filesystem;

namespace control_center {

namespace {

std::string trim_cstr(const char* data, size_t max_len) {
  size_t len = 0;
  while (len < max_len && data[len] != '\0') {
    ++len;
  }
  return std::string(data, len);
}

std::vector<std::string> decode_fixed_strings(const cnpy::NpyArray& arr) {
  std::vector<std::string> out;
  const size_t n = arr.num_vals;
  const size_t stride = arr.word_size;
  const char* raw = arr.data<char>();
  out.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    out.emplace_back(trim_cstr(raw + i * stride, stride));
  }
  return out;
}

template <typename T>
std::vector<T> to_vec(const cnpy::NpyArray& arr) {
  const T* data = arr.data<T>();
  return std::vector<T>(data, data + arr.num_vals);
}

void load_vocab(
    const cnpy::npz_t& npz,
    const std::string& keys_name,
    const std::string& vals_name,
    std::unordered_map<uint8_t, std::string>& out) {
  auto keys = decode_fixed_strings(npz.at(keys_name));
  auto vals = to_vec<uint8_t>(npz.at(vals_name));
  out.clear();
  const size_t n = std::min(keys.size(), vals.size());
  for (size_t i = 0; i < n; ++i) {
    out[vals[i]] = keys[i];
  }
}

}  // namespace

NpzLoader::NpzLoader(const std::string& path, size_t preload_chunks)
    : preload_chunks_(preload_chunks) {
  files_ = resolve_files(path);
  if (files_.empty()) {
    error_ = "no npz files found for: " + path;
    ok_ = false;
    return;
  }
  try {
    load_initial();
    ok_ = true;
  } catch (const std::exception& e) {
    error_ = e.what();
    ok_ = false;
  }
}

bool NpzLoader::has_next() const {
  if (!cache_.empty()) {
    const auto& chunk = cache_.front();
    if (chunk_row_idx_ < chunk.size) {
      return true;
    }
  }
  return next_file_idx_ < files_.size();
}

bool NpzLoader::next_row(NpzRow& out) {
  if (cache_.empty() || chunk_row_idx_ >= cache_.front().size) {
    if (!load_next_chunk()) {
      return false;
    }
  }
  const auto& c = cache_.front();
  const size_t i = chunk_row_idx_++;
  out.seq = c.seq[i];
  out.x = c.x[i];
  out.y = c.y[i];
  out.z = c.z[i];
  out.a = c.a[i];
  out.b = c.b[i];
  out.c = c.c[i];
  out.e = c.e[i];
  out.tool_id = c.tool_id[i];
  out.move_type = c.move_type[i];
  out.src_line = c.src_line[i];
  out.event_flag = c.event_flag[i];
  out.event_type = c.event_type[i];
  out.payload = c.payload[i];
  out.trigger_seq = c.trigger_seq[i];

  if (chunk_row_idx_ >= c.size) {
    cache_.pop_front();
    chunk_row_idx_ = 0;
    ensure_preload();
  }
  return true;
}

void NpzLoader::load_initial() {
  for (size_t i = 0; i < preload_chunks_; ++i) {
    if (!load_next_chunk()) {
      break;
    }
  }
}

void NpzLoader::ensure_preload() {
  while (cache_.size() < preload_chunks_) {
    if (!load_next_chunk()) {
      break;
    }
  }
}

bool NpzLoader::load_next_chunk() {
  if (next_file_idx_ >= files_.size()) {
    return false;
  }
  const std::string& file = files_[next_file_idx_++];
  cache_.push_back(load_chunk(file));
  return true;
}

std::vector<std::string> NpzLoader::resolve_files(const std::string& path) const {
  fs::path p(path);
  std::vector<std::string> out;

  if (p.extension() != ".npz") {
    p.replace_extension(".npz");
  }

  if (fs::exists(p)) {
    out.push_back(p.string());
    return out;
  }

  const fs::path dir = p.parent_path().empty() ? fs::current_path() : p.parent_path();
  const std::string stem = p.stem().string();
  const std::string prefix = stem + "_part";

  if (!fs::exists(dir)) {
    return out;
  }

  for (const auto& entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto& f = entry.path();
    if (f.extension() != ".npz") {
      continue;
    }
    const std::string name = f.stem().string();
    if (name.rfind(prefix, 0) == 0) {
      out.push_back(f.string());
    }
  }

  std::sort(out.begin(), out.end());
  return out;
}

NpzChunk NpzLoader::load_chunk(const std::string& file) {
  cnpy::npz_t npz = cnpy::npz_load(file);

  if (move_type_vocab_.empty() && npz.count("move_type_vocab_keys")) {
    load_vocab(npz, "move_type_vocab_keys", "move_type_vocab_vals", move_type_vocab_);
  }
  if (event_type_vocab_.empty() && npz.count("event_type_vocab_keys")) {
    load_vocab(npz, "event_type_vocab_keys", "event_type_vocab_vals", event_type_vocab_);
  }

  NpzChunk c;
  c.seq = to_vec<uint32_t>(npz.at("seq"));
  c.x = to_vec<float>(npz.at("x"));
  c.y = to_vec<float>(npz.at("y"));
  c.z = to_vec<float>(npz.at("z"));
  c.a = to_vec<float>(npz.at("a"));
  c.b = to_vec<float>(npz.at("b"));
  c.c = to_vec<float>(npz.at("c"));
  c.e = to_vec<float>(npz.at("e"));
  {
    auto raw_tool = to_vec<uint8_t>(npz.at("tool_id"));
    c.tool_id.reserve(raw_tool.size());
    for (auto v : raw_tool) {
      c.tool_id.push_back(static_cast<int32_t>(v));
    }
  }
  c.move_type = to_vec<uint8_t>(npz.at("move_type"));
  c.src_line = decode_fixed_strings(npz.at("src_line"));
  c.event_flag = to_vec<uint8_t>(npz.at("event_flag"));
  c.event_type = to_vec<uint8_t>(npz.at("event_type"));
  c.payload = decode_fixed_strings(npz.at("payload"));
  c.trigger_seq = to_vec<int32_t>(npz.at("trigger_seq"));
  c.size = c.seq.size();

  return c;
}

}  // namespace control_center
