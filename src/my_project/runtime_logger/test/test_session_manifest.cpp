#include <gtest/gtest.h>

#include "runtime_logger/session_manifest.hpp"

#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

namespace
{

std::string read_file(const fs::path & path)
{
  std::ifstream in(path);
  std::ostringstream os;
  os << in.rdbuf();
  return os.str();
}

}  // namespace

TEST(SessionManifest, PreservesConfigOnFinalize)
{
  const fs::path root = fs::temp_directory_path() /
    ("rtlog_manifest_test_" + std::to_string(::getpid()));
  fs::remove_all(root);

  runtime_logger::SessionManifest manifest;
  const std::string config =
    "{\"rsi_context\":\"PythonHomeDemo\",\"rsi_pt1_t1\":\"0.012\"}";
  ASSERT_TRUE(
    manifest.start(
      root.string(), "testrun", "test_version", config, false));

  const fs::path manifest_path = root / "run_testrun" / "manifest.json";
  const std::string at_start = read_file(manifest_path);
  EXPECT_NE(at_start.find("\"config\":" + config), std::string::npos);

  ASSERT_TRUE(manifest.finalize(true, "{}", "shutdown"));
  const std::string at_end = read_file(manifest_path);
  EXPECT_NE(at_end.find("\"config\":" + config), std::string::npos);
  EXPECT_NE(at_end.find("\"complete\":true"), std::string::npos);

  fs::remove_all(root);
}

TEST(SessionManifest, EmptyConfigNormalizesToObject)
{
  const fs::path root = fs::temp_directory_path() /
    ("rtlog_manifest_test_empty_" + std::to_string(::getpid()));
  fs::remove_all(root);

  runtime_logger::SessionManifest manifest;
  ASSERT_TRUE(manifest.start(root.string(), "empty", "v", "", false));
  ASSERT_TRUE(manifest.finalize(true, "{}", "shutdown"));

  const std::string body = read_file(
    root / "run_empty" / "manifest.json");
  EXPECT_NE(body.find("\"config\":{}"), std::string::npos);

  fs::remove_all(root);
}
