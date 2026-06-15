#ifndef RSI_SERVER__RSI_XML_PARSER_HPP_
#define RSI_SERVER__RSI_XML_PARSER_HPP_

#include <cctype>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>

namespace rsi_server
{

struct PoseXYZABC
{
  double x{std::numeric_limits<double>::quiet_NaN()};
  double y{std::numeric_limits<double>::quiet_NaN()};
  double z{std::numeric_limits<double>::quiet_NaN()};
  double a{std::numeric_limits<double>::quiet_NaN()};
  double b{std::numeric_limits<double>::quiet_NaN()};
  double c{std::numeric_limits<double>::quiet_NaN()};
};

inline bool is_pose_axis(char c)
{
  const char upper = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return upper == 'X' || upper == 'Y' || upper == 'Z' ||
         upper == 'A' || upper == 'B' || upper == 'C';
}

inline std::optional<PoseXYZABC> parse_kuka_xyzabc(const std::string & xml)
{
  PoseXYZABC pose;
  bool found_any = false;

  for (std::size_t i = 0; i < xml.size(); ++i) {
    const char key = static_cast<char>(std::toupper(static_cast<unsigned char>(xml[i])));
    if (!is_pose_axis(key)) {
      continue;
    }

    if (i > 0) {
      const unsigned char prev = static_cast<unsigned char>(xml[i - 1]);
      if (std::isalnum(prev) || xml[i - 1] == '_') {
        continue;
      }
    }

    std::size_t pos = i + 1;
    while (pos < xml.size() && std::isspace(static_cast<unsigned char>(xml[pos]))) {
      ++pos;
    }
    if (pos >= xml.size() || xml[pos] != '=') {
      continue;
    }
    ++pos;
    while (pos < xml.size() && std::isspace(static_cast<unsigned char>(xml[pos]))) {
      ++pos;
    }
    if (pos >= xml.size() || xml[pos] != '"') {
      continue;
    }
    ++pos;

    char * end = nullptr;
    const double value = std::strtod(xml.c_str() + pos, &end);
    if (end == nullptr || end == xml.c_str() + pos || *end != '"') {
      continue;
    }

    found_any = true;
    switch (key) {
      case 'X':
        pose.x = value;
        break;
      case 'Y':
        pose.y = value;
        break;
      case 'Z':
        pose.z = value;
        break;
      case 'A':
        pose.a = value;
        break;
      case 'B':
        pose.b = value;
        break;
      case 'C':
        pose.c = value;
        break;
      default:
        break;
    }

    i = static_cast<std::size_t>(end - xml.c_str());
  }

  if (!found_any) {
    return std::nullopt;
  }
  return pose;
}

}  // namespace rsi_server

#endif  // RSI_SERVER__RSI_XML_PARSER_HPP_
