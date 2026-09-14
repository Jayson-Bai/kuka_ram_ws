#include "cnpy.h"

#include <zlib.h>

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

template<typename T>
void append_little_endian(std::vector<unsigned char> & output, T value)
{
  for (size_t index = 0; index < sizeof(T); ++index) {
    output.push_back(static_cast<unsigned char>((value >> (index * 8U)) & 0xffU));
  }
}

std::vector<unsigned char> make_npy()
{
  std::vector<unsigned char> output = {
    0x93U, 'N', 'U', 'M', 'P', 'Y', 0x01U, 0x00U
  };
  std::string header = "{'descr': '<i4', 'fortran_order': False, 'shape': (3,), }";
  const size_t padding = (16U - ((10U + header.size() + 1U) % 16U)) % 16U;
  header.append(padding, ' ');
  header.push_back('\n');
  append_little_endian<uint16_t>(output, static_cast<uint16_t>(header.size()));
  output.insert(output.end(), header.begin(), header.end());
  append_little_endian<int32_t>(output, 10);
  append_little_endian<int32_t>(output, 20);
  append_little_endian<int32_t>(output, 30);
  return output;
}

std::vector<unsigned char> deflate_raw(const std::vector<unsigned char> & input)
{
  z_stream stream{};
  if (deflateInit2(
      &stream, Z_DEFAULT_COMPRESSION, Z_DEFLATED, -MAX_WBITS, 8,
      Z_DEFAULT_STRATEGY) != Z_OK)
  {
    throw std::runtime_error("failed to initialize ZIP test compressor");
  }
  std::vector<unsigned char> output(compressBound(input.size()));
  stream.next_in = const_cast<Bytef *>(input.data());
  stream.avail_in = static_cast<uInt>(input.size());
  stream.next_out = output.data();
  stream.avail_out = static_cast<uInt>(output.size());
  const int result = deflate(&stream, Z_FINISH);
  const size_t size = stream.total_out;
  deflateEnd(&stream);
  if (result != Z_STREAM_END) {
    throw std::runtime_error("failed to compress ZIP test member");
  }
  output.resize(size);
  return output;
}

void append_zip64_extra(
  std::vector<unsigned char> & output, uint64_t uncompressed, uint64_t compressed)
{
  append_little_endian<uint16_t>(output, 0x0001U);
  append_little_endian<uint16_t>(output, 16U);
  append_little_endian<uint64_t>(output, uncompressed);
  append_little_endian<uint64_t>(output, compressed);
}

std::filesystem::path write_zip64_npz()
{
  const std::string name = "seq.npy";
  const auto npy = make_npy();
  const auto compressed = deflate_raw(npy);
  const uint32_t crc = crc32(0L, npy.data(), static_cast<uInt>(npy.size()));
  std::vector<unsigned char> zip;

  append_little_endian<uint32_t>(zip, 0x04034b50U);
  append_little_endian<uint16_t>(zip, 45U);
  append_little_endian<uint16_t>(zip, 0U);
  append_little_endian<uint16_t>(zip, 8U);
  append_little_endian<uint16_t>(zip, 0U);
  append_little_endian<uint16_t>(zip, 0U);
  append_little_endian<uint32_t>(zip, crc);
  append_little_endian<uint32_t>(zip, 0xffffffffU);
  append_little_endian<uint32_t>(zip, 0xffffffffU);
  append_little_endian<uint16_t>(zip, static_cast<uint16_t>(name.size()));
  append_little_endian<uint16_t>(zip, 20U);
  zip.insert(zip.end(), name.begin(), name.end());
  append_zip64_extra(zip, npy.size(), compressed.size());
  zip.insert(zip.end(), compressed.begin(), compressed.end());

  const uint32_t central_offset = static_cast<uint32_t>(zip.size());
  append_little_endian<uint32_t>(zip, 0x02014b50U);
  append_little_endian<uint16_t>(zip, 45U);
  append_little_endian<uint16_t>(zip, 45U);
  append_little_endian<uint16_t>(zip, 0U);
  append_little_endian<uint16_t>(zip, 8U);
  append_little_endian<uint16_t>(zip, 0U);
  append_little_endian<uint16_t>(zip, 0U);
  append_little_endian<uint32_t>(zip, crc);
  append_little_endian<uint32_t>(zip, 0xffffffffU);
  append_little_endian<uint32_t>(zip, 0xffffffffU);
  append_little_endian<uint16_t>(zip, static_cast<uint16_t>(name.size()));
  append_little_endian<uint16_t>(zip, 20U);
  append_little_endian<uint16_t>(zip, 0U);
  append_little_endian<uint16_t>(zip, 0U);
  append_little_endian<uint16_t>(zip, 0U);
  append_little_endian<uint32_t>(zip, 0U);
  append_little_endian<uint32_t>(zip, 0U);
  zip.insert(zip.end(), name.begin(), name.end());
  append_zip64_extra(zip, npy.size(), compressed.size());

  const uint32_t central_size = static_cast<uint32_t>(zip.size()) - central_offset;
  append_little_endian<uint32_t>(zip, 0x06054b50U);
  append_little_endian<uint16_t>(zip, 0U);
  append_little_endian<uint16_t>(zip, 0U);
  append_little_endian<uint16_t>(zip, 1U);
  append_little_endian<uint16_t>(zip, 1U);
  append_little_endian<uint32_t>(zip, central_size);
  append_little_endian<uint32_t>(zip, central_offset);
  append_little_endian<uint16_t>(zip, 0U);

  const auto path = std::filesystem::temp_directory_path() / "control_center_cnpy_zip64.npz";
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  stream.write(
    reinterpret_cast<const char *>(zip.data()),
    static_cast<std::streamsize>(zip.size()));
  if (!stream) {
    throw std::runtime_error("failed to write ZIP64 NPZ fixture");
  }
  return path;
}

bool values_are_correct(const cnpy::NpyArray & array)
{
  if (array.shape != std::vector<size_t>{3U} || array.word_size != sizeof(int32_t)) {
    return false;
  }
  const auto values = array.as_vec<int32_t>();
  return values == std::vector<int32_t>{10, 20, 30};
}

}  // namespace

int main()
{
  const auto path = write_zip64_npz();
  try {
    const auto arrays = cnpy::npz_load(path.string());
    if (arrays.count("seq") != 1U || !values_are_correct(arrays.at("seq"))) {
      std::filesystem::remove(path);
      return 1;
    }
    if (!values_are_correct(cnpy::npz_load(path.string(), "seq"))) {
      std::filesystem::remove(path);
      return 2;
    }
  } catch (...) {
    std::filesystem::remove(path);
    throw;
  }
  std::filesystem::remove(path);
  return 0;
}
