//Copyright (C) 2011  Carl Rogers
//Released under MIT License
//license available in LICENSE file, or at http://www.opensource.org/licenses/mit-license.php

#include "cnpy.h"
#include <complex>
#include <cstdlib>
#include <algorithm>
#include <cstring>
#include <iomanip>
#include <limits>
#include <stdint.h>
#include <stdexcept>
#include <regex>

namespace
{

constexpr uint32_t ZIP32_SIZE_SENTINEL = 0xffffffffU;
constexpr uint16_t ZIP64_EXTRA_FIELD_ID = 0x0001U;

template<typename T>
T read_little_endian(const char * data)
{
  T value{};
  std::memcpy(&value, data, sizeof(T));
  return value;
}

struct ZipEntrySizes
{
  size_t compressed{};
  size_t uncompressed{};
};

ZipEntrySizes read_zip_entry_sizes(
  const std::vector<char> & local_header,
  const std::vector<char> & extra_field)
{
  uint64_t compressed = read_little_endian<uint32_t>(&local_header[18]);
  uint64_t uncompressed = read_little_endian<uint32_t>(&local_header[22]);
  bool needs_compressed = compressed == ZIP32_SIZE_SENTINEL;
  bool needs_uncompressed = uncompressed == ZIP32_SIZE_SENTINEL;

  size_t offset = 0;
  while ((needs_compressed || needs_uncompressed) && offset + 4 <= extra_field.size()) {
    const uint16_t field_id = read_little_endian<uint16_t>(&extra_field[offset]);
    const uint16_t field_size = read_little_endian<uint16_t>(&extra_field[offset + 2]);
    const size_t payload = offset + 4;
    const size_t end = payload + static_cast<size_t>(field_size);
    if (end > extra_field.size()) {
      throw std::runtime_error("npz_load: malformed ZIP extra field");
    }
    if (field_id == ZIP64_EXTRA_FIELD_ID) {
      size_t cursor = payload;
      if (needs_uncompressed) {
        if (cursor + sizeof(uint64_t) > end) {
          throw std::runtime_error("npz_load: ZIP64 extra field has no uncompressed size");
        }
        uncompressed = read_little_endian<uint64_t>(&extra_field[cursor]);
        cursor += sizeof(uint64_t);
        needs_uncompressed = false;
      }
      if (needs_compressed) {
        if (cursor + sizeof(uint64_t) > end) {
          throw std::runtime_error("npz_load: ZIP64 extra field has no compressed size");
        }
        compressed = read_little_endian<uint64_t>(&extra_field[cursor]);
        needs_compressed = false;
      }
    }
    offset = end;
  }

  if (needs_compressed || needs_uncompressed) {
    throw std::runtime_error("npz_load: ZIP64 sizes are missing from the local extra field");
  }
  if (
    compressed > std::numeric_limits<size_t>::max() ||
    uncompressed > std::numeric_limits<size_t>::max())
  {
    throw std::runtime_error("npz_load: ZIP member is too large for this platform");
  }
  return {
    static_cast<size_t>(compressed),
    static_cast<size_t>(uncompressed)
  };
}

void seek_forward(FILE * fp, size_t byte_count)
{
  if (byte_count > static_cast<size_t>(std::numeric_limits<long>::max())) {
    throw std::runtime_error("npz_load: ZIP member is too large to seek");
  }
  if (fseek(fp, static_cast<long>(byte_count), SEEK_CUR) != 0) {
    throw std::runtime_error("npz_load: failed to seek over ZIP member");
  }
}

}  // namespace

char cnpy::BigEndianTest()
{
  int x = 1;
  return (((char *)&x)[0]) ? '<' : '>';
}

char cnpy::map_type(const std::type_info & t)
{
  if (t == typeid(float) ) {return 'f';}
  if (t == typeid(double) ) {return 'f';}
  if (t == typeid(long double) ) {return 'f';}

  if (t == typeid(int) ) {return 'i';}
  if (t == typeid(char) ) {return 'i';}
  if (t == typeid(short) ) {return 'i';}
  if (t == typeid(long) ) {return 'i';}
  if (t == typeid(long long) ) {return 'i';}

  if (t == typeid(unsigned char) ) {return 'u';}
  if (t == typeid(unsigned short) ) {return 'u';}
  if (t == typeid(unsigned long) ) {return 'u';}
  if (t == typeid(unsigned long long) ) {return 'u';}
  if (t == typeid(unsigned int) ) {return 'u';}

  if (t == typeid(bool) ) {return 'b';}

  if (t == typeid(std::complex<float>) ) {return 'c';}
  if (t == typeid(std::complex<double>) ) {return 'c';}
  if (t == typeid(std::complex<long double>) ) {return 'c';} else {return '?';}
}

template<> std::vector<char> & cnpy::operator+=(std::vector<char> & lhs, const std::string rhs)
{
  lhs.insert(lhs.end(), rhs.begin(), rhs.end());
  return lhs;
}

template<> std::vector<char> & cnpy::operator+=(std::vector<char> & lhs, const char * rhs)
{
  //write in little endian
  size_t len = strlen(rhs);
  lhs.reserve(len);
  for (size_t byte = 0; byte < len; byte++) {
    lhs.push_back(rhs[byte]);
  }
  return lhs;
}

void cnpy::parse_npy_header(
  unsigned char * buffer, size_t & word_size, std::vector<size_t> & shape,
  bool & fortran_order)
{
  //std::string magic_string(buffer,6);
  uint8_t major_version = *reinterpret_cast<uint8_t *>(buffer + 6);
  uint8_t minor_version = *reinterpret_cast<uint8_t *>(buffer + 7);
  (void)major_version;
  (void)minor_version;
  uint16_t header_len = *reinterpret_cast<uint16_t *>(buffer + 8);
  std::string header(reinterpret_cast<char *>(buffer + 9), header_len);

  size_t loc1, loc2;

  //fortran order
  loc1 = header.find("fortran_order") + 16;
  fortran_order = (header.substr(loc1, 4) == "True" ? true : false);

  //shape
  loc1 = header.find("(");
  loc2 = header.find(")");

  std::regex num_regex("[0-9][0-9]*");
  std::smatch sm;
  shape.clear();

  std::string str_shape = header.substr(loc1 + 1, loc2 - loc1 - 1);
  while (std::regex_search(str_shape, sm, num_regex)) {
    shape.push_back(std::stoi(sm[0].str()));
    str_shape = sm.suffix().str();
  }

  //endian, word size, data type
  //byte order code | stands for not applicable.
  //not sure when this applies except for byte array
  loc1 = header.find("descr") + 9;
  bool littleEndian = (header[loc1] == '<' || header[loc1] == '|' ? true : false);
  assert(littleEndian);

  //char type = header[loc1+1];
  //assert(type == map_type(T));

  std::string str_ws = header.substr(loc1 + 2);
  loc2 = str_ws.find("'");
  word_size = atoi(str_ws.substr(0, loc2).c_str());
}

void cnpy::parse_npy_header(
  FILE * fp, size_t & word_size, std::vector<size_t> & shape,
  bool & fortran_order)
{
  char buffer[256];
  size_t res = fread(buffer, sizeof(char), 11, fp);
  if (res != 11) {
    throw std::runtime_error("parse_npy_header: failed fread");
  }
  std::string header = fgets(buffer, 256, fp);
  assert(header[header.size() - 1] == '\n');

  size_t loc1, loc2;

  //fortran order
  loc1 = header.find("fortran_order");
  if (loc1 == std::string::npos) {
    throw std::runtime_error("parse_npy_header: failed to find header keyword: 'fortran_order'");
  }
  loc1 += 16;
  fortran_order = (header.substr(loc1, 4) == "True" ? true : false);

  //shape
  loc1 = header.find("(");
  loc2 = header.find(")");
  if (loc1 == std::string::npos || loc2 == std::string::npos) {
    throw std::runtime_error("parse_npy_header: failed to find header keyword: '(' or ')'");
  }

  std::regex num_regex("[0-9][0-9]*");
  std::smatch sm;
  shape.clear();

  std::string str_shape = header.substr(loc1 + 1, loc2 - loc1 - 1);
  while (std::regex_search(str_shape, sm, num_regex)) {
    shape.push_back(std::stoi(sm[0].str()));
    str_shape = sm.suffix().str();
  }

  //endian, word size, data type
  //byte order code | stands for not applicable.
  //not sure when this applies except for byte array
  loc1 = header.find("descr");
  if (loc1 == std::string::npos) {
    throw std::runtime_error("parse_npy_header: failed to find header keyword: 'descr'");
  }
  loc1 += 9;
  bool littleEndian = (header[loc1] == '<' || header[loc1] == '|' ? true : false);
  assert(littleEndian);

  //char type = header[loc1+1];
  //assert(type == map_type(T));

  std::string str_ws = header.substr(loc1 + 2);
  loc2 = str_ws.find("'");
  word_size = atoi(str_ws.substr(0, loc2).c_str());
}

void cnpy::parse_zip_footer(
  FILE * fp, uint16_t & nrecs, size_t & global_header_size,
  size_t & global_header_offset)
{
  std::vector<char> footer(22);
  fseek(fp, -22, SEEK_END);
  size_t res = fread(&footer[0], sizeof(char), 22, fp);
  if (res != 22) {
    throw std::runtime_error("parse_zip_footer: failed fread");
  }

  uint16_t disk_no, disk_start, nrecs_on_disk, comment_len;
  disk_no = *(uint16_t *) &footer[4];
  disk_start = *(uint16_t *) &footer[6];
  nrecs_on_disk = *(uint16_t *) &footer[8];
  nrecs = *(uint16_t *) &footer[10];
  global_header_size = *(uint32_t *) &footer[12];
  global_header_offset = *(uint32_t *) &footer[16];
  comment_len = *(uint16_t *) &footer[20];

  assert(disk_no == 0);
  assert(disk_start == 0);
  assert(nrecs_on_disk == nrecs);
  assert(comment_len == 0);
}

cnpy::NpyArray load_the_npy_file(FILE * fp)
{
  std::vector<size_t> shape;
  size_t word_size;
  bool fortran_order;
  cnpy::parse_npy_header(fp, word_size, shape, fortran_order);

  cnpy::NpyArray arr(shape, word_size, fortran_order);
  size_t nread = fread(arr.data<char>(), 1, arr.num_bytes(), fp);
  if (nread != arr.num_bytes()) {
    throw std::runtime_error("load_the_npy_file: failed fread");
  }
  return arr;
}

cnpy::NpyArray load_the_npz_array(FILE * fp, size_t compr_bytes, size_t uncompr_bytes)
{
  if (
    compr_bytes > static_cast<size_t>(std::numeric_limits<uInt>::max()) ||
    uncompr_bytes > static_cast<size_t>(std::numeric_limits<uInt>::max()))
  {
    throw std::runtime_error("npz_load: compressed ZIP member exceeds the zlib buffer limit");
  }

  std::vector<unsigned char> buffer_compr(compr_bytes);
  std::vector<unsigned char> buffer_uncompr(uncompr_bytes);
  size_t nread = fread(buffer_compr.data(), 1, compr_bytes, fp);
  if (nread != compr_bytes) {
    throw std::runtime_error("load_the_npy_file: failed fread");
  }

  int err;
  z_stream d_stream;

  d_stream.zalloc = Z_NULL;
  d_stream.zfree = Z_NULL;
  d_stream.opaque = Z_NULL;
  d_stream.avail_in = 0;
  d_stream.next_in = Z_NULL;
  err = inflateInit2(&d_stream, -MAX_WBITS);
  if (err != Z_OK) {
    throw std::runtime_error("npz_load: failed to initialize zlib inflater");
  }

  d_stream.avail_in = static_cast<uInt>(compr_bytes);
  d_stream.next_in = buffer_compr.data();
  d_stream.avail_out = static_cast<uInt>(uncompr_bytes);
  d_stream.next_out = buffer_uncompr.data();

  err = inflate(&d_stream, Z_FINISH);
  const uLong inflated_bytes = d_stream.total_out;
  inflateEnd(&d_stream);
  if (err != Z_STREAM_END || inflated_bytes != uncompr_bytes) {
    throw std::runtime_error("npz_load: failed to inflate complete ZIP member");
  }

  std::vector<size_t> shape;
  size_t word_size;
  bool fortran_order;
  cnpy::parse_npy_header(buffer_uncompr.data(), word_size, shape, fortran_order);

  cnpy::NpyArray array(shape, word_size, fortran_order);

  if (array.num_bytes() > uncompr_bytes) {
    throw std::runtime_error("npz_load: NPY payload exceeds the ZIP member size");
  }
  size_t offset = uncompr_bytes - array.num_bytes();
  memcpy(array.data<unsigned char>(), buffer_uncompr.data() + offset, array.num_bytes());

  return array;
}

cnpy::npz_t cnpy::npz_load(std::string fname)
{
  FILE * fp = fopen(fname.c_str(), "rb");

  if (!fp) {
    throw std::runtime_error("npz_load: Error! Unable to open file " + fname + "!");
  }

  cnpy::npz_t arrays;

  while (1) {
    std::vector<char> local_header(30);
    size_t headerres = fread(&local_header[0], sizeof(char), 30, fp);
    if (headerres != 30) {
      throw std::runtime_error("npz_load: failed fread");
    }

    //if we've reached the global header, stop reading
    if (local_header[2] != 0x03 || local_header[3] != 0x04) {break;}

    //read in the variable name
    uint16_t name_len = *(uint16_t *) &local_header[26];
    std::string varname(name_len, ' ');
    size_t vname_res = fread(&varname[0], sizeof(char), name_len, fp);
    if (vname_res != name_len) {
      throw std::runtime_error("npz_load: failed fread");
    }

    //erase the lagging .npy
    varname.erase(varname.end() - 4, varname.end());

    //read in the extra field
    uint16_t extra_field_len = *(uint16_t *) &local_header[28];
    std::vector<char> extra_field(extra_field_len);
    if (extra_field_len > 0) {
      size_t efield_res = fread(extra_field.data(), sizeof(char), extra_field_len, fp);
      if (efield_res != extra_field_len) {
        throw std::runtime_error("npz_load: failed fread");
      }
    }

    uint16_t compr_method = *reinterpret_cast<uint16_t *>(&local_header[0] + 8);
    const auto sizes = read_zip_entry_sizes(local_header, extra_field);

    if (compr_method == 0) {arrays[varname] = load_the_npy_file(fp);} else {
      if (compr_method != 8) {
        throw std::runtime_error("npz_load: unsupported ZIP compression method");
      }
      arrays[varname] = load_the_npz_array(fp, sizes.compressed, sizes.uncompressed);
    }
  }

  fclose(fp);
  return arrays;
}

cnpy::NpyArray cnpy::npz_load(std::string fname, std::string varname)
{
  FILE * fp = fopen(fname.c_str(), "rb");

  if (!fp) {throw std::runtime_error("npz_load: Unable to open file " + fname);}

  while (1) {
    std::vector<char> local_header(30);
    size_t header_res = fread(&local_header[0], sizeof(char), 30, fp);
    if (header_res != 30) {
      throw std::runtime_error("npz_load: failed fread");
    }

    //if we've reached the global header, stop reading
    if (local_header[2] != 0x03 || local_header[3] != 0x04) {break;}

    //read in the variable name
    uint16_t name_len = *(uint16_t *) &local_header[26];
    std::string vname(name_len, ' ');
    size_t vname_res = fread(&vname[0], sizeof(char), name_len, fp);
    if (vname_res != name_len) {
      throw std::runtime_error("npz_load: failed fread");
    }
    vname.erase(vname.end() - 4, vname.end());  //erase the lagging .npy

    //read in the extra field
    uint16_t extra_field_len = *(uint16_t *) &local_header[28];
    std::vector<char> extra_field(extra_field_len);
    if (extra_field_len > 0) {
      const size_t extra_result = fread(extra_field.data(), sizeof(char), extra_field_len, fp);
      if (extra_result != extra_field_len) {
        throw std::runtime_error("npz_load: failed fread");
      }
    }

    uint16_t compr_method = *reinterpret_cast<uint16_t *>(&local_header[0] + 8);
    const auto sizes = read_zip_entry_sizes(local_header, extra_field);

    if (vname == varname) {
      if (compr_method != 0 && compr_method != 8) {
        throw std::runtime_error("npz_load: unsupported ZIP compression method");
      }
      NpyArray array = (compr_method == 0) ? load_the_npy_file(fp) : load_the_npz_array(
        fp,
        sizes.compressed,
        sizes.uncompressed);
      fclose(fp);
      return array;
    } else {
      //skip past the data
      seek_forward(fp, sizes.compressed);
    }
  }

  fclose(fp);

  //if we get here, we haven't found the variable in the file
  throw std::runtime_error("npz_load: Variable name " + varname + " not found in " + fname);
}

cnpy::NpyArray cnpy::npy_load(std::string fname)
{

  FILE * fp = fopen(fname.c_str(), "rb");

  if (!fp) {throw std::runtime_error("npy_load: Unable to open file " + fname);}

  NpyArray arr = load_the_npy_file(fp);

  fclose(fp);
  return arr;
}
