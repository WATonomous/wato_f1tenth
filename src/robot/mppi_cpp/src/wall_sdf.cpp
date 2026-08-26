#include "mppi_cpp/wall_sdf.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace mppi {
namespace {

// Minimal PGM (P5 binary / P2 ascii) reader -> 8-bit grayscale, row 0 = top.
std::vector<uint8_t> read_pgm(const std::string& path, int& w, int& h) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot open map image: " + path);
  auto next_token = [&in]() {
    std::string tok;
    while (in >> tok) {
      if (tok[0] == '#') { std::string rest; std::getline(in, rest); continue; }
      return tok;
    }
    throw std::runtime_error("truncated PGM header");
  };
  const std::string magic = next_token();
  w = std::stoi(next_token());
  h = std::stoi(next_token());
  const int maxval = std::stoi(next_token());
  std::vector<uint8_t> img(static_cast<size_t>(w) * h);
  if (magic == "P5") {
    in.get();  // single whitespace after maxval
    if (maxval < 256) {
      in.read(reinterpret_cast<char*>(img.data()), img.size());
    } else {
      for (auto& px : img) { uint8_t b[2]; in.read(reinterpret_cast<char*>(b), 2); px = b[0]; }
    }
  } else if (magic == "P2") {
    for (auto& px : img) px = static_cast<uint8_t>(std::stoi(next_token()) * 255 / maxval);
  } else {
    throw std::runtime_error("unsupported map image format (need P2/P5 PGM): " + path);
  }
  if (!in) throw std::runtime_error("truncated PGM data: " + path);
  return img;
}

// Felzenszwalb & Huttenlocher 1D squared distance transform.
void edt_1d(const double* f, double* d, int n, std::vector<int>& v, std::vector<double>& z) {
  constexpr double inf = std::numeric_limits<double>::infinity();
  int k = 0;
  v[0] = 0; z[0] = -inf; z[1] = inf;
  auto sect = [&](int q, int p) {
    return ((f[q] + double(q) * q) - (f[p] + double(p) * p)) / (2.0 * q - 2.0 * p);
  };
  for (int q = 1; q < n; ++q) {
    double s = sect(q, v[k]);
    while (s <= z[k]) { --k; s = sect(q, v[k]); }
    ++k; v[k] = q; z[k] = s; z[k + 1] = inf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    d[q] = double(q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

}  // namespace

WallSdf WallSdf::load(const std::string& map_yaml) {
  const std::filesystem::path yaml_path = std::filesystem::absolute(map_yaml);
  const YAML::Node cfg = YAML::LoadFile(yaml_path.string());
  std::filesystem::path image = cfg["image"].as<std::string>();
  if (image.is_relative()) image = yaml_path.parent_path() / image;

  WallSdf sdf;
  int w, h;
  const std::vector<uint8_t> img = read_pgm(image.string(), w, h);
  sdf.w = w; sdf.h = h;
  sdf.res = cfg["resolution"].as<float>();
  sdf.ox = cfg["origin"][0].as<float>();
  sdf.oy = cfg["origin"][1].as<float>();
  const bool negate = cfg["negate"] ? cfg["negate"].as<int>() != 0 : false;
  const double occ_thresh = cfg["occupied_thresh"] ? cfg["occupied_thresh"].as<double>() : 0.65;

  // f = 0 on occupied cells, +big elsewhere; flip rows so row 0 is the map origin.
  constexpr double big = 1e12;
  std::vector<double> f(static_cast<size_t>(w) * h);
  for (int r = 0; r < h; ++r)
    for (int c = 0; c < w; ++c) {
      const double px = img[static_cast<size_t>(h - 1 - r) * w + c];
      const double occ = negate ? px / 255.0 : (255.0 - px) / 255.0;
      f[static_cast<size_t>(r) * w + c] = occ >= occ_thresh ? 0.0 : big;
    }

  const int n = std::max(w, h);
  std::vector<double> col_in(h), col_out(h), row_in(w), row_out(w), z(n + 1);
  std::vector<int> v(n);
  for (int c = 0; c < w; ++c) {
    for (int r = 0; r < h; ++r) col_in[r] = f[static_cast<size_t>(r) * w + c];
    edt_1d(col_in.data(), col_out.data(), h, v, z);
    for (int r = 0; r < h; ++r) f[static_cast<size_t>(r) * w + c] = col_out[r];
  }
  sdf.data.resize(f.size());
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) row_in[c] = f[static_cast<size_t>(r) * w + c];
    edt_1d(row_in.data(), row_out.data(), w, v, z);
    for (int c = 0; c < w; ++c)
      sdf.data[static_cast<size_t>(r) * w + c] = static_cast<float>(std::sqrt(row_out[c]) * sdf.res);
  }
  return sdf;
}

float WallSdf::sample(float x, float y) const {
  if (!valid()) return 100.f;
  const int col = static_cast<int>(std::floor((x - ox) / res));
  const int row = static_cast<int>(std::floor((y - oy) / res));
  if (row < 0 || row >= h || col < 0 || col >= w) return 0.f;
  return data[static_cast<size_t>(row) * w + col];
}

}  // namespace mppi
