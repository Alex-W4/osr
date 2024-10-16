#include <filesystem>
#include <thread>
#include <vector>

#include "boost/asio/executor_work_guard.hpp"
#include "cista/serialization.h"

#include "fmt/core.h"
#include "fmt/std.h"

#include "conf/options_parser.h"

#include "osr/lookup.h"
#include "osr/ways.h"
#include "rtree.h"

// push in own fork for error
namespace fs = std::filesystem;
using namespace osr;

class settings : public conf::configuration {
public:
  explicit settings() : configuration("Options") {
    param(data_dir_, "data,d", "Data directory");
    param(static_file_path_, "static,s", "Path to static files (ui/web)");
    param(threads_, "threads,t", "Number of routing threads");
    param(lock_, "lock,l", "Lock to memory");
  }

  fs::path data_dir_{"osr"};
  std::string static_file_path_;
  bool lock_{true};
  unsigned threads_{std::thread::hardware_concurrency()};
};

struct search_context {
  void *target;
  void *found_data;
  bool found;
  int(*compare)(const void *a, const void *b);
};

bool search_iter(const double *min, const double *max, const void *data, void *udata) {
  (void) min;
  (void) max;
  auto *ctx = (search_context*)udata;
  if ((ctx->compare && ctx->compare(data, ctx->target) == 0) || data == ctx->target) {
    assert(!ctx->found);
    ctx->found_data = (void*) data;
    ctx->found = true;
  }
  return true;
};

int main(int argc, char const* argv[]) {
  auto opt = settings{};
  auto parser = conf::options_parser({&opt});
  parser.read_command_line_args(argc, argv);

  if (parser.help()) {
    parser.print_help(std::cout);
    return 0;
  } else if (parser.version()) {
    return 0;
  }

  parser.read_configuration_file();
  parser.print_unrecognized(std::cout);
  //parser.print_used(std::cout);

  if (!fs::is_directory(opt.data_dir_)) {
    fmt::println("directory not found: {}", opt.data_dir_);
    return 1;
  }

  auto const w = ways{opt.data_dir_, cista::mmap::protection::READ};

  std::cout << "Start Benchmark" << std::endl;

  rtree* rtree_ = rtree_new();
  cista::rtree<size_t, 2, double, 64, uint32_t, cista::offset::vector_map> cpp_rtree_;

  cista::byte_buf buf;
  {
    cista::rtree<size_t, 2, double, 64, uint32_t, cista::offset::vector_map> cpp_mm_rtree_;
    buf = cista::serialize(cpp_mm_rtree_);
  }
  auto const cpp_ser_rtree_ = cista::deserialize<cista::rtree<size_t, 2, double, 64, uint32_t, cista::offset::vector_map>>(buf);

  using rtree_t = cista::rtree<size_t, 2, double, 64, uint32_t, cista::mmap_vec_map>;
  auto f = cista::mmap{"./mmap.bin", cista::mmap::protection::WRITE};
  //rtree_t cpp_mm_rtree_{.nodes_ = cista::mmap_vec_map<rtree_t::node_idx_t, rtree_t::node>{std::move(f)}};
  rtree_t cpp_mm_rtree_{rtree_t::rect(), rtree_t::node_idx_t::invalid(), rtree_t::node_idx_t::invalid(), 0, 0, {}, cista::mmap_vec_map<rtree_t::node_idx_t, rtree_t::node>{std::move(f)}};

  ways const& ways_ = w;


  auto start = std::chrono::high_resolution_clock::now();

  for (auto i = way_idx_t{0U}; i != w.n_ways(); ++i) {
    auto b = geo::box{};
    for (auto const& c : ways_.way_polylines_[i]) {
      b.extend(c);
    }

    auto const min = b.min_.lnglat();
    auto const max = b.max_.lnglat();
    rtree_insert(rtree_, min.data(), max.data(),
                 // NOLINTNEXTLINE(performance-no-int-to-ptr)
                 reinterpret_cast<void*>(static_cast<std::size_t>(to_idx(i))));
  }
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = duration_cast<std::chrono::milliseconds>(stop - start);

  std::cout << "Benchmark: insert " << rtree_count(rtree_) << " data points:" << std::endl;
  std::cout << std::endl;

  std::cout << "Insert reference time: " << duration.count() << " milliseconds" << std::endl;


  auto start_cpp = std::chrono::high_resolution_clock::now();

  for (auto i = way_idx_t{0U}; i != w.n_ways(); ++i) {
    auto b = geo::box{};
    for (auto const& c : ways_.way_polylines_[i]) {
      b.extend(c);
    }

    auto const min = b.min_.lnglat();
    auto const max = b.max_.lnglat();
    cpp_rtree_.insert(min, max, static_cast<std::size_t>(to_idx(i)));
  }

  auto stop_cpp = std::chrono::high_resolution_clock::now();
  auto duration_cpp = duration_cast<std::chrono::milliseconds>(stop_cpp - start_cpp);

  std::cout << "Insert C++ time: " << duration_cpp.count() << " milliseconds" << std::endl;

  auto start_cpp_mm = std::chrono::high_resolution_clock::now();

  for (auto i = way_idx_t{0U}; i != w.n_ways(); ++i) {
    auto b = geo::box{};
    for (auto const& c : ways_.way_polylines_[i]) {
      b.extend(c);
    }

    auto const min = b.min_.lnglat();
    auto const max = b.max_.lnglat();
    cpp_mm_rtree_.insert(min, max, static_cast<std::size_t>(to_idx(i)));
  }

  auto stop_cpp_mm = std::chrono::high_resolution_clock::now();
  auto duration_cpp_mm = duration_cast<std::chrono::milliseconds>(stop_cpp_mm - start_cpp_mm);

  std::cout << "Insert mm time: " << duration_cpp_mm.count() << " milliseconds" << std::endl;

  auto start_cpp_ser = std::chrono::high_resolution_clock::now();

  for (auto i = way_idx_t{0U}; i != w.n_ways(); ++i) {
    auto b = geo::box{};
    for (auto const& c : ways_.way_polylines_[i]) {
      b.extend(c);
    }

    auto const min = b.min_.lnglat();
    auto const max = b.max_.lnglat();
    cpp_ser_rtree_->insert(min, max, static_cast<std::size_t>(to_idx(i)));
  }

  auto stop_cpp_ser = std::chrono::high_resolution_clock::now();
  auto duration_cpp_ser = duration_cast<std::chrono::milliseconds>(stop_cpp_ser - start_cpp_ser);

  std::cout << "Insert serial time: " << duration_cpp_ser.count() << " milliseconds" << std::endl;


  std::cout << std::endl;
  std::cout << std::endl;

  // Search benchmark
  auto N = 100;
  std::cout << "Benchmark: Search for " << N << ", " << N * 10 << ", " << N * 100 << ", " << N * 1000<< ", " << " data points:" << std::endl;
  std::cout << std::endl;
  for (int n = 1; n < 1001; n = n * 10) {
    auto start_search = std::chrono::high_resolution_clock::now();
    bool all_found = true;
    for (auto i = way_idx_t{0U}; i != w.n_ways() && i < N * n; ++i) {
      auto b = geo::box{};
      for (auto const& c : ways_.way_polylines_[i]) {
        b.extend(c);
      }

      auto const min_temp = b.min_.lnglat();
      auto const max_temp = b.max_.lnglat();
      const double *min = min_temp.data();
      const double *max = max_temp.data();
      bool found = false;
      search_context ctx = {.target = reinterpret_cast<void*>(static_cast<std::size_t>(to_idx(i))), .found_data= nullptr, .found= false, .compare = nullptr};
      rtree_search(rtree_, min, max, search_iter, &ctx);
      if (ctx.found) {
        found = true;
      }

      all_found = all_found && found;
    }
    auto stop_search = std::chrono::high_resolution_clock::now();
    auto duration_search = duration_cast<std::chrono::milliseconds>(stop_search - start_search);

    std::cout << "Search ref time for " << N * n << " points: " << duration_search.count() << " milliseconds, correctness: " << (all_found ? "true" : "false") << std::endl;
  }


  // Search cpp benchmark
  for (int n = 1; n < 1001; n = n * 10) {
    auto start_cpp_search = std::chrono::high_resolution_clock::now();
    bool all_cpp_found = true;
    for (auto i = way_idx_t{0U}; i != w.n_ways() && i < N * n; ++i) {

      auto b = geo::box{};
      for (auto const& c : ways_.way_polylines_[i]) {
        b.extend(c);
      }

      auto const min = b.min_.lnglat();
      auto const max = b.max_.lnglat();
      bool found = false;
      cpp_rtree_.search(min, max, [min, max, i, &found](cista::rtree<size_t, 2, double>::coord_t const &min_temp, cista::rtree<size_t, 2, double>::coord_t const &max_temp, size_t way_temp) {
        //std::cout << static_cast<std::size_t>(to_idx(way)) << ", " << way_temp << std::endl;
        if (static_cast<std::size_t>(to_idx(i)) == way_temp && cista::rtree<size_t, 2, double>::rect::coord_t_equal(min, min_temp) && cista::rtree<size_t, 2, double>::rect::coord_t_equal(max, max_temp)) {
          found = true;
        }
        return true;
      });
      all_cpp_found = all_cpp_found && found;
    }
    auto stop_cpp_search = std::chrono::high_resolution_clock::now();
    auto duration_cpp_search = duration_cast<std::chrono::milliseconds>(stop_cpp_search - start_cpp_search);

    std::cout << "Search C++ time for " << N * n << " points: " << duration_cpp_search.count() << " milliseconds, correctness: " << (all_cpp_found ? "true" : "false") << std::endl;
  }

  // Search cpp_mm benchmark
  for (int n = 1; n < 1001; n = n * 10) {
    auto start_cpp_mm_search = std::chrono::high_resolution_clock::now();
    bool all_cpp_found = true;
    for (auto i = way_idx_t{0U}; i != w.n_ways() && i < N * n; ++i) {

      auto b = geo::box{};
      for (auto const& c : ways_.way_polylines_[i]) {
        b.extend(c);
      }

      auto const min = b.min_.lnglat();
      auto const max = b.max_.lnglat();
      bool found = false;
      /*
      cpp_mm_rtree_.search(min, max, [min, max, i, &found](cista::rtree<size_t, 2, double>::coord_t const &min_temp, cista::rtree<size_t, 2, double>::coord_t const &max_temp, size_t way_temp) {
        //std::cout << static_cast<std::size_t>(to_idx(way)) << ", " << way_temp << std::endl;
        if (static_cast<std::size_t>(to_idx(i)) == way_temp && cista::rtree<size_t, 2, double>::rect::coord_t_equal(min, min_temp) && cista::rtree<size_t, 2, double>::rect::coord_t_equal(max, max_temp)) {
          found = true;
        }
        return true;
      });*/
      all_cpp_found = all_cpp_found && found;
    }
    auto stop_cpp_mm_search = std::chrono::high_resolution_clock::now();
    auto duration_cpp_mm_search = duration_cast<std::chrono::milliseconds>(stop_cpp_mm_search - start_cpp_mm_search);

    std::cout << "Search mm C++ time for " << N * n << " points: " << duration_cpp_mm_search.count() << " milliseconds, correctness: " << (all_cpp_found ? "true" : "false") << std::endl;
  }

  // Search cpp_serialized benchmark
  for (int n = 1; n < 1001; n = n * 10) {
    auto start_cpp_mm_search = std::chrono::high_resolution_clock::now();
    bool all_cpp_found = true;
    for (auto i = way_idx_t{0U}; i != w.n_ways() && i < N * n; ++i) {

      auto b = geo::box{};
      for (auto const& c : ways_.way_polylines_[i]) {
        b.extend(c);
      }

      auto const min = b.min_.lnglat();
      auto const max = b.max_.lnglat();
      bool found = false;
      cpp_ser_rtree_->search(min, max, [min, max, i, &found](cista::rtree<size_t, 2, double>::coord_t const &min_temp, cista::rtree<size_t, 2, double>::coord_t const &max_temp, size_t way_temp) {
        //std::cout << static_cast<std::size_t>(to_idx(way)) << ", " << way_temp << std::endl;
        if (static_cast<std::size_t>(to_idx(i)) == way_temp && cista::rtree<size_t, 2, double>::rect::coord_t_equal(min, min_temp) && cista::rtree<size_t, 2, double>::rect::coord_t_equal(max, max_temp)) {
          found = true;
        }
        return true;
      });
      all_cpp_found = all_cpp_found && found;
    }
    auto stop_cpp_mm_search = std::chrono::high_resolution_clock::now();
    auto duration_cpp_mm_search = duration_cast<std::chrono::milliseconds>(stop_cpp_mm_search - start_cpp_mm_search);

    std::cout << "Search serial C++ time for " << N * n << " points: " << duration_cpp_mm_search.count() << " milliseconds, correctness: " << (all_cpp_found ? "true" : "false") << std::endl;
  }

  //auto const l = lookup{w};
}