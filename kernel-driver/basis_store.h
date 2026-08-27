#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace mono_synth {

using GridIndex = std::uint64_t;
using TableOffset = std::uint64_t;
using Height = std::uint32_t;

inline GridIndex checkedMultiply(GridIndex lhs, GridIndex rhs,
                                 const char* what = "grid product") {
  if (rhs != 0 && lhs > std::numeric_limits<GridIndex>::max() / rhs) {
    throw std::overflow_error(std::string(what) + " overflows uint64_t");
  }
  return lhs * rhs;
}

/** Exact antichain representation of a finite lower set.
 *
 * Coordinates are one-based and use the internal monotone order: p <= q iff
 * p[d] <= q[d] in every dimension.  The stored vectors are always unique,
 * pairwise incomparable, and lexicographically sorted.
 */
class BasisStore {
 public:
  using Point = std::vector<Height>;

  explicit BasisStore(
      std::size_t capacity = std::numeric_limits<std::size_t>::max())
      : capacity_(capacity) {}

  BasisStore(const std::vector<GridIndex>& widths,
             const std::vector<Point>& points,
             std::size_t capacity = std::numeric_limits<std::size_t>::max())
      : widths_(widths), capacity_(capacity), points_(points) {
    validateWidths();
    buildStrides();
    canonicalize();
  }

  void initialize_box(const std::vector<GridIndex>& widths) {
    widths_ = widths;
    validateWidths();
    buildStrides();
    points_.assign(1, Point(widths_.size()));
    for (std::size_t d = 0; d < widths_.size(); ++d) {
      if (widths_[d] > std::numeric_limits<Height>::max()) {
        throw std::overflow_error("grid width does not fit Height");
      }
      points_[0][d] = static_cast<Height>(widths_[d]);
    }
    ensureCapacity(points_.size());
    rebuildFlatIndices();
  }

  void assign(const std::vector<GridIndex>& widths,
              const std::vector<Point>& points) {
    widths_ = widths;
    validateWidths();
    buildStrides();
    points_ = points;
    canonicalize();
  }

  /** Assign points already known to be an antichain.
   *
   * This is for algorithmic states constructed as subsets of an existing
   * canonical basis. Sorting is retained for deterministic storage, while the
   * quadratic dominance elimination in canonicalize() is intentionally
   * skipped. Debug builds verify the caller's antichain proof explicitly.
   */
  void assign_antichain(const std::vector<GridIndex>& widths,
                        const std::vector<Point>& points) {
    widths_ = widths;
    validateWidths();
    buildStrides();
    points_ = points;
    for (const Point& point : points_) validatePoint(point);
    std::sort(points_.begin(), points_.end());
    if (std::adjacent_find(points_.begin(), points_.end()) != points_.end()) {
      throw std::logic_error("trusted antichain assignment contains duplicates");
    }
#ifndef NDEBUG
    for (std::size_t i = 0; i < points_.size(); ++i) {
      for (std::size_t j = i + 1; j < points_.size(); ++j) {
        if (covers(points_[i], points_[j]) || covers(points_[j], points_[i])) {
          throw std::logic_error(
              "trusted antichain assignment contains comparable points");
        }
      }
    }
#endif
    ensureCapacity(points_.size());
    rebuildFlatIndices();
  }

  void clear() {
    points_.clear();
    flat_indices_.clear();
  }

  void erase_one_and_expand(std::size_t index) {
    if (index >= points_.size()) {
      throw std::out_of_range("basis deletion index is out of range");
    }
    eraseAndExpand(std::vector<std::size_t>{index});
  }

  void erase_batch_and_expand(const std::vector<std::size_t>& indices) {
    if (indices.empty()) return;
    std::vector<std::size_t> sorted(indices);
    std::sort(sorted.begin(), sorted.end());
    if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end()) {
      throw std::invalid_argument("basis batch contains duplicate indices");
    }
    if (sorted.back() >= points_.size()) {
      throw std::out_of_range("basis batch deletion index is out of range");
    }

    eraseAndExpand(sorted);
  }

  void canonicalize() {
    for (const Point& point : points_) validatePoint(point);
    std::sort(points_.begin(), points_.end());
    points_.erase(std::unique(points_.begin(), points_.end()), points_.end());

    std::vector<Point> maximal;
    maximal.reserve(points_.size());
    for (const Point& point : points_) {
      bool covered = false;
      for (const Point& generator : maximal) {
        if (covers(generator, point)) {
          covered = true;
          break;
        }
      }
      if (covered) continue;
      maximal.erase(
          std::remove_if(maximal.begin(), maximal.end(),
                         [&point](const Point& generator) {
                           return covers(point, generator);
                         }),
          maximal.end());
      maximal.push_back(point);
    }
    std::sort(maximal.begin(), maximal.end());
    ensureCapacity(maximal.size());
    points_.swap(maximal);
    rebuildFlatIndices();
  }

  bool contains_in_lower_closure(const Point& point) const {
    validatePoint(point);
    for (const Point& generator : points_) {
      if (covers(generator, point)) return true;
    }
    return false;
  }

  std::vector<Height> to_threshold(
      std::size_t d_star, const std::vector<GridIndex>& widths) const {
    if (widths != widths_) {
      throw std::invalid_argument("threshold widths differ from basis widths");
    }
    if (d_star >= widths_.size()) {
      throw std::out_of_range("threshold axis is out of range");
    }
    TableOffset table_size = 1;
    for (std::size_t d = 0; d < widths_.size(); ++d) {
      if (d != d_star) {
        table_size = checkedMultiply(table_size, widths_[d],
                                     "threshold-table size");
      }
    }
    if (table_size > std::numeric_limits<std::size_t>::max()) {
      throw std::overflow_error("threshold table does not fit host size_t");
    }
    std::vector<Height> table(static_cast<std::size_t>(table_size), 0);

    std::vector<TableOffset> key_strides(widths_.size(), 0);
    TableOffset stride = 1;
    for (std::size_t d = 0; d < widths_.size(); ++d) {
      if (d == d_star) continue;
      key_strides[d] = stride;
      stride = checkedMultiply(stride, widths_[d], "threshold stride");
    }
    for (const Point& generator : points_) {
      TableOffset key = 0;
      for (std::size_t d = 0; d < widths_.size(); ++d) {
        if (d != d_star) {
          key += static_cast<TableOffset>(generator[d] - 1) * key_strides[d];
        }
      }
      Height& height = table[static_cast<std::size_t>(key)];
      height = std::max(height, generator[d_star]);
    }
    // Multi-dimensional suffix maximum: a key is covered by every generator
    // whose projection is componentwise above it.
    for (std::size_t d = 0; d < widths_.size(); ++d) {
      if (d == d_star) continue;
      const TableOffset dimension_stride = key_strides[d];
      const TableOffset block = checkedMultiply(
          dimension_stride, widths_[d], "threshold prefix block");
      for (GridIndex coordinate = widths_[d] - 1; coordinate > 0; --coordinate) {
        for (TableOffset outer = 0; outer < table_size; outer += block) {
          for (TableOffset inner = 0; inner < dimension_stride; ++inner) {
            const TableOffset lower =
                outer + (coordinate - 1) * dimension_stride + inner;
            const TableOffset upper = lower + dimension_stride;
            table[static_cast<std::size_t>(lower)] = std::max(
                table[static_cast<std::size_t>(lower)],
                table[static_cast<std::size_t>(upper)]);
          }
        }
      }
    }
    return table;
  }

  bool empty() const { return points_.empty(); }
  std::size_t size() const { return points_.size(); }
  const std::vector<Point>& coordinates() const { return points_; }
  const std::vector<GridIndex>& flat_indices() const { return flat_indices_; }
  const std::vector<GridIndex>& widths() const { return widths_; }

  bool operator==(const BasisStore& other) const {
    return widths_ == other.widths_ && points_ == other.points_;
  }
  bool operator!=(const BasisStore& other) const { return !(*this == other); }

  std::uint64_t deterministic_hash() const {
    // FNV-1a over dimensions, widths, and canonical coordinates.
    std::uint64_t hash = UINT64_C(1469598103934665603);
    const auto mix = [&hash](std::uint64_t value) {
      for (unsigned byte = 0; byte < 8; ++byte) {
        hash ^= static_cast<unsigned char>(value >> (8 * byte));
        hash *= UINT64_C(1099511628211);
      }
    };
    mix(widths_.size());
    for (GridIndex width : widths_) mix(width);
    mix(points_.size());
    for (const Point& point : points_) {
      for (Height coordinate : point) mix(coordinate);
    }
    return hash;
  }

  static GridIndex checked_grid_size(const std::vector<GridIndex>& widths) {
    GridIndex size = 1;
    for (GridIndex width : widths) {
      if (width == 0) throw std::invalid_argument("grid width must be positive");
      size = checkedMultiply(size, width);
    }
    return size;
  }

 private:
  static bool covers(const Point& upper, const Point& lower) {
    if (upper.size() != lower.size()) return false;
    for (std::size_t d = 0; d < upper.size(); ++d) {
      if (lower[d] > upper[d]) return false;
    }
    return true;
  }

  void validateWidths() const {
    if (widths_.empty()) throw std::invalid_argument("basis dimension is zero");
    for (GridIndex width : widths_) {
      if (width == 0) throw std::invalid_argument("grid width must be positive");
      if (width > std::numeric_limits<Height>::max()) {
        throw std::overflow_error("grid width does not fit Height");
      }
    }
    (void)checked_grid_size(widths_);
  }

  void validatePoint(const Point& point) const {
    if (point.size() != widths_.size()) {
      throw std::invalid_argument("basis point has the wrong dimension");
    }
    for (std::size_t d = 0; d < point.size(); ++d) {
      if (point[d] == 0 || point[d] > widths_[d]) {
        throw std::out_of_range("basis coordinate is outside the grid");
      }
    }
  }

  void ensureCapacity(std::size_t size) const {
    if (size > capacity_) throw std::length_error("basis capacity exceeded");
  }

  void buildStrides() {
    strides_.assign(widths_.size(), 1);
    for (std::size_t d = 1; d < widths_.size(); ++d) {
      strides_[d] = checkedMultiply(strides_[d - 1], widths_[d - 1],
                                    "grid stride");
    }
  }

  GridIndex flatten(const Point& point) const {
    validatePoint(point);
    GridIndex result = 0;
    for (std::size_t d = 0; d < point.size(); ++d) {
      const GridIndex term = checkedMultiply(
          static_cast<GridIndex>(point[d] - 1), strides_[d], "flat index");
      if (result > std::numeric_limits<GridIndex>::max() - term) {
        throw std::overflow_error("flat index overflows uint64_t");
      }
      result += term;
    }
    return result;
  }

  void rebuildFlatIndices() {
    flat_indices_.clear();
    flat_indices_.reserve(points_.size());
    for (const Point& point : points_) flat_indices_.push_back(flatten(point));
  }

  void eraseAndExpand(const std::vector<std::size_t>& sorted_indices) {
    std::vector<Point> remaining;
    std::vector<Point> removed;
    remaining.reserve(points_.size() - sorted_indices.size());
    removed.reserve(sorted_indices.size());
    std::size_t deletion = 0;
    for (std::size_t index = 0; index < points_.size(); ++index) {
      if (deletion < sorted_indices.size() &&
          index == sorted_indices[deletion]) {
        removed.push_back(points_[index]);
        ++deletion;
      } else {
        remaining.push_back(points_[index]);
      }
    }

    // The old basis is an antichain, so a lower neighbour of a removed
    // generator can never dominate a retained generator.  Only candidate
    // deduplication and dominance against retained/accepted candidates are
    // needed; this avoids re-canonicalizing the complete basis after every
    // literal CDC mutation.
    std::set<Point> candidates;
    for (const Point& point : removed) {
      for (std::size_t d = 0; d < point.size(); ++d) {
        if (point[d] <= 1) continue;
        Point neighbour = point;
        --neighbour[d];
        candidates.insert(std::move(neighbour));
      }
    }

    std::vector<Point> accepted;
    for (const Point& candidate : candidates) {
      bool covered = false;
      for (const Point& generator : remaining) {
        if (covers(generator, candidate)) {
          covered = true;
          break;
        }
      }
      if (covered) continue;
      for (const Point& generator : accepted) {
        if (covers(generator, candidate)) {
          covered = true;
          break;
        }
      }
      if (covered) continue;
      accepted.erase(
          std::remove_if(accepted.begin(), accepted.end(),
                         [&candidate](const Point& generator) {
                           return covers(candidate, generator);
                         }),
          accepted.end());
      accepted.push_back(candidate);
    }

    ensureCapacity(remaining.size() + accepted.size());
    remaining.insert(remaining.end(), accepted.begin(), accepted.end());
    std::sort(remaining.begin(), remaining.end());
    points_.swap(remaining);
    rebuildFlatIndices();
  }

  std::vector<GridIndex> widths_;
  std::vector<GridIndex> strides_;
  std::size_t capacity_;
  std::vector<Point> points_;
  std::vector<GridIndex> flat_indices_;
};

}  // namespace mono_synth
