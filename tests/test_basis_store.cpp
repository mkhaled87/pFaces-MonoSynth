#include "basis_store.h"

#include <cassert>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

using mono_synth::BasisStore;
using mono_synth::GridIndex;
using mono_synth::Height;

namespace {

using Point = BasisStore::Point;

std::set<Point> lowerClosure(const std::vector<Point>& basis) {
  std::set<Point> result;
  if (basis.empty()) return result;
  Point point(basis.front().size(), 1);
  const auto visit = [&](const auto& self, std::size_t d) -> void {
    if (d == point.size()) {
      for (const Point& generator : basis) {
        bool covered = true;
        for (std::size_t k = 0; k < point.size(); ++k) {
          if (point[k] > generator[k]) covered = false;
        }
        if (covered) {
          result.insert(point);
          break;
        }
      }
      return;
    }
    Height maximum = 1;
    for (const Point& generator : basis) maximum = std::max(maximum, generator[d]);
    for (Height value = 1; value <= maximum; ++value) {
      point[d] = value;
      self(self, d + 1);
    }
  };
  visit(visit, 0);
  return result;
}

void expectThrow(const std::function<void()>& function) {
  bool thrown = false;
  try {
    function();
  } catch (const std::exception&) {
    thrown = true;
  }
  assert(thrown);
}

void checkCanonical(const BasisStore& basis) {
  const auto& points = basis.coordinates();
  assert(std::is_sorted(points.begin(), points.end()));
  assert(std::adjacent_find(points.begin(), points.end()) == points.end());
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = 0; j < points.size(); ++j) {
      if (i == j) continue;
      bool i_below_j = true;
      for (std::size_t d = 0; d < points[i].size(); ++d) {
        i_below_j = i_below_j && points[i][d] <= points[j][d];
      }
      assert(!i_below_j);
    }
  }
}

}  // namespace

int main() {
  {
    BasisStore basis({3, 3}, {{3, 3}});
    const std::set<Point> before = lowerClosure(basis.coordinates());
    const Point removed = basis.coordinates()[0];
    basis.erase_one_and_expand(0);
    std::set<Point> expected = before;
    expected.erase(removed);
    assert(lowerClosure(basis.coordinates()) == expected);
    checkCanonical(basis);
  }

  {
    BasisStore basis({4, 4}, {{2, 4}, {4, 2}});
    const std::set<Point> before = lowerClosure(basis.coordinates());
    const auto removed_points = basis.coordinates();
    basis.erase_batch_and_expand({0, 1});
    std::set<Point> expected = before;
    expected.erase(removed_points[0]);
    expected.erase(removed_points[1]);
    assert(lowerClosure(basis.coordinates()) == expected);
    checkCanonical(basis);
  }

  {
    BasisStore basis({4, 4}, {{2, 2}, {2, 2}, {1, 1}, {4, 1}});
    assert((basis.coordinates() == std::vector<Point>{{2, 2}, {4, 1}}));
    checkCanonical(basis);
    assert(basis.contains_in_lower_closure({3, 1}));
    assert(basis.contains_in_lower_closure({4, 1}));
    assert(!basis.contains_in_lower_closure({3, 2}));
  }

  {
    BasisStore basis({3, 4}, {{2, 4}, {3, 2}});
    const std::vector<Height> threshold = basis.to_threshold(1, {3, 4});
    assert((threshold == std::vector<Height>{4, 4, 2}));
    const std::vector<Height> threshold0 = basis.to_threshold(0, {3, 4});
    assert((threshold0 == std::vector<Height>{3, 3, 2, 2}));
  }

  {
    BasisStore basis(1);
    basis.initialize_box({2, 2});
    expectThrow([&]() { basis.erase_one_and_expand(0); });
  }

  {
    const GridIndex large = std::numeric_limits<Height>::max();
    BasisStore basis({large, 2}, {{1, 1}});
    assert(basis.flat_indices()[0] == 0);
    BasisStore high({large, 2}, {{1, 2}});
    assert(high.flat_indices()[0] == large);
    expectThrow([]() {
      (void)BasisStore::checked_grid_size(
          {std::numeric_limits<GridIndex>::max(), 2});
    });
    expectThrow([]() {
      BasisStore too_wide({static_cast<GridIndex>(
                               std::numeric_limits<Height>::max()) + 1},
                          {{1}});
    });
  }

  {
    BasisStore a({5, 5}, {{5, 2}, {2, 5}});
    BasisStore b({5, 5}, {{2, 5}, {5, 2}});
    assert(a == b);
    assert(a.deterministic_hash() == b.deterministic_hash());
  }

  {
    const std::vector<GridIndex> widths{5, 5, 5};
    const std::vector<Point> points{{2, 5, 3}, {4, 2, 4}, {5, 1, 1}};
    BasisStore canonical(widths, points, 16);
    BasisStore trusted(16);
    trusted.assign_antichain(widths, {points[2], points[0], points[1]});
    assert(trusted == canonical);
    assert(trusted.to_threshold(0, widths) ==
           canonical.to_threshold(0, widths));
  }

  // Exhaust every lower set on a 3x3 grid.  This exercises the optimized
  // immediate and batch mutation paths against explicit-set semantics.
  {
    const std::vector<Point> universe = {
        {1, 1}, {1, 2}, {1, 3}, {2, 1}, {2, 2},
        {2, 3}, {3, 1}, {3, 2}, {3, 3}};
    for (std::uint32_t mask = 1; mask < (1u << universe.size()); ++mask) {
      std::vector<Point> generators;
      for (std::size_t i = 0; i < universe.size(); ++i) {
        if (mask & (1u << i)) generators.push_back(universe[i]);
      }
      BasisStore canonical({3, 3}, generators);
      checkCanonical(canonical);
      const std::set<Point> before = lowerClosure(canonical.coordinates());

      for (std::size_t i = 0; i < canonical.size(); ++i) {
        BasisStore mutated = canonical;
        const Point removed = canonical.coordinates()[i];
        mutated.erase_one_and_expand(i);
        std::set<Point> expected = before;
        expected.erase(removed);
        assert(lowerClosure(mutated.coordinates()) == expected);
        checkCanonical(mutated);
      }

      const std::size_t batch_count = std::size_t{1} << canonical.size();
      for (std::size_t batch = 1; batch < batch_count; ++batch) {
        std::vector<std::size_t> indices;
        std::set<Point> expected = before;
        for (std::size_t i = 0; i < canonical.size(); ++i) {
          if (batch & (std::size_t{1} << i)) {
            indices.push_back(i);
            expected.erase(canonical.coordinates()[i]);
          }
        }
        BasisStore mutated = canonical;
        mutated.erase_batch_and_expand(indices);
        assert(lowerClosure(mutated.coordinates()) == expected);
        checkCanonical(mutated);
      }

      for (std::size_t d_star = 0; d_star < 2; ++d_star) {
        const auto table = canonical.to_threshold(d_star, {3, 3});
        const auto closure = lowerClosure(canonical.coordinates());
        for (const Point& point : universe) {
          const std::size_t key = static_cast<std::size_t>(
              point[1 - d_star] - 1);
          assert((point[d_star] <= table[key]) ==
                 (closure.count(point) != 0));
        }
      }
    }
  }

  std::cout << "BasisStore tests passed\n";
  return 0;
}
