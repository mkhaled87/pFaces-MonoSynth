/**
 * @file safe_set.h
 * @brief Bitmap-based safe set with O(1) queries and O(N^n · n) construction.
 *
 * The safe set K_* = ↓B is the downward closure of an antichain basis B
 * under the component-wise partial order induced by the monotone system's
 * priority ordering.  This module stores K_* as a dense bitmap for O(1)
 * membership queries and constructs it from B via an N-dimensional sweep
 * (prefix-OR) algorithm that runs in O(N^n · n) — linear in grid size
 * times number of dimensions — regardless of |B|.
 *
 * Memory for a 3D 81×49×61 grid: 242,109 bits ≈ 30 KB (fits in L1 cache).
 *
 * Thread safety: read-only queries are safe from any thread.  Mutation
 * (rebuild) must be externally serialized.
 */

#pragma once

#include "types.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace rt_ctrl {

class SafeSet {
public:
    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    SafeSet() = default;

    /**
     * Initialize with grid descriptor and priority ordering.
     * @param grid       Grid dimensions, bounds, eta.
     * @param priorities Per-dimension priority (0 = ↑ order, 1 = ↓ order).
     */
    void init(const GridDesc& grid, const std::vector<int>& priorities) {
        grid_       = grid;
        priorities_ = priorities;
        bitmap_.assign(grid_.total_cells, 0);
        basis_.clear();
    }

    // -----------------------------------------------------------------------
    // Basis management
    // -----------------------------------------------------------------------

    /// Set basis from flat int array (dim-interleaved): [b0_d0, b0_d1, ..., b1_d0, ...]
    void set_basis(const int* data, int count, int n_dim) {
        basis_.resize(count);
        for (int i = 0; i < count; ++i) {
            basis_[i].resize(n_dim);
            for (int d = 0; d < n_dim; ++d) {
                basis_[i][d] = data[i * n_dim + d];
            }
        }
    }

    /// Set basis from vector of vectors
    void set_basis(const std::vector<std::vector<int>>& basis) {
        basis_ = basis;
    }

    /**
     * Set bitmap directly from an external uint8 array (zero-copy from pFaces buffer).
     * Skips basis storage and bitmap construction entirely.
     * @param data   Pointer to uint8 bitmap array.
     * @param size   Number of cells (must match grid_.total_cells).
     */
    void set_bitmap_direct(const uint8_t* data, int size) {
        if (size != grid_.total_cells) {
            std::cerr << "[SafeSet] WARNING: bitmap size mismatch ("
                      << size << " vs " << grid_.total_cells << ")\n";
        }
        int n = std::min(size, grid_.total_cells);
        bitmap_.assign(data, data + n);
        // Count safe cells for statistics
        int safe = 0;
        for (auto b : bitmap_) safe += b;
        last_build_ms_ = 0.0;  // Built externally
    }

    /// Set basis size for reporting (when bitmap is set externally)
    void set_basis_size_hint(int n) { basis_size_hint_ = n; }

    /**
     * Load basis from CSV file.
     * Supports two formats:
     *   (a) pFaces evolution CSV: "iteration,idx0,idx1,..." with header.
     *       Only the LAST iteration's rows are used.
     *   (b) Plain CSV: "idx0,idx1,..." with no header (or # comments).
     */
    bool load_basis_csv(const std::string& path) {
        std::ifstream ifs(path);
        if (!ifs.is_open()) return false;

        basis_.clear();
        std::string line;
        bool has_header = false;
        bool is_evolution_format = false;
        int last_iter = -1;

        // First pass: detect format and find last iteration.
        while (std::getline(ifs, line)) {
            if (line.empty() || line[0] == '#') continue;
            // Detect header row
            if (line.find("iteration") != std::string::npos) {
                has_header = true;
                is_evolution_format = true;
                continue;
            }
            if (is_evolution_format) {
                int iter = std::stoi(line.substr(0, line.find(',')));
                if (iter > last_iter) last_iter = iter;
            }
        }

        // Second pass: extract basis elements.
        ifs.clear();
        ifs.seekg(0);

        while (std::getline(ifs, line)) {
            if (line.empty() || line[0] == '#') continue;
            if (line.find("iteration") != std::string::npos) continue; // skip header

            std::vector<int> vals;
            std::stringstream ss(line);
            std::string token;
            while (std::getline(ss, token, ',')) {
                try { vals.push_back(std::stoi(token)); }
                catch (...) { break; }
            }

            if (is_evolution_format && !vals.empty()) {
                int iter = vals[0];
                if (iter == last_iter) {
                    // Skip iteration column, keep idx0..idxN
                    std::vector<int> elem(vals.begin() + 1, vals.end());
                    if (!elem.empty()) basis_.push_back(std::move(elem));
                }
            } else if (!vals.empty()) {
                basis_.push_back(std::move(vals));
            }
        }
        return !basis_.empty();
    }

    int basis_size() const { 
        return basis_.empty() ? basis_size_hint_ : static_cast<int>(basis_.size()); 
    }
    const std::vector<std::vector<int>>& basis() const { return basis_; }

    // -----------------------------------------------------------------------
    // Bitmap construction: N-dimensional prefix-OR sweep
    // -----------------------------------------------------------------------
    /**
     * Build bitmap from current basis via sweep algorithm.
     *
     * Algorithm:
     *   1. Mark each basis element in the grid.
     *   2. For each dimension d, sweep from "high" to "low" in the
     *      monotone order, propagating marks via OR.
     *
     * After the sweep, bitmap[flat] == 1 iff the cell is dominated by
     * some basis element in ALL dimensions — i.e., the cell is in ↓B.
     *
     * Complexity: O(N^n · n) where n = number of dimensions,
     *             N^n = total grid cells.  For 3D turn example:
     *             242,109 × 3 = ~726K operations ≈ <1 ms.
     *
     * @return Build time in milliseconds.
     */
    double build_bitmap() {
        auto t0 = std::chrono::high_resolution_clock::now();

        const int n_dim = grid_.n_dim;
        const int total = grid_.total_cells;

        // Zero the bitmap
        std::fill(bitmap_.begin(), bitmap_.end(), static_cast<uint8_t>(0));

        // Step 1: Mark basis elements
        for (const auto& b : basis_) {
            int flat = grid_.flatten(b.data());
            if (flat >= 0 && flat < total) {
                bitmap_[flat] = 1;
            }
        }

        // Step 2: Prefix-OR sweep per dimension
        //
        // For each dimension d:
        //   If priority[d] == 0 (max = good), sweep HIGH → LOW:
        //     bitmap[..., x_d, ...] |= bitmap[..., x_d+1, ...]
        //   If priority[d] == 1 (min = good), sweep LOW → HIGH:
        //     bitmap[..., x_d, ...] |= bitmap[..., x_d-1, ...]
        //
        // This propagates basis marks "downward" in the partial order.
        //
        // Implementation: decompose flat index into
        //   flat = outer * (sizes[d] * stride) + coord_d * stride + inner
        // where stride = product of sizes[d+1..n-1],
        //       outer  = product of sizes[0..d-1] / stride / sizes[d]

        for (int d = 0; d < n_dim; ++d) {
            // Compute stride for dimension d
            int stride = 1;
            for (int k = d + 1; k < n_dim; ++k) stride *= grid_.sizes[k];

            int dim_size = grid_.sizes[d];
            int outer = total / (dim_size * stride);

            bool sweep_down = (priorities_.empty() || priorities_[d] == 0);
            // priority 0 → max is "good" → ↓B means sweep high→low
            // priority 1 → min is "good" → ↓B means sweep low→high

            if (sweep_down) {
                // Sweep from coord = dim_size-2 down to 0
                for (int i_outer = 0; i_outer < outer; ++i_outer) {
                    for (int c = dim_size - 2; c >= 0; --c) {
                        for (int i_inner = 0; i_inner < stride; ++i_inner) {
                            int flat     = i_outer * dim_size * stride + c * stride + i_inner;
                            int flat_above = flat + stride;  // coord c+1
                            bitmap_[flat] |= bitmap_[flat_above];
                        }
                    }
                }
            } else {
                // Sweep from coord = 1 up to dim_size-1
                for (int i_outer = 0; i_outer < outer; ++i_outer) {
                    for (int c = 1; c < dim_size; ++c) {
                        for (int i_inner = 0; i_inner < stride; ++i_inner) {
                            int flat      = i_outer * dim_size * stride + c * stride + i_inner;
                            int flat_below = flat - stride;  // coord c-1
                            bitmap_[flat] |= bitmap_[flat_below];
                        }
                    }
                }
            }
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        last_build_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
        return last_build_ms_;
    }

    // -----------------------------------------------------------------------
    // Queries — O(1)
    // -----------------------------------------------------------------------

    /// Check if a grid-index cell is in the safe set.  O(1).
    inline bool is_safe_grid(const int* idx) const {
        int flat = grid_.flatten(idx);
        return (flat >= 0 && flat < grid_.total_cells) && bitmap_[flat];
    }

    /// Check if a continuous state is in the safe set.  O(1).
    inline bool is_safe(const double* x) const {
        int idx[MAX_DIM];
        grid_.state_to_grid(x, idx);
        return is_safe_grid(idx);
    }

    /// Check with Eigen vector
    inline bool is_safe(const Vec& x) const {
        return is_safe(x.data());
    }

    /**
     * Conservative query under measurement uncertainty.
     * For dimensions with priority 0 (max = good), check x[d] + eps[d].
     * For dimensions with priority 1 (min = good), check x[d] - eps[d].
     * This ensures the query is valid for the WORST-CASE true state.
     */
    inline bool is_safe_conservative(const double* x, const double* eps) const {
        double x_worst[MAX_DIM];
        for (int d = 0; d < grid_.n_dim; ++d) {
            bool max_is_good = priorities_.empty() || priorities_[d] == 0;
            // Worst case: move state AWAY from the "good" direction
            x_worst[d] = max_is_good ? (x[d] - eps[d]) : (x[d] + eps[d]);
        }
        return is_safe(x_worst);
    }

    // -----------------------------------------------------------------------
    // Statistics
    // -----------------------------------------------------------------------

    /**
     * Scan the bitmap along a given dimension while holding other dims
     * fixed at their values in `state`, and return the safe range.
     *
     * @param dim    Dimension to scan.
     * @param state  Reference state (size >= n_dim). The scan varies
     *               state[dim] while keeping all other dims fixed.
     * @return (lo, hi) in physical units; both NaN if no safe cells found.
     */
    std::pair<double, double> safe_range_along_dim(int dim,
                                                    const double* state) const {
        if (dim < 0 || dim >= grid_.n_dim)
            return {std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()};

        // Map the reference state to grid indices
        int idx[MAX_DIM] = {};
        grid_.state_to_grid(state, idx);

        // Compute base flat index with dim set to 0
        int saved = idx[dim];
        idx[dim]  = 0;
        int base_flat = grid_.flatten(idx);
        idx[dim]  = saved;

        // Stride for the scan dimension
        int stride = 1;
        for (int d = dim + 1; d < grid_.n_dim; ++d) stride *= grid_.sizes[d];

        double lo = std::numeric_limits<double>::quiet_NaN();
        double hi = std::numeric_limits<double>::quiet_NaN();
        for (int i = 0; i < grid_.sizes[dim]; ++i) {
            int flat = base_flat + i * stride;
            if (flat >= 0 && flat < grid_.total_cells && bitmap_[flat]) {
                double val = grid_.lb[dim] + i * grid_.eta[dim];
                if (std::isnan(lo)) lo = val;
                hi = val;
            }
        }
        return {lo, hi};
    }

    /// Legacy convenience: scan along dim 0 for fixed (v_ego, s_onc).
    std::pair<double, double> safe_s_ego_range(double v_ego, double s_onc) const {
        double state[MAX_DIM] = {};
        state[0] = grid_.lb[0];
        state[1] = v_ego;
        if (grid_.n_dim > 2) state[2] = s_onc;
        return safe_range_along_dim(0, state);
    }
    int count_safe_cells() const {
        int count = 0;
        for (auto b : bitmap_) count += b;
        return count;
    }

    double last_build_ms() const { return last_build_ms_; }
    int total_cells() const { return grid_.total_cells; }
    double safe_fraction() const {
        return grid_.total_cells > 0
            ? static_cast<double>(count_safe_cells()) / grid_.total_cells
            : 0.0;
    }

    const GridDesc& grid() const { return grid_; }
    const std::vector<uint8_t>& bitmap() const { return bitmap_; }

    // -----------------------------------------------------------------------
    // Serialization
    // -----------------------------------------------------------------------
    void save_bitmap(const std::string& path) const {
        std::ofstream ofs(path, std::ios::binary);
        int total = grid_.total_cells;
        ofs.write(reinterpret_cast<const char*>(&total), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(bitmap_.data()), total);
    }

    bool load_bitmap(const std::string& path) {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs.is_open()) return false;
        int total = 0;
        ifs.read(reinterpret_cast<char*>(&total), sizeof(int));
        if (total != grid_.total_cells) return false;
        bitmap_.resize(total);
        ifs.read(reinterpret_cast<char*>(bitmap_.data()), total);
        return true;
    }

private:
    GridDesc              grid_;
    std::vector<int>      priorities_;
    std::vector<uint8_t>  bitmap_;
    std::vector<std::vector<int>> basis_;
    double                last_build_ms_ = 0.0;
    int                   basis_size_hint_ = 0;
};

}  // namespace rt_ctrl
