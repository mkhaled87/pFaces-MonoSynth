/**
 * @file safe_set.h
 * @brief Safe set with O(1) threshold-table queries.
 *
 * The safe set K_* = ↓B is the downward closure of an antichain basis B
 * under the component-wise partial order induced by the monotone system's
 * priority ordering.  This module stores K_* as a threshold table for O(1)
 * membership queries.  The threshold table is indexed by all dimensions
 * except d* (the dimension with the most grid cells) and stores the maximum
 * safe height along d* for each column.
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
    * @param priorities Per-dimension priority (0 = max-good, 1 = min-good).
     */
    void init(const GridDesc& grid, const std::vector<int>& priorities) {
        grid_       = grid;
        priorities_ = priorities;
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

    /// Set basis size for reporting (when TT is set externally)
    void set_basis_size_hint(int n) { basis_size_hint_ = n; }

    /**
     * Set threshold table directly from the mono_synth kernel output.
     * This replaces the bitmap: safety queries use O(1) threshold lookup
     * instead of the dense bitmap.  Avoids the expensive GPU bitmap build
     * and 97 MB GPU→CPU transfer.
     *
     * @param data         Threshold table values (size = table_size).
     * @param table_size   Number of entries in the table.
     * @param d_star       Dimension index used as the threshold dimension.
     * @param key_strides  Per-dim strides for computing key_flat (size = n_dim,
     *                     d_star entry is ignored).
     */
    void set_threshold_table(const int* data, int table_size, int d_star,
                             const std::vector<int>& key_strides) {
        tt_data_.assign(data, data + table_size);
        tt_size_ = table_size;
        tt_d_star_ = d_star;
        tt_key_strides_ = key_strides;
        has_tt_ = true;
        // Count safe cells for statistics
        tt_safe_cells_ = 0;
        for (int i = 0; i < table_size; ++i) tt_safe_cells_ += data[i];
        last_build_ms_ = 0.0;
    }

    /**
     * Set a bit-packed bitmap directly from the mono_synth bitmap GFP output.
     * Word j stores cells [32*j, 32*j+31], dim 0 fastest.
     */
    void set_bitmap_words(const uint32_t* words, size_t word_count) {
        bitmap_words_.assign(words, words + word_count);
        has_bitmap_ = true;
        has_tt_ = false;
        bitmap_safe_cells_ = 0;
        for (uint32_t w : bitmap_words_) {
            bitmap_safe_cells_ += __builtin_popcount(w);
        }
        last_build_ms_ = 0.0;
    }

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
                    for (auto& v : elem) v = std::max(0, v - 1);
                    if (!elem.empty()) basis_.push_back(std::move(elem));
                }
            } else if (!vals.empty()) {
                for (auto& v : vals) v = std::max(0, v - 1);
                basis_.push_back(std::move(vals));
            }
        }
        return !basis_.empty();
    }

    int basis_size() const {
        if (is_dual_proxy_)
            return (dual_a_ ? dual_a_->basis_size() : 0)
                 + (dual_b_ ? dual_b_->basis_size() : 0);
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
        const int64_t total = grid_.total_cells;

        // Allocate temporary bitmap for sweep
        std::vector<uint8_t> bitmap(total, 0);

        // Step 1: Mark basis elements
        for (const auto& b : basis_) {
            int flat = grid_.flatten(b.data());
            if (flat >= 0 && flat < total) {
                bitmap[flat] = 1;
            }
        }

        // Step 2: Prefix-OR sweep per dimension
        for (int d = 0; d < n_dim; ++d) {
            int stride = 1;
            for (int k = 0; k < d; ++k) stride *= grid_.sizes[k];

            int dim_size = grid_.sizes[d];
            int outer = total / (dim_size * stride);

            for (int i_outer = 0; i_outer < outer; ++i_outer) {
                for (int c = dim_size - 2; c >= 0; --c) {
                    for (int i_inner = 0; i_inner < stride; ++i_inner) {
                        int flat = i_outer * dim_size * stride + c * stride + i_inner;
                        int flat_above = flat + stride;
                        bitmap[flat] |= bitmap[flat_above];
                    }
                }
            }
        }

        // Step 3: Build threshold table from bitmap
        tt_d_star_ = 0;
        for (int d = 1; d < n_dim; ++d) {
            if (grid_.sizes[d] > grid_.sizes[tt_d_star_]) tt_d_star_ = d;
        }
        tt_key_strides_.resize(n_dim, 0);
        int tbl_stride = 1;
        for (int d = 0; d < n_dim; ++d) {
            if (d == tt_d_star_) continue;
            tt_key_strides_[d] = tbl_stride;
            tbl_stride *= grid_.sizes[d];
        }
        tt_size_ = tbl_stride;
        tt_data_.assign(tt_size_, 0);

        for (int flat = 0; flat < total; ++flat) {
            if (!bitmap[flat]) continue;
            int idx[MAX_DIM];
            grid_.unflatten(flat, idx);
            int key_flat = 0;
            for (int d = 0; d < n_dim; ++d) {
                if (d == tt_d_star_) continue;
                key_flat += idx[d] * tt_key_strides_[d];
            }
            tt_data_[key_flat] = std::max(tt_data_[key_flat], idx[tt_d_star_] + 1);
        }
        tt_safe_cells_ = 0;
        for (int i = 0; i < tt_size_; ++i) tt_safe_cells_ += tt_data_[i];
        has_tt_ = true;

        auto t1 = std::chrono::high_resolution_clock::now();
        last_build_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
        return last_build_ms_;
    }

    // -----------------------------------------------------------------------
    // Queries — O(1)
    // -----------------------------------------------------------------------

    /// Check if a grid-index cell is in the safe set.  O(1).
    inline bool is_safe_grid(const int* idx) const {
        if (has_bitmap_) {
            int flat = grid_.flatten(idx);
            if (flat < 0 || flat >= grid_.total_cells) return false;
            return (bitmap_words_[flat >> 5] & (uint32_t(1) << (flat & 31))) != 0;
        }
        if (!has_tt_) return false;
        int key_flat = 0;
        for (int d = 0; d < grid_.n_dim; ++d) {
            if (d == tt_d_star_) continue;
            key_flat += idx[d] * tt_key_strides_[d];
        }
        int thresh = (key_flat >= 0 && key_flat < tt_size_) ? tt_data_[key_flat] : 0;
        return (thresh > 0 && idx[tt_d_star_] < thresh);
    }

    /// Continuous state → grid index, conservatively rounded toward the
    /// less-safe direction using the configured monotonicity priorities.
    inline void state_to_grid_conservative(const double* x, int* idx) const {
        for (int d = 0; d < grid_.n_dim; ++d) {
            double clamped = std::max(grid_.lb[d], std::min(x[d], grid_.ub[d]));
            bool max_is_good = (priorities_.empty() || priorities_[d] == 0);
            double q = max_is_good
                ? (grid_.ub[d] - clamped) / grid_.eta[d]
                : (clamped - grid_.lb[d]) / grid_.eta[d];
            int id = static_cast<int>(std::ceil(q - 1e-9));
            idx[d] = std::max(0, std::min(id, grid_.sizes[d] - 1));
        }
    }

    /// Check if a continuous state is in the safe set.  O(1).
    /// In dual proxy mode, delegates to both sub-SafeSets.
    /// Bypass: if an oncoming vehicle has passed the collision zone, that
    /// sub-SafeSet's constraint is automatically satisfied.
    inline bool is_safe(const double* x) const {
        if (is_dual_proxy_) {
            // Check wait sub-SafeSet (or bypass if oncoming 1 has passed)
            bool a_ok = false;
            if (onc_fulldim_a_ >= 0 && x[onc_fulldim_a_] >= bypass_threshold_) {
                a_ok = true;  // oncoming 1 past collision zone → wait satisfied
            } else {
                double xa[MAX_DIM];
                for (size_t i = 0; i < dims_a_.size(); ++i) xa[i] = x[dims_a_[i]];
                a_ok = dual_a_->is_safe(xa);
            }
            // Check go sub-SafeSet (or bypass if oncoming 2 has passed)
            bool b_ok = false;
            if (onc_fulldim_b_ >= 0 && x[onc_fulldim_b_] >= bypass_threshold_) {
                b_ok = true;  // oncoming 2 past collision zone -> go satisfied
            } else {
                double xb[MAX_DIM];
                for (size_t i = 0; i < dims_b_.size(); ++i) xb[i] = x[dims_b_[i]];
                b_ok = dual_b_->is_safe(xb);
            }
            return a_ok && b_ok;
        }
        int idx[MAX_DIM];
        state_to_grid_conservative(x, idx);
        return is_safe_grid(idx);
    }

    /// Check only sub-SafeSet B ("go" / ego-first).
    /// Useful for goal-state queries where only the "go" grid covers s_ego=+10.
    inline bool is_safe_go(const double* x) const {
        if (is_dual_proxy_ && dual_b_) {
            double xb[MAX_DIM];
            for (size_t i = 0; i < dims_b_.size(); ++i) xb[i] = x[dims_b_[i]];
            return dual_b_->is_safe(xb);
        }
        return is_safe(x);  // fallback: non-dual mode
    }

    /// Check only sub-SafeSet A ("wait" / oncoming-first).
    inline bool is_safe_wait(const double* x) const {
        if (is_dual_proxy_ && dual_a_) {
            double xa[MAX_DIM];
            for (size_t i = 0; i < dims_a_.size(); ++i) xa[i] = x[dims_a_[i]];
            return dual_a_->is_safe(xa);
        }
        return is_safe(x);  // fallback: non-dual mode
    }

    /// Check with Eigen vector
    inline bool is_safe(const Vec& x) const {
        return is_safe(x.data());
    }

    /**
     * Conservative query under measurement uncertainty.
     * For dimensions with priority 0 (max = good), check x[d] - eps[d].
     * For dimensions with priority 1 (min = good), check x[d] + eps[d].
     * This ensures the query is valid for the WORST-CASE true state.
     */
    inline bool is_safe_conservative(const double* x, const double* eps) const {
        double x_worst[MAX_DIM];
        for (int d = 0; d < grid_.n_dim; ++d) {
            bool max_is_good = (priorities_.empty() || priorities_[d] == 0);
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
        if (is_dual_proxy_) {
            // In dual proxy mode, scan along the given dim and check both
            // sub-SafeSets at each cell. Use the full-state grid for stepping.
            if (dim < 0 || dim >= grid_.n_dim)
                return {std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN()};
            double lo = std::numeric_limits<double>::quiet_NaN();
            double hi = std::numeric_limits<double>::quiet_NaN();
            double x[MAX_DIM];
            for (int d = 0; d < grid_.n_dim; ++d) x[d] = state[d];
            for (int i = 0; i < grid_.sizes[dim]; ++i) {
                bool max_is_good = (priorities_.empty() || priorities_[dim] == 0);
                x[dim] = max_is_good
                    ? (grid_.ub[dim] - i * grid_.eta[dim])
                    : (grid_.lb[dim] + i * grid_.eta[dim]);
                if (is_safe(x)) {
                    double val = x[dim];
                    if (std::isnan(lo)) lo = val;
                    hi = val;
                }
            }
            return {lo, hi};
        }

        if (dim < 0 || dim >= grid_.n_dim)
            return {std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()};

        // Map the reference state to grid indices conservatively
        int idx[MAX_DIM] = {};
        state_to_grid_conservative(state, idx);

        // Scan along dim using threshold table
        double lo = std::numeric_limits<double>::quiet_NaN();
        double hi = std::numeric_limits<double>::quiet_NaN();
        bool max_is_good = (priorities_.empty() || priorities_[dim] == 0);
        for (int i = 0; i < grid_.sizes[dim]; ++i) {
            idx[dim] = i;
            if (is_safe_grid(idx)) {
                double val = max_is_good
                    ? (grid_.ub[dim] - i * grid_.eta[dim])
                    : (grid_.lb[dim] + i * grid_.eta[dim]);
                if (std::isnan(lo)) lo = val;
                hi = val;
            }
        }
        idx[dim] = 0;  // restore
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
    int64_t count_safe_cells() const {
        if (is_dual_proxy_) {
            int64_t ca = dual_a_ ? dual_a_->count_safe_cells() : 0;
            int64_t cb = dual_b_ ? dual_b_->count_safe_cells() : 0;
            return (ca + cb) / 2;
        }
        return has_bitmap_ ? bitmap_safe_cells_ : tt_safe_cells_;
    }

    double last_build_ms() const { return last_build_ms_; }
    int64_t total_cells() const { return grid_.total_cells; }
    double safe_fraction() const {
        if (is_dual_proxy_) {
            double fa = dual_a_ ? dual_a_->safe_fraction() : 0.0;
            double fb = dual_b_ ? dual_b_->safe_fraction() : 0.0;
            return std::min(fa, fb);  // conservative: min of two sub-fractions
        }
        return grid_.total_cells > 0
            ? static_cast<double>(count_safe_cells()) / grid_.total_cells
            : 0.0;
    }

    const GridDesc& grid() const { return grid_; }

    // -----------------------------------------------------------------------
    // Dual proxy mode — two independent sub-SafeSets
    // -----------------------------------------------------------------------

    /**
     * Initialize as a dual proxy that delegates is_safe() to two independent
     * sub-SafeSets by extracting the relevant coordinates from the full state.
     *
     * For the two-oncoming scenario with state x = (s_ego, v_ego, s_onc1, s_onc2):
     *   - ss_a (wait): checks (x[dims_a[0]], x[dims_a[1]], x[dims_a[2]])
     *   - ss_b (go):   checks (x[dims_b[0]], x[dims_b[1]], x[dims_b[2]])
     *
     * A state is safe iff it is safe in BOTH sub-SafeSets.
     * No bitmap is allocated on this object.
     *
     * When a sub-SafeSet's oncoming vehicle has passed the collision zone
     * (full-state value exceeds bypass_threshold), that sub-constraint is
     * automatically satisfied and the bitmap check is skipped.
     *
     * @param grid    Grid descriptor for the full state space (used for bounds).
     * @param ss_a    First sub-SafeSet (e.g. wait / oncoming-first).
     * @param dims_a  Coordinate indices to extract for ss_a.
     * @param ss_b    Second sub-SafeSet (e.g. go / ego-first).
     * @param dims_b  Coordinate indices to extract for ss_b.
     * @param onc_fulldim_a  Full-state dimension of the oncoming vehicle tracked by ss_a.
     * @param onc_fulldim_b  Full-state dimension of the oncoming vehicle tracked by ss_b.
     * @param bypass_threshold  If oncoming exceeds this, sub-constraint is auto-satisfied.
     */
    void init_dual_proxy(const GridDesc& grid,
                         SafeSet* ss_a, const std::vector<int>& dims_a,
                         SafeSet* ss_b, const std::vector<int>& dims_b,
                         int onc_fulldim_a = -1, int onc_fulldim_b = -1,
                         double bypass_threshold = 10.0) {
        grid_       = grid;
        is_dual_proxy_ = true;
        dual_a_     = ss_a;
        dual_b_     = ss_b;
        dims_a_     = dims_a;
        dims_b_     = dims_b;
        onc_fulldim_a_    = onc_fulldim_a;
        onc_fulldim_b_    = onc_fulldim_b;
        bypass_threshold_ = bypass_threshold;
        // No bitmap — queries delegate to sub-SafeSets
        basis_.clear();
    }

    bool is_dual_proxy() const { return is_dual_proxy_; }

    /// Access sub-SafeSets (for statistics / re-synthesis)
    SafeSet* dual_ss_a() { return dual_a_; }
    SafeSet* dual_ss_b() { return dual_b_; }
    const SafeSet* dual_ss_a() const { return dual_a_; }
    const SafeSet* dual_ss_b() const { return dual_b_; }

private:
    inline void grid_to_state_internal(const int* idx, double* x) const {
        for (int d = 0; d < grid_.n_dim; ++d) {
            bool max_is_good = (priorities_.empty() || priorities_[d] == 0);
            x[d] = max_is_good
                ? (grid_.ub[d] - idx[d] * grid_.eta[d])
                : (grid_.lb[d] + idx[d] * grid_.eta[d]);
        }
    }

    GridDesc              grid_;
    std::vector<int>      priorities_;
    std::vector<std::vector<int>> basis_;
    std::vector<uint32_t> bitmap_words_;
    double                last_build_ms_ = 0.0;
    int                   basis_size_hint_ = 0;

    // Dual proxy state
    bool                  is_dual_proxy_ = false;
    SafeSet*              dual_a_ = nullptr;
    SafeSet*              dual_b_ = nullptr;
    std::vector<int>      dims_a_;  ///< Coord indices from full state → sub-SafeSet A
    std::vector<int>      dims_b_;  ///< Coord indices from full state → sub-SafeSet B
    int                   onc_fulldim_a_ = -1;  ///< Full-state dim of wait oncoming
    int                   onc_fulldim_b_ = -1;  ///< Full-state dim of go oncoming
    double                bypass_threshold_ = 10.0;  ///< Oncoming has passed collision zone

    // Threshold table (replaces bitmap for O(1) lookup in GPU TT-only mode)
    bool                  has_bitmap_ = false;
    int64_t               bitmap_safe_cells_ = 0;
    bool                  has_tt_ = false;
    std::vector<int>      tt_data_;
    int                   tt_size_ = 0;
    int                   tt_d_star_ = 0;
    std::vector<int>      tt_key_strides_;
    int64_t               tt_safe_cells_ = 0;
};

}  // namespace rt_ctrl
