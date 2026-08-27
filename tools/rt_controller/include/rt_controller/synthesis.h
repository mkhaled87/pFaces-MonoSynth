/**
 * @file synthesis.h
 * @brief pFaces SDK wrapper for safe set synthesis.
 *
 * Provides two backends:
 *   1. DirectSynthesis  — calls pFaces SDK API directly (in-memory).
 *   2. ExternalSynthesis — shells out to the pFaces binary and reads the
 *                          basis CSV output (fallback when SDK not linked).
 *
 * The wrapper abstracts the synthesis step so the controller loop simply
 * calls  synthesize(v_oncoming) → SafeSet  without knowing the backend.
 *
 * Compile with -DHAS_PFACES_SDK=1 to enable the direct backend.
 */

#pragma once

#include "config.h"
#include "safe_set.h"
#include "types.h"

#include <chrono>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Optional: pFaces SDK direct integration
// ---------------------------------------------------------------------------
#if defined(HAS_PFACES_SDK) && HAS_PFACES_SDK
#include <pfaces-sdk.h>
#include "configReader.h"
#include "pfacesKernel_mono_synth.h"
#endif

namespace rt_ctrl {

// ---------------------------------------------------------------------------
// Synthesis timing breakdown (filled by backends that support it)
// ---------------------------------------------------------------------------
struct SynthesisDetail {
    double total_ms       = 0.0;  ///< Wall-clock total
    double gpu_exec_ms    = 0.0;  ///< GPU kernel execution (all phases)
    double transfer_ms    = 0.0;  ///< GPU→CPU bitmap transfer
    double precompute_ms  = 0.0;  ///< Precompute transitions phase
    double gfp_ms         = 0.0;  ///< GFP iteration phase (from compute_start)
    int    iterations     = 0;    ///< Fixed-point iteration count
    int    basis_size     = 0;    ///< Antichain basis |B|
    int64_t safe_cells     = 0;    ///< Total safe cells
    int64_t total_cells    = 0;    ///< Total grid cells
};

// ---------------------------------------------------------------------------
// Abstract synthesis interface
// ---------------------------------------------------------------------------
class SynthesisBackend {
public:
    virtual ~SynthesisBackend() = default;

    /**
     * Run synthesis for a given sensed oncoming velocity.
     *
     * @param v_oncoming  Sensed oncoming vehicle velocity (m/s).
     * @param safe_set    [out] SafeSet to populate with the computed basis.
     * @return            Synthesis wall-clock time in milliseconds.
     */
    virtual double synthesize(double v_oncoming, SafeSet& safe_set) = 0;

    /// Last synthesis detail (populated by backends that support it)
    const SynthesisDetail& last_detail() const { return last_detail_; }

protected:
    SynthesisDetail last_detail_;
};

// ===========================================================================
// Backend 1: External process (pFaces CLI)
// ===========================================================================
/**
 * Shells out to the pFaces binary, reads the resulting basis_*.csv file.
 * Works without linking the SDK — only requires the pFaces binary on $PATH.
 *
 * Workflow:
 *   1. Generate a temporary .cfg with updated V0_MAX = v_oncoming.
 *   2. Run: pfaces -CG -k <kernel_pack> -cfg <temp.cfg> -d <device> -p
 *   3. Parse the output basis CSV.
 *   4. Feed basis into SafeSet.
 */
class ExternalSynthesis : public SynthesisBackend {
public:
    struct Params {
        std::string cfg_path;         ///< Path to the base .cfg file
        std::string kernel_pack;      ///< Kernel pack directory
        std::string dynamics_file;    ///< Dynamics .cl file to patch
        std::string synthesis_macro;  ///< #define to patch (e.g. "V0_MAX"), empty = no patching
        int device_id       = 1;      ///< pFaces device (0=GPU, 1=CPU usually)
        std::string output_dir = "."; ///< Where pFaces writes output
    };

    ExternalSynthesis(const Params& params, const Config& cfg)
        : params_(params), cfg_(cfg)
    {
        // Resolve dynamics_file relative to cfg_path directory
        if (!params_.dynamics_file.empty() && params_.dynamics_file[0] != '/') {
            namespace fs = std::filesystem;
            auto cfg_dir = fs::path(params_.cfg_path).parent_path();
            auto candidate = cfg_dir / params_.dynamics_file;
            if (fs::exists(candidate))
                params_.dynamics_file = candidate.string();
        }
    }

    double synthesize(double param_value, SafeSet& safe_set) override {
        auto t0 = std::chrono::high_resolution_clock::now();
        namespace fs = std::filesystem;

        // 1. Generate patched dynamics file with updated macro
        std::string patched_dynamics = patch_dynamics_file(param_value);

        // 2. Generate temporary .cfg pointing to patched dynamics
        std::string temp_cfg = generate_temp_cfg(patched_dynamics);

        // Figure out the directory where we run pFaces (same as temp_cfg)
        auto run_dir = fs::path(temp_cfg).parent_path().string();
        if (run_dir.empty()) run_dir = ".";

        // 3. Run pFaces from the cfg directory
        std::ostringstream cmd;
        cmd << "cd " << run_dir << " && "
            << "pfaces -CG"
            << " -k mono_synth.cpu@" << fs::absolute(params_.kernel_pack).string()
            << " -cfg " << fs::path(temp_cfg).filename().string()
            << " -d " << params_.device_id
            << " -p 2>&1";

        std::cout << "[Synthesis] Running: " << cmd.str() << "\n";
        int ret = std::system(cmd.str().c_str());
        if (ret != 0) {
            std::cerr << "[Synthesis] pFaces returned error code " << ret << "\n";
        }

        // 4. Find and parse basis CSV (look in the run directory)
        std::string basis_file = find_basis_csv(run_dir);
        if (!basis_file.empty()) {
            std::cout << "[Synthesis] Loading basis from: " << basis_file << "\n";
            safe_set.load_basis_csv(basis_file);
        } else {
            std::cerr << "[Synthesis] WARNING: No basis CSV found in " << run_dir << "\n";
        }

        // 5. Build bitmap
        safe_set.build_bitmap();

        auto t1 = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

private:
    Params params_;
    Config cfg_;

    std::string patch_dynamics_file(double param_value) {
        // Read original dynamics file
        std::ifstream ifs(params_.dynamics_file);
        std::ostringstream content;
        content << ifs.rdbuf();
        std::string src = content.str();

        // Replace the synthesis macro (e.g. "V0_MAX", "V0_MIN") if set
        if (!params_.synthesis_macro.empty()) {
            std::string old_def = "#define " + params_.synthesis_macro;
            auto pos = src.find(old_def);
            if (pos != std::string::npos) {
                auto eol = src.find('\n', pos);
                std::ostringstream new_def;
                new_def << "#define " << params_.synthesis_macro << " "
                        << std::fixed << std::setprecision(1) << param_value << "f";
                src.replace(pos, eol - pos, new_def.str());
            }
        }

        // Write patched file
        std::string patched_path = params_.output_dir + "/patched_dynamics.cl";
        std::ofstream ofs(patched_path);
        ofs << src;
        return patched_path;
    }

    std::string generate_temp_cfg(const std::string& patched_dynamics) {
        std::string temp_path = params_.output_dir + "/temp_synth.cfg";
        std::ifstream ifs(params_.cfg_path);
        std::ostringstream content;
        content << ifs.rdbuf();
        std::string src = content.str();

        // Replace dynamics file reference
        auto pos = src.find("user_dynamics_file");
        if (pos != std::string::npos) {
            auto eq = src.find('=', pos);
            auto eol = src.find('\n', pos);
            if (eq != std::string::npos && eol != std::string::npos) {
                std::string new_line = "user_dynamics_file = \"" + patched_dynamics + "\";";
                src.replace(pos, eol - pos, new_line);
            }
        }

        std::ofstream ofs(temp_path);
        ofs << src;
        return temp_path;
    }

    std::string find_basis_csv(const std::string& search_dir = ".") {
        namespace fs = std::filesystem;
        std::string best;
        std::filesystem::file_time_type best_time{};
        for (auto& entry : fs::directory_iterator(search_dir)) {
            if (entry.path().extension() == ".csv" &&
                entry.path().filename().string().find("basis") != std::string::npos) {
                auto wt = entry.last_write_time();
                if (best.empty() || wt > best_time) {
                    best = entry.path().string();
                    best_time = wt;
                }
            }
        }
        return best;
    }
};

// ===========================================================================
// Backend 2: Direct pFaces SDK integration (in-memory)
// ===========================================================================
#if defined(HAS_PFACES_SDK) && HAS_PFACES_SDK

/**
 * Directly instantiates pfacesKernel_mono_synth, runs the parallel program,
 * and extracts the basis from m_safe_set_basis without any file I/O.
 *
 * Requirements:
 *   - Link against pFaces SDK libraries
 *   - PFACES_SDK_ROOT environment variable set
 */
class DirectSynthesis : public SynthesisBackend {
public:
    struct Params {
        std::string cfg_path;
        std::string kernel_pack;
        int device_id = 1;
        bool has_runtime_param = true;  ///< false for scenarios with no measured param
    };

    DirectSynthesis(const Params& params)
        : params_(params)
    {
        // Ensure kernel_pack path ends with '/' for SDK concatenation
        if (!params_.kernel_pack.empty() && params_.kernel_pack.back() != '/')
            params_.kernel_pack += '/';

        // 1. Detect compute devices (GPU only, matching pFaces CLI behavior)
        machine_ = std::make_unique<pfacesMachineIdentifier>(
            pfacesDeviceSelection{false, true, false});  // GPU only
        
        // Use configured GPU device index when available (0-based within GPU list)
        auto allDevices = machine_->getDeviceIndicies({false, true, false});
        if (allDevices.empty()) {
            throw std::runtime_error("[DirectSynthesis] No GPU devices found");
        }
        size_t gpuSlot = 0;
        if (params_.device_id >= 0 && static_cast<size_t>(params_.device_id) < allDevices.size()) {
            gpuSlot = static_cast<size_t>(params_.device_id);
        }
        size_t devIdx = allDevices[gpuSlot];
        std::vector<size_t> targets = { devIdx };
        
        // 2. Create OpenCL context for the target device
        const cl::Device& dev = machine_->getDevice(devIdx);
        auto ctx = std::make_shared<cl::Context>(std::vector<cl::Device>{dev});
        
        std::cout << "[DirectSynthesis] Using device " << devIdx 
                  << ": " << machine_->getDeviceName(devIdx) << "\n";
        
        // 3. Create pFaces configuration reader with schema
        auto spCfg = std::make_shared<pfacesConfigurationReader>(
            params_.cfg_path, "", false);
        spCfg->parse(defaultConfiguration::getSchema(),
                     defaultConfiguration::getDefaults());
        
        // 4. Create kernel launch state. Match the scope to the selected device
        // so the SDK loads the GPU kernel pack variant for GPU execution.
        const std::string kernel_scope =
            pfacesKernelSource::getDefaultScope(*machine_, devIdx);
        auto spLaunchState = std::make_shared<pfacesKernelLaunchState>(
            "mono_synth", kernel_scope, params_.kernel_pack);
        
        // 5. Create kernel instance
        kernel_ = std::make_shared<mono_synth::pfacesKernel_mono_synth>(
            spLaunchState, spCfg);
        
        // Configure for direct mode: no CSV, no benchmark repetition, skip cache
        kernel_->setSkipCache(true);
        if (kernel_->getSynthesisMethod() == SynthesisMethod::THRESHOLD) {
            kernel_->setExtractBasis(false);
        }

        // Load persisted tune results. If none exist, mark as tuned with defaults.
        // (pFaces SDK will use internal defaults when setTuned(true) is called.)
        if (!kernel_->loadTuneResults(*machine_)) {
            std::cout << "[DirectSynthesis] No tune results found. Using defaults.\n";
            kernel_->setTuned(true);
            std::cout << "[DirectSynthesis] Kernel marked as tuned (defaults).\n";
        }
        
        // 6. Create parallel program and configure
        program_ = std::make_unique<pfacesParallelProgram>(
            *machine_, ctx, targets);
        // Fixed job distribution: 100% to single device
        program_->m_isFixedJobDistribution = true;
        program_->m_fixedJobDistribution = { 1.0 };
        program_->m_beVerboseLevel = 0;
        kernel_->configureParallelProgram(*program_);
        
        compiled_ = false;
        std::cout << "[DirectSynthesis] Kernel initialized, ready for synthesis.\n";
    }

    double synthesize(double param_value, SafeSet& safe_set) override {
        using Clk = std::chrono::high_resolution_clock;
        auto t0 = Clk::now();

        // Stage A: param write (only if scenario has a runtime parameter)
        if (params_.has_runtime_param) {
            if (program_->m_dataPool.size() > 1 && program_->m_dataPool[1].first) {
                float* params = (float*)program_->m_dataPool[1].first;
                params[0] = (float)param_value;
            }
            kernel_->setRuntimeParam0((float)param_value);
        }

        // Stage B: GPU kernel execution (precompute_transitions → iterations)
        // Suppress pFaces internal prints (benchmark stats)
        auto tg0 = Clk::now();
        struct CoutGuard {
            std::streambuf* orig;
            std::ostringstream sink;
            CoutGuard() : orig(std::cout.rdbuf()) { std::cout.rdbuf(sink.rdbuf()); }
            ~CoutGuard() { std::cout.rdbuf(orig); }
        } cout_guard;
        auto profile = kernel_->runSinglePlatformMultipleDevices(*program_);
        std::cout.rdbuf(cout_guard.orig);  // restore early
        auto tg1 = Clk::now();

        // Stage C: populate SafeSet from kernel threshold table
        auto tt0 = Clk::now();
        if (kernel_->getSynthesisMethod() == SynthesisMethod::BITMAP_REFERENCE) {
            const auto& bitmap = kernel_->getBitmapWords();
            safe_set.set_bitmap_words(
                reinterpret_cast<const uint32_t*>(bitmap.data()),
                kernel_->getBitmapWordCount());
            safe_set.set_basis_size_hint(kernel_->getBasisSize());
        } else {
            const auto& tt = kernel_->getThresholdTable();
            safe_set.set_threshold_table(
                tt.data(),
                kernel_->getThresholdTableSize(),
                kernel_->getThresholdDStar(),
                kernel_->getThresholdKeyStrides());
            safe_set.set_basis_size_hint(kernel_->getBasisSize());
        }
        auto tt1 = Clk::now();

        auto t1 = Clk::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // Populate detail struct
        last_detail_.total_ms    = ms;
        last_detail_.gpu_exec_ms = std::chrono::duration<double, std::milli>(tg1 - tg0).count();
        last_detail_.transfer_ms = std::chrono::duration<double, std::milli>(tt1 - tt0).count();
        last_detail_.iterations  = kernel_->m_iterations;
        last_detail_.basis_size  = kernel_->getBasisSize();
        last_detail_.safe_cells  = safe_set.count_safe_cells();
        last_detail_.total_cells = safe_set.total_cells();

        // Phase timing from kernel host functions
        last_detail_.precompute_ms = kernel_->m_precompute_ms;
        last_detail_.gfp_ms        = kernel_->m_gfp_total_ms;

        return ms;
    }

private:
    Params params_;
    std::unique_ptr<pfacesMachineIdentifier> machine_;
    std::unique_ptr<pfacesParallelProgram> program_;
    std::shared_ptr<mono_synth::pfacesKernel_mono_synth> kernel_;
    bool compiled_;
};

#endif  // HAS_PFACES_SDK

// ===========================================================================
// Backend 2b: fast OpenCL TT-inline path for RT control
// ===========================================================================
#if defined(HAS_PFACES_SDK) && HAS_PFACES_SDK

class FastTTInlineSynthesis : public SynthesisBackend {
public:
    struct Params {
        std::string cfg_path;
        std::string kernel_pack;
        int device_id = 0;
        bool has_runtime_param = true;
    };

    FastTTInlineSynthesis(const Params& params)
        : params_(params)
    {
        if (!params_.kernel_pack.empty() && params_.kernel_pack.back() != '/')
            params_.kernel_pack += '/';

        machine_ = std::make_unique<pfacesMachineIdentifier>(
            pfacesDeviceSelection{false, true, false});
        auto all_devices = machine_->getDeviceIndicies({false, true, false});
        if (all_devices.empty())
            throw std::runtime_error("[FastTTInline] No GPU devices found");

        size_t gpu_slot = 0;
        if (params_.device_id >= 0 && static_cast<size_t>(params_.device_id) < all_devices.size())
            gpu_slot = static_cast<size_t>(params_.device_id);
        const size_t dev_idx = all_devices[gpu_slot];
        device_ = machine_->getDevice(dev_idx);
        context_ = std::make_unique<cl::Context>(std::vector<cl::Device>{device_});
        queue_ = std::make_unique<cl::CommandQueue>(*context_, device_);

        std::cout << "[FastTTInline] Using device " << dev_idx
                  << ": " << machine_->getDeviceName(dev_idx) << "\n";

        auto sp_cfg = std::make_shared<pfacesConfigurationReader>(
            params_.cfg_path, "", false);
        sp_cfg->setOverrideConfigurations({
            {"synthesis_method", "threshold"},
            {"transition_semantics", "extremal_single_successor"},
            {"transition_backend", "inline"},
            {"boundary_semantics", "favorable_saturating"},
            {"use_threshold_table", "__mono_synth_legacy_unset__"},
            {"use_tt_only", "__mono_synth_legacy_unset__"},
            {"use_tt_only_gpu", "__mono_synth_legacy_unset__"},
            {"use_bitmap_gfp", "__mono_synth_legacy_unset__"},
            {"use_inline_dynamics", "__mono_synth_legacy_unset__"},
            {"use_prefix_sweep", "__mono_synth_legacy_unset__"},
            {"boundary_seeding", "__mono_synth_legacy_unset__"},
        });
        sp_cfg->parse(defaultConfiguration::getSchema(),
                      defaultConfiguration::getDefaults());

        const std::string kernel_scope =
            pfacesKernelSource::getDefaultScope(*machine_, dev_idx);
        auto sp_launch_state = std::make_shared<pfacesKernelLaunchState>(
            "mono_synth", kernel_scope, params_.kernel_pack);

        kernel_meta_ = std::make_shared<mono_synth::pfacesKernel_mono_synth>(
            sp_launch_state, sp_cfg);
        kernel_meta_->setSkipCache(true);
        kernel_meta_->setExtractBasis(false);

        if (kernel_meta_->getSynthesisMethod() != SynthesisMethod::THRESHOLD ||
            kernel_meta_->getTransitionBackend() != TransitionBackend::INLINE) {
            throw std::runtime_error(
                "[FastTTInline] requires synthesis_method=threshold and transition_backend=inline");
        }
        table_size_ = kernel_meta_->getThresholdTableSize();
        d_star_ = kernel_meta_->getThresholdDStar();
        key_strides_ = kernel_meta_->getThresholdKeyStrides();
        const auto& grid_sizes = kernel_meta_->getGridSizes();
        n_dstar_ = static_cast<int>(grid_sizes[d_star_]);

        std::string source = load_kernel_source(params_.kernel_pack + "mono_synth.gpu.cl");
        auto replacements = kernel_meta_->getOpenClParameterList();
        for (size_t i = 0; i < replacements.first.size(); ++i)
            replace_all(source, replacements.first[i], replacements.second[i]);
        std::string unresolved = find_unresolved_macro(source);
        if (!unresolved.empty())
            throw std::runtime_error("[FastTTInline] unresolved kernel macro: " + unresolved);

        program_ = std::make_unique<cl::Program>(*context_, source);
        cl_int err = program_->build(std::vector<cl::Device>{device_}, "");
        if (err != CL_SUCCESS) {
            std::string log = program_->getBuildInfo<CL_PROGRAM_BUILD_LOG>(device_);
            throw std::runtime_error("[FastTTInline] OpenCL build failed:\n" + log);
        }
        column_kernel_ = std::make_unique<cl::Kernel>(*program_, "threshold_gfp_step");

        dummy_next_ = std::make_unique<cl::Buffer>(*context_, CL_MEM_READ_ONLY, sizeof(cl_ulong));
        tt_a_ = std::make_unique<cl::Buffer>(*context_, CL_MEM_READ_WRITE, table_size_ * sizeof(cl_uint));
        tt_b_ = std::make_unique<cl::Buffer>(*context_, CL_MEM_READ_WRITE, table_size_ * sizeof(cl_uint));
        parity_ = std::make_unique<cl::Buffer>(*context_, CL_MEM_READ_ONLY, sizeof(cl_uint));
        changed_ = std::make_unique<cl::Buffer>(*context_, CL_MEM_READ_WRITE, sizeof(cl_uint));
        runtime_params_ = std::make_unique<cl::Buffer>(*context_, CL_MEM_READ_ONLY, 4 * sizeof(cl_float));
        tt_host_.resize(table_size_);

        std::cout << "[FastTTInline] Kernel built once; table=" << table_size_
                  << " entries, d*=" << d_star_ << "\n";
    }

    double synthesize(double param_value, SafeSet& safe_set) override {
        using Clk = std::chrono::high_resolution_clock;
        auto t0 = Clk::now();

        std::fill(tt_host_.begin(), tt_host_.end(), n_dstar_);
        cl_float rt_params[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        if (params_.has_runtime_param)
            rt_params[0] = static_cast<cl_float>(param_value);

        queue_->enqueueWriteBuffer(*tt_a_, CL_FALSE, 0, table_size_ * sizeof(cl_uint), tt_host_.data());
        queue_->enqueueWriteBuffer(*tt_b_, CL_FALSE, 0, table_size_ * sizeof(cl_uint), tt_host_.data());
        queue_->enqueueWriteBuffer(*runtime_params_, CL_FALSE, 0, sizeof(rt_params), rt_params);

        cl_uint parity = 0;
        uint64_t iterations = 0;
        if (n_dstar_ > 0 && table_size_ >
                std::numeric_limits<uint64_t>::max() / static_cast<uint64_t>(n_dstar_))
            throw std::overflow_error("[FastTTInline] convergence bound overflow");
        const uint64_t max_iterations =
            table_size_ * static_cast<uint64_t>(n_dstar_) + 1;
        auto tg0 = Clk::now();
        while (true) {
            cl_uint changed = 0;
            queue_->enqueueWriteBuffer(*parity_, CL_FALSE, 0, sizeof(parity), &parity);
            queue_->enqueueWriteBuffer(*changed_, CL_FALSE, 0, sizeof(changed), &changed);

            column_kernel_->setArg(0, *dummy_next_);
            column_kernel_->setArg(1, *tt_a_);
            column_kernel_->setArg(2, *tt_b_);
            column_kernel_->setArg(3, *parity_);
            column_kernel_->setArg(4, *changed_);
            column_kernel_->setArg(5, *runtime_params_);
            queue_->enqueueNDRangeKernel(*column_kernel_, cl::NullRange,
                                         cl::NDRange(static_cast<size_t>(table_size_)),
                                         cl::NullRange);
            queue_->enqueueReadBuffer(*changed_, CL_TRUE, 0, sizeof(changed), &changed);
            ++iterations;
            if (changed == 0)
                break;
            parity ^= 1u;
            if (iterations > max_iterations)
                throw std::runtime_error("[FastTTInline] exceeded convergence guard");
        }
        queue_->finish();
        auto tg1 = Clk::now();

        cl::Buffer& result = parity ? *tt_b_ : *tt_a_;
        queue_->enqueueReadBuffer(result, CL_TRUE, 0, table_size_ * sizeof(cl_uint), tt_host_.data());
        safe_set.set_threshold_table(tt_host_.data(), table_size_, d_star_, key_strides_);

        auto t1 = Clk::now();
        const double total_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        const double gpu_ms = std::chrono::duration<double, std::milli>(tg1 - tg0).count();

        last_detail_.total_ms = total_ms;
        last_detail_.gpu_exec_ms = gpu_ms;
        last_detail_.transfer_ms = total_ms - gpu_ms;
        last_detail_.precompute_ms = 0.0;
        last_detail_.gfp_ms = gpu_ms;
        if (iterations > static_cast<uint64_t>(std::numeric_limits<int>::max()))
            throw std::overflow_error("[FastTTInline] iteration count exceeds int32");
        last_detail_.iterations = static_cast<int>(iterations);
        last_detail_.basis_size = 0;
        last_detail_.safe_cells = safe_set.count_safe_cells();
        last_detail_.total_cells = safe_set.total_cells();
        return total_ms;
    }

private:
    static void replace_all(std::string& s, const std::string& from, const std::string& to) {
        if (from.empty()) return;
        size_t pos = 0;
        while ((pos = s.find(from, pos)) != std::string::npos) {
            s.replace(pos, from.size(), to);
            pos += to.size();
        }
    }

    static std::string read_text(const std::string& path) {
        std::ifstream ifs(path);
        if (!ifs.is_open())
            throw std::runtime_error("[FastTTInline] failed to read " + path);
        std::ostringstream oss;
        oss << ifs.rdbuf();
        return oss.str();
    }

    static std::string find_unresolved_macro(const std::string& source) {
        size_t pos = 0;
        while ((pos = source.find("@@", pos)) != std::string::npos) {
            const size_t start = pos + 2;
            const size_t end = source.find("@@", start);
            if (end == std::string::npos)
                return "";
            bool looks_like_macro = end > start;
            for (size_t i = start; i < end; ++i) {
                unsigned char c = static_cast<unsigned char>(source[i]);
                if (!std::isalnum(c) && source[i] != '_') {
                    looks_like_macro = false;
                    break;
                }
            }
            if (looks_like_macro)
                return source.substr(pos, end + 2 - pos);
            pos = start;
        }
        return "";
    }

    std::string load_kernel_source(const std::string& path) const {
        std::string source = read_text(path);
        const std::string marker = "@pfaces-include:\"";
        size_t pos = 0;
        while ((pos = source.find(marker, pos)) != std::string::npos) {
            const size_t name_start = pos + marker.size();
            const size_t name_end = source.find('"', name_start);
            if (name_end == std::string::npos)
                throw std::runtime_error("[FastTTInline] malformed include directive");
            const std::string include_name = source.substr(name_start, name_end - name_start);
            const size_t line_end = source.find('\n', name_end);
            const size_t replace_end = (line_end == std::string::npos) ? name_end + 1 : line_end + 1;
            const std::string include_text = read_text(params_.kernel_pack + include_name);
            source.replace(pos, replace_end - pos, include_text + "\n");
            pos += include_text.size() + 1;
        }
        return source;
    }

    Params params_;
    std::unique_ptr<pfacesMachineIdentifier> machine_;
    cl::Device device_;
    std::unique_ptr<cl::Context> context_;
    std::unique_ptr<cl::CommandQueue> queue_;
    std::shared_ptr<mono_synth::pfacesKernel_mono_synth> kernel_meta_;
    std::unique_ptr<cl::Program> program_;
    std::unique_ptr<cl::Kernel> column_kernel_;
    std::unique_ptr<cl::Buffer> dummy_next_;
    std::unique_ptr<cl::Buffer> tt_a_;
    std::unique_ptr<cl::Buffer> tt_b_;
    std::unique_ptr<cl::Buffer> parity_;
    std::unique_ptr<cl::Buffer> changed_;
    std::unique_ptr<cl::Buffer> runtime_params_;
    std::vector<uint32_t> tt_host_;
    uint64_t table_size_ = 0;
    int d_star_ = 0;
    int n_dstar_ = 0;
    std::vector<uint64_t> key_strides_;
};

#endif  // HAS_PFACES_SDK

// ===========================================================================
// Backend 3: File-based (load pre-computed basis, no synthesis)
// ===========================================================================
/**
 * For testing and offline workflows: load a previously saved basis CSV
 * and build the bitmap.  No actual synthesis is performed.
 */
class FileSynthesis : public SynthesisBackend {
public:
    FileSynthesis(const std::string& basis_csv_path)
        : basis_path_(basis_csv_path) {}

    double synthesize(double /*v_oncoming*/, SafeSet& safe_set) override {
        auto t0 = std::chrono::high_resolution_clock::now();

        if (!safe_set.load_basis_csv(basis_path_)) {
            throw std::runtime_error("Failed to load basis from: " + basis_path_);
        }
        safe_set.build_bitmap();

        auto t1 = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

private:
    std::string basis_path_;
};

// ===========================================================================
// Factory
// ===========================================================================
inline std::unique_ptr<SynthesisBackend> create_synthesis(
    const std::string& mode,
    const Config& cfg,
    const std::string& extra_path = "")
{
    if (mode == "file") {
        return std::make_unique<FileSynthesis>(extra_path);
    }
    if (mode == "external") {
        ExternalSynthesis::Params p;
        p.cfg_path      = extra_path;
        p.kernel_pack   = "../../kernel-pack";
        p.dynamics_file = cfg.dynamics_file;
        p.device_id     = 1;
        return std::make_unique<ExternalSynthesis>(p, cfg);
    }
#if defined(HAS_PFACES_SDK) && HAS_PFACES_SDK
    if (mode == "direct") {
        DirectSynthesis::Params p;
        p.cfg_path    = extra_path;
        p.kernel_pack = "../../kernel-pack";
        p.device_id   = 1;
        return std::make_unique<DirectSynthesis>(p);
    }
    if (mode == "fast_tt") {
        FastTTInlineSynthesis::Params p;
        p.cfg_path    = extra_path;
        p.kernel_pack = "../../kernel-pack";
        p.device_id   = 1;
        return std::make_unique<FastTTInlineSynthesis>(p);
    }
#endif
    throw std::runtime_error("Unknown synthesis mode: " + mode);
}

}  // namespace rt_ctrl
