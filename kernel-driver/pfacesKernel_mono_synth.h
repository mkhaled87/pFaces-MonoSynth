#pragma once

#include "basis_store.h"
#include "configReader.h"

#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace mono_synth {

constexpr const char* KERNEL_NAME_MONO_SYNTH = "mono_synth";

enum class KernelFunction : std::size_t {
  PRECOMPUTE_TRANSITIONS = 0,
  PREDECESSOR_SCAN = 1,
  PREDECESSOR_THRESHOLD = 2,
  THRESHOLD_CLEAR = 3,
  THRESHOLD_SCATTER = 4,
  THRESHOLD_PREFIX = 5,
  THRESHOLD_GFP_STEP = 6,
  BITMAP_GFP_ITERATE = 7,
  BITMAP_GFP_ADVANCE = 8,
  BITMAP_GFP_PREFIX = 9
};

constexpr std::size_t functionIndex(KernelFunction function) {
  return static_cast<std::size_t>(function);
}

enum class MembershipKind { SCAN, THRESHOLD };

struct SolverStats {
  SynthesisMethod method = SynthesisMethod::THRESHOLD;
  std::uint64_t cdc_epochs = 0;
  std::uint64_t cdc_mutations = 0;
  std::uint64_t gfp_rounds = 0;
  std::uint64_t frontier_batches = 0;
  std::uint64_t membership_queries = 0;
  std::uint64_t binary_search_probes = 0;
  std::uint64_t allocated_bytes = 0;
  double transition_ms = 0.0;
  double membership_ms = 0.0;
  double representation_ms = 0.0;
  double basis_update_ms = 0.0;
  double solver_ms = 0.0;
};

class pfacesKernel_mono_synth : public pfaces2DKernel {
 public:
  pfacesKernel_mono_synth(
      const std::shared_ptr<pfacesKernelLaunchState>& launch_state,
      const std::shared_ptr<pfacesConfigurationReader>& configuration);
  ~pfacesKernel_mono_synth();

  void configureParallelProgram(pfacesParallelProgram& parallel_program);
  void configureTuneParallelProgram(pfacesParallelProgram& parallel_program,
                                    std::size_t target_function);

  const SolverStats& getSolverStats() const { return stats_; }
  SynthesisMethod getSynthesisMethod() const { return method_; }
  TransitionBackend getTransitionBackend() const { return backend_; }
  std::pair<std::vector<std::string>, std::vector<std::string>>
  getOpenClParameterList() { return getParameterList(); }

  int getBasisSize() const { return static_cast<int>(basis_coordinates_.size() / state_dimension_); }
  const int* getBasisData() const { return basis_coordinates_.data(); }
  int getStateDim() const { return static_cast<int>(state_dimension_); }
  const std::vector<cl_ulong>& getGridSizes() const { return grid_widths_cl_; }
  const std::vector<cl_uint>& getBitmapWords() const { return bitmap_words_; }
  std::size_t getBitmapWordCount() const { return bitmap_words_.size(); }
  std::size_t getBitmapSize() const { return static_cast<std::size_t>(total_states_); }

  void setSkipCache(bool value) { skip_cache_ = value; }
  void setRuntimeParam0(float value) { runtime_param0_ = value; }
  float getRuntimeParam0() const { return runtime_param0_; }
  void setExtractBasis(bool value) { extract_basis_ = value; }

  const std::vector<Height>& getThresholdTable() const { return threshold_result_u32_; }
  TableOffset getThresholdTableSize() const { return threshold_table_size_; }
  int getThresholdDStar() const { return static_cast<int>(threshold_d_star_); }
  const std::vector<TableOffset>& getThresholdKeyStrides() const { return threshold_key_strides_; }
  std::int64_t getSafeCellCount() const;

  double m_precompute_ms = 0.0;
  double m_gfp_total_ms = 0.0;
  int m_iterations = 0;

 private:
  // pFaces data-pool indices. Resident arguments do not allocate a new entry.
  enum Pool : std::size_t {
    POOL_NEXT_STATE = 0,
    POOL_RUNTIME_PARAMS = 1,
    POOL_SCAN_QUERY = 2,
    POOL_SCAN_QUERY_COUNT = 3,
    POOL_SCAN_TARGET_BASIS = 4,
    POOL_SCAN_TARGET_SIZE = 5,
    POOL_SCAN_FLAGS = 6,
    POOL_THRESHOLD_QUERY = 7,
    POOL_THRESHOLD_QUERY_COUNT = 8,
    POOL_AUTOMATICA_THRESHOLD = 9,
    POOL_THRESHOLD_FLAGS = 10,
    POOL_CONVERSION_BASIS = 11,
    POOL_CONVERSION_BASIS_SIZE = 12,
    POOL_PREFIX_PARAMS = 13,
    POOL_GFP_A = 14,
    POOL_GFP_B = 15,
    POOL_GFP_PARITY = 16,
    POOL_GFP_CHANGED = 17,
    POOL_BITMAP_IN = 18,
    POOL_BITMAP_OUT = 19,
    POOL_BITMAP_CHANGED = 20,
    POOL_BITMAP_PREFIX_PARAMS = 21
  };

  std::pair<std::vector<std::string>, std::vector<std::string>> getParameterList();
  void registerKernelFunctions(const std::string& pack_path);

  void appendCommonPrecompute(pfacesParallelProgram& program,
                              pfacesParallelAdvisor& advisor,
                              cl::NDRange offset);
  void appendCdcSchedule(pfacesParallelProgram& program,
                         pfacesParallelAdvisor& advisor,
                         cl::NDRange offset);
  void appendAutomaticaSchedule(pfacesParallelProgram& program,
                                pfacesParallelAdvisor& advisor,
                                cl::NDRange offset,
                                MembershipKind membership);
  void appendThresholdSchedule(pfacesParallelProgram& program,
                               pfacesParallelAdvisor& advisor,
                               cl::NDRange offset);
  void appendReferenceSchedule(pfacesParallelProgram& program,
                               pfacesParallelAdvisor& advisor,
                               cl::NDRange offset);
  void appendCommonFinalize(pfacesParallelProgram& program);

  static size_t hostInitCdc(void*, void*);
  static size_t hostPrepareCdc(void*, void*);
  static size_t hostProcessCdc(void*, void*);
  static size_t hostInitAutomatica(void*, void*);
  static size_t hostPrepareAutomaticaOuter(void*, void*);
  static size_t hostPrepareAutomaticaBatch(void*, void*);
  static size_t hostProcessAutomaticaBatch(void*, void*);
  static size_t hostFinishAutomaticaOuter(void*, void*);
  static size_t hostInitThreshold(void*, void*);
  static size_t hostPrepareThresholdRound(void*, void*);
  static size_t hostProcessThresholdRound(void*, void*);
  static size_t hostFinalThresholdIsB(void*, void*);
  static size_t hostFinalizeThreshold(void*, void*);
  static size_t hostRunCpuReference(void*, void*);
  static size_t hostInitBitmapReference(void*, void*);
  static size_t hostPrepareBitmapReference(void*, void*);
  static size_t hostSetBitmapPrefixParams(void*, void*);
  static size_t hostProcessBitmapReference(void*, void*);
  static size_t hostFinalizeBitmapReference(void*, void*);
  static size_t hostFinalizeSolver(void*, void*);
  static size_t hostSetPrefixParams(void*, void*);
  static size_t hostFinishRepresentation(void*, void*);
  static size_t hostStartPrecomputeTimer(void*, void*);
  static size_t hostStopPrecomputeTimer(void*, void*);

  static size_t saveTransitionTable(void*, void*);
  static size_t loadTransitionTable(void*, void*);

  void resetStatsAndTimer();
  void stopSolverTimer();
  void setCanonicalResult(const BasisStore& basis);
  void setCanonicalResult(const std::vector<Height>& threshold);
  void uploadBasis(pfacesParallelProgram& program, const BasisStore& query,
                   const BasisStore& target, MembershipKind membership,
                   bool upload_target);
  void initializeThresholdGeometry();
  std::vector<Height> cpuThresholdStep(const std::vector<Height>& source,
                                       const GridIndex* transitions,
                                       std::uint64_t* probes) const;
  void extractBasisFromThreshold();

  std::string cache_file_ = "transition_cache_v2_u64.bin";
  std::uint64_t cache_fingerprint_ = 0;
  std::shared_ptr<configReader> config_;
  SynthesisMethod method_ = SynthesisMethod::THRESHOLD;
  TransitionBackend backend_ = TransitionBackend::PRECOMPUTED;
  std::size_t state_dimension_ = 0;
  std::size_t max_basis_elements_ = 0;
  GridIndex total_states_ = 0;
  std::vector<GridIndex> grid_widths_;
  std::vector<cl_ulong> grid_widths_cl_;
  std::size_t threshold_d_star_ = 0;
  TableOffset threshold_table_size_ = 0;
  std::vector<std::size_t> threshold_key_dimensions_;
  std::vector<TableOffset> threshold_key_strides_;
  std::vector<Height> threshold_result_u32_;
  std::vector<int> basis_coordinates_;
  std::vector<cl_uint> bitmap_words_;

  BasisStore current_basis_;
  BasisStore target_basis_;
  BasisStore work_basis_;
  BasisStore controlled_basis_;
  BasisStore frontier_basis_;
  MembershipKind automatica_membership_ = MembershipKind::SCAN;
  std::uint32_t gfp_parity_ = 0;
  std::uint32_t conversion_sweep_index_ = 0;
  std::uint32_t bitmap_sweep_index_ = 0;

  std::vector<std::shared_ptr<pfacesDeviceExecuteJob>> membership_jobs_;
  std::vector<std::shared_ptr<pfacesDeviceExecuteJob>> conversion_scatter_jobs_;
  SolverStats stats_;
  std::chrono::high_resolution_clock::time_point solver_timer_;
  std::chrono::high_resolution_clock::time_point phase_timer_;
  bool solver_timer_stopped_ = false;
  std::uint64_t allocated_bytes_ = 0;

  std::vector<std::shared_ptr<pfacesInstruction>> instructions_;
  bool skip_cache_ = false;
  bool extract_basis_ = true;
  float runtime_param0_ = 0.0f;
};

}  // namespace mono_synth
