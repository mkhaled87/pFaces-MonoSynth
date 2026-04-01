/*
* pfacesKernel_mono_synth.h
*
*  created on: 01.10.2025
*      author: M. Khaled
*/

#pragma once

#include "configReader.h"

namespace mono_synth {

// Kernel name and scopes
#define KERNEL_NAME_MONO_SYNTH "mono_synth"

// Kernel function names and their arguments
#define KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NAME "precompute_transitions"
#define KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_IDX 0
#define KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NUM_ARGS 2
#define KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNCARG_NEXT_STATE_TABLE_NAME "next_state_table"
#define KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNCARG_NEXT_STATE_TABLE_IDX 0
#define KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNCARG_RUNTIME_PARAMS_NAME "runtime_params"
#define KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNCARG_RUNTIME_PARAMS_IDX 1

// Safety check kernel
#define KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_NAME "check_basis_safety"
#define KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_IDX 1
#define KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_NUM_ARGS 5
#define KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_BASIS_FLAT_IDX_NAME "basis_flat_idx"
#define KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_BASIS_FLAT_IDX_IDX 0
#define KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_NEXT_STATE_TABLE_NAME "next_state_table"
#define KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_NEXT_STATE_TABLE_IDX 1
#define KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_BASIS_LIST_NAME "basis_list"
#define KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_BASIS_LIST_IDX 2
#define KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_BASIS_LIST_SIZE_NAME "basis_list_size_ptr"
#define KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_BASIS_LIST_SIZE_IDX 3
#define KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_UNSAFE_FLAGS_NAME "unsafe_flags"
#define KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_UNSAFE_FLAGS_IDX 4

// Bitmap construction kernel (GPU)
#define KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNC_NAME "build_bitmap"
#define KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNC_IDX 2
#define KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNC_NUM_ARGS 3
#define KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNCARG_BITMAP_NAME "bitmap"
#define KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNCARG_BITMAP_IDX 0
#define KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNCARG_BASIS_LIST_NAME "basis_list"
#define KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNCARG_BASIS_LIST_IDX 1
#define KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNCARG_BASIS_LIST_SIZE_NAME "basis_list_size_ptr"
#define KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNCARG_BASIS_LIST_SIZE_IDX 2

// TT-only column update kernel (GPU)
#define KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNC_NAME "tt_only_column_update"
#define KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNC_IDX 3
#define KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNC_NUM_ARGS 5
#define KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNCARG_NEXT_STATE_TABLE_IDX 0
#define KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNCARG_TT_IN_IDX 1
#define KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNCARG_TT_OUT_IDX 2
#define KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNCARG_CHANGED_FLAG_IDX 3
#define KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNCARG_RUNTIME_PARAMS_IDX 4

// TT-only prefix-max sweep kernel (GPU)
#define KERNEL_MONO_SYNTH_TT_PREFIX_MAX_FUNC_NAME "tt_only_prefix_max"
#define KERNEL_MONO_SYNTH_TT_PREFIX_MAX_FUNC_IDX 4
#define KERNEL_MONO_SYNTH_TT_PREFIX_MAX_FUNC_NUM_ARGS 2
#define KERNEL_MONO_SYNTH_TT_PREFIX_MAX_FUNCARG_TT_IDX 0
#define KERNEL_MONO_SYNTH_TT_PREFIX_MAX_FUNCARG_SWEEP_PARAMS_IDX 1

// DataPool index for the GPU bitmap buffer
#define BITMAP_DATA_POOL_IDX 6


/**********************************************************/
/** pfacesKernel_mono_synth *************************************/
/**********************************************************/
class pfacesKernel_mono_synth : public pfaces2DKernel {
private:
  const char*  cache_file = "transition_cache.bin";


  /* the jobs referenced in the parallel program and the tune program*/
  std::vector<std::shared_ptr<pfacesDeviceExecuteJob>> job_execPrecomputeTransition;
  std::shared_ptr<pfacesDeviceReadJob> job_readNextStateTable;
  std::shared_ptr<pfacesDeviceWriteJob> job_writeNextStateTable;
  std::shared_ptr<pfacesDeviceWriteJob> job_writeRuntimeParams;

  /* Safe Set Jobs */
  std::vector<std::shared_ptr<pfacesDeviceExecuteJob>> job_execCheckBasisSafety;
  std::shared_ptr<pfacesDeviceReadJob> job_readUnsafeFlags;
  std::shared_ptr<pfacesDeviceWriteJob> job_writeBasisFlatIdx;
  std::shared_ptr<pfacesDeviceWriteJob> job_writeBasisList;
  std::shared_ptr<pfacesDeviceWriteJob> job_writeBasisListSize;

  /* GPU Bitmap Jobs */
  std::vector<std::shared_ptr<pfacesDeviceExecuteJob>> job_execBuildBitmap;
  std::shared_ptr<pfacesDeviceReadJob> job_readBitmap;

  /* GPU TT-only Jobs */
  std::vector<std::shared_ptr<pfacesDeviceExecuteJob>> job_execTTColumnUpdate;
  std::vector<std::vector<std::shared_ptr<pfacesDeviceExecuteJob>>> job_execTTPrefixMax; // per key dim
  std::shared_ptr<pfacesDeviceWriteJob> job_writeTTIn;
  std::shared_ptr<pfacesDeviceReadJob> job_readTTOut;
  std::shared_ptr<pfacesDeviceWriteJob> job_writeChangedFlag;
  std::shared_ptr<pfacesDeviceReadJob> job_readChangedFlag;
  std::vector<std::shared_ptr<pfacesDeviceWriteJob>> job_writeSweepParams; // per key dim

  /* instructions for the parallel program */
  std::vector<std::shared_ptr<pfacesInstruction>> instructionList;
  std::shared_ptr<pfacesInstruction> instr_BlockingSyncPoint = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_readNextStateTable = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_writeNextStateTable = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_writeRuntimeParams = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_hostFuncSaveTransitions = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_hostFuncLoadNextStateTable = std::make_shared<pfacesInstruction>();
  
  /* Safe Set Instructions */
  std::shared_ptr<pfacesInstruction> instr_hostFuncInitSafeSet = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_hostFuncPrepareSafeSetIteration = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_hostFuncProcessSafeSetUpdate = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_jumpToSafeSetStart = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_readUnsafeFlags = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_writeBasisFlatIdx = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_writeBasisList = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_writeBasisListSize = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_logOff = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_logOn = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_hostFuncBenchmarkStart = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_hostFuncBenchmarkNext = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_jumpToBenchmarkStart = std::make_shared<pfacesInstruction>();

  /* GPU Bitmap Instructions */
  std::shared_ptr<pfacesInstruction> instr_readBitmap = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_hostFuncPrepareBitmapGPU = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_hostFuncCopyBitmapFromGPU = std::make_shared<pfacesInstruction>();

  /* GPU TT-only Instructions */
  std::shared_ptr<pfacesInstruction> instr_writeTTIn = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_readTTOut = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_writeChangedFlag = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_readChangedFlag = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_writeSweepParams = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_hostFuncInitTTGPU = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_hostFuncPrepareTTGPUIteration = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_hostFuncProcessTTGPUUpdate = std::make_shared<pfacesInstruction>();

  size_t x_flat_width;

public:
  /* constructor */
  pfacesKernel_mono_synth(const std::shared_ptr<pfacesKernelLaunchState>& spLaunchState, const std::shared_ptr<pfacesConfigurationReader>& spCfg);

  /* destructor */
  ~pfacesKernel_mono_synth() {
    if (m_basis_csv_file.is_open()) {
      m_basis_csv_file.close();
    }
    if (m_iteration_stats_csv_file.is_open()) {
      m_iteration_stats_csv_file.close();
    }
  }

  /* pFaces program configuration */
  void configureParallelProgram(pfacesParallelProgram& parallelProgram);

  /* pFaces program tuning configuration */
  void configureTuneParallelProgram(pfacesParallelProgram& tuneParallelProgram, size_t targetFunctionIdx);

  /* a post-back function to save/load the controller/abstraction after the kernel finishes */
  static size_t saveTransitionTable(void* pPackedKernel, void* pPackedParallelProgram);
  static size_t loadTransitionTable(void* pPackedKernel, void* pPackedParallelProgram);
  
  /* host functions for safe set iteration */
  static size_t initSafeSet(void* pPackedKernel, void* pPackedParallelProgram);
  static size_t prepareSafeSetIteration(void* pPackedKernel, void* pPackedParallelProgram);
  static size_t processSafeSetUpdate(void* pPackedKernel, void* pPackedParallelProgram);
  static size_t benchmarkStart(void* pPackedKernel, void* pPackedParallelProgram);
  static size_t benchmarkNext(void* pPackedKernel, void* pPackedParallelProgram);
  static size_t prepareBitmapGPU(void* pPackedKernel, void* pPackedParallelProgram);
  static size_t copyBitmapFromGPU(void* pPackedKernel, void* pPackedParallelProgram);
  
  /* host functions for GPU TT-only iteration */
  static size_t initTTGPU(void* pPackedKernel, void* pPackedParallelProgram);
  static size_t prepareTTGPUIteration(void* pPackedKernel, void* pPackedParallelProgram);
  static size_t processTTGPUUpdate(void* pPackedKernel, void* pPackedParallelProgram);
  static size_t setSweepParams(void* pPackedKernel, void* pPackedParallelProgram);
  static size_t checkSkipPrecompute(void* pPackedKernel, void* pPackedParallelProgram);
  static size_t timerAfterPrecompute(void* pPackedKernel, void* pPackedParallelProgram);
  static size_t timerAfterGFP(void* pPackedKernel, void* pPackedParallelProgram);

  /* internal helper for safe set */
  int flattenIndex(const int* idx) const;
  int updateSafeSet(int* unsafe_flags);
  void rebuildCoordIndex();
  void buildThresholdTable();
  void cpuSafetyCheck(const int* next_state_table, int* unsafe_flags);

  /* public accessors for direct SDK integration */
  const std::vector<uint8_t>& getBitmap() const { return m_bitmap; }
  int getBitmapSize() const { return (int)x_flat_width; }
  int getBasisSize() const { return m_safe_set_size; }
  const int* getBasisData() const { return m_safe_set_basis; }
  int getStateDim() const { return m_ss_dim; }
  const std::vector<cl_ulong>& getGridSizes() const { return X_widthPerDimension; }
  bool isUseTTOnlyGPU() const { return m_use_tt_only && m_use_tt_only_gpu; }
  bool isUseInlineDynamics() const { return m_use_inline_dynamics; }

  /* direct-mode controls */
  void setSkipCache(bool v) { m_skip_cache = v; }
  void setBenchmarkCount(int c) { m_benchmark_count = c; }
  void setRecordBasisEvolution(bool v) { m_record_basis_evolution = v; }
  void setWriteIterationStats(bool v) {
    if (!v && m_iteration_stats_csv_file.is_open())
      m_iteration_stats_csv_file.close();
  }
  void setRuntimeParam0(float value) { m_runtime_param0 = value; }
  float getRuntimeParam0() const { return m_runtime_param0; }
  void setSkipBitmapBuild(bool v) { m_skip_bitmap_build = v; }
  void setSkipPrecompute(bool v) { m_skip_precompute = v; }

  /* threshold table accessors for direct RT controller integration */
  const std::vector<int>& getThresholdTable() const { return m_threshold_table; }
  int getThresholdTableSize() const { return m_threshold_table_size; }
  int getThresholdDStar() const { return m_threshold_d_star; }
  const std::vector<int>& getThresholdKeyStrides() const { return m_threshold_orig_to_key_stride; }
  int getSafeCellCount() const {
    int total = 0;
    for (int i = 0; i < m_threshold_table_size; ++i) total += m_threshold_table[i];
    return total;
  }

  /* Phase timing (filled by host functions in instruction list) */
  double m_precompute_ms = 0.0;
  double m_gfp_total_ms = 0.0;
  double m_bitmap_total_ms = 0.0;
  std::chrono::high_resolution_clock::time_point m_phase_timer;

  /* safe set state - will be initialized from config */
  int MAX_BASIS_ELEMENTS;
  int MAX_STATE_DIM;
  int MAX_COORD_VALUE;
  int MAX_BUCKET_SIZE;
  
  /* safe set data */
  int* m_safe_set_basis = nullptr;
  int* m_safe_set_flat_indices = nullptr;
  int m_safe_set_size = 0;
  int m_iterations = 0;
  int m_ss_dim = 2;
  std::chrono::high_resolution_clock::time_point m_compute_start;
  std::chrono::high_resolution_clock::time_point m_iteration_start;
  std::chrono::high_resolution_clock::time_point m_bitmap_build_start;
  
  // Basis evolution recording
  bool m_record_basis_evolution = false;
  std::ofstream m_basis_csv_file;
  std::vector<char> m_basis_csv_buf;  // large stream buffer to reduce syscalls
  std::ofstream m_iteration_stats_csv_file;
  // Threshold table evolution recording (TT-only mode)
  std::ofstream m_threshold_csv_file;
  std::vector<char> m_threshold_csv_buf;
  void writeThresholdCSV(int iteration);
  int m_iter_basis_in = 0;
  int m_iter_basis_out = 0;
  int m_iter_unsafe_count = 0;
  int m_iter_neighbor_count = 0;
  int m_iter_added_count = 0;
  double m_iter_clear_ms = 0.0;
  double m_iter_neighbor_gen_ms = 0.0;
  double m_iter_compact_ms = 0.0;
  double m_iter_rebuild_ms = 0.0;
  double m_iter_add_neighbors_ms = 0.0;
  double m_iter_csv_io_ms = 0.0;
  double m_iter_safety_phase_ms = 0.0;
  double m_iter_tt_build_ms = 0.0;
  double m_iter_tt_lookup_ms = 0.0;
  double m_iter_update_total_ms = 0.0;
  
  /* coordinate index for O(1) redundancy lookup */
  std::vector<std::vector<std::vector<int>>> m_coord_buckets;
  std::vector<std::vector<int>> m_bucket_sizes;
  
  /* persistent work buffers - reused across iterations */
  std::vector<unsigned char> m_unsafe_mask;
  std::vector<unsigned char> m_seen_neighbors;
  std::vector<int> m_neighbor_buffer;
  std::vector<int> m_neighbor_parent_dim;
  std::vector<int> m_neighbor_parent_coord;
  
  /* benchmark results */
  int m_benchmark_count = 10;
  int m_benchmark_current_run = 0;
  double m_benchmark_total_time_ms = 0;
  int m_benchmark_total_iterations = 0;

    /* safe-set bitmap from GPU build_bitmap kernel; direct copy (0-based,
      row-major, same convention as kernel and rt_controller) */
  std::vector<uint8_t> m_bitmap;
  double m_bitmap_build_total_ms = 0.0;
  
  /* Threshold table for CPU-side O(1) safety check */
  bool m_use_threshold_table = true;
  std::vector<int> m_threshold_table;
  int m_threshold_d_star = 0;
  int m_threshold_table_size = 0;
  std::vector<int> m_threshold_key_dims;
  std::vector<int> m_threshold_key_dim_indices;
  std::vector<int> m_threshold_orig_to_key_stride;
  
  /* TT-only iteration mode */
  bool m_use_tt_only = false;
  bool m_use_tt_only_gpu = true;
  bool m_use_inline_dynamics = false;
  bool m_tt_only_changed = false;
  void ttOnlyOneIteration(const int* next_state_table);
  void extractBasisFromThresholdTable();
  
  /* GPU TT-only iteration state */
  int m_tt_gpu_iteration = 0;
  bool m_tt_gpu_ping = true;  // ping-pong: true = in→out, false = out→in
  int m_tt_gpu_sweep_dim_idx = 0;  // which key dim to sweep next
  
  /* direct-mode flags */
  bool m_skip_cache = false;
  bool m_skip_bitmap_build = false;
  bool m_skip_precompute = false;
  float m_runtime_param0 = 0.0f;  // 0 = use compile-time default from dynamics file

  /* providing implementation of the virtual method: getParameterList*/
  std::pair<std::vector<std::string>, std::vector<std::string>> getParameterList();

  /* the configuration reader */
  const std::shared_ptr<configReader> m_spCfg;

  /* the scope of the kernel */
  std::string m_kernelScope;

  /* problem size : X cardinality for each dimension */
  std::vector<cl_ulong> X_widthPerDimension;

  /* per-dimension priorities (from .cfg): 0 = max-good, 1 = min-good */
  std::vector<int> m_priorities;
};

}  // namespace mono_synth
