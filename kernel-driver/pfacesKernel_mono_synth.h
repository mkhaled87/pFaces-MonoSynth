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
#define KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NUM_ARGS 1
#define KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNCARG_NEXT_STATE_TABLE_NAME "next_state_table"
#define KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNCARG_NEXT_STATE_TABLE_IDX 0

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

  /* Safe Set Jobs */
  std::vector<std::shared_ptr<pfacesDeviceExecuteJob>> job_execCheckBasisSafety;
  std::shared_ptr<pfacesDeviceReadJob> job_readUnsafeFlags;
  std::shared_ptr<pfacesDeviceWriteJob> job_writeBasisFlatIdx;
  std::shared_ptr<pfacesDeviceWriteJob> job_writeBasisList;
  std::shared_ptr<pfacesDeviceWriteJob> job_writeBasisListSize;

  /* instructions for the parallel program */
  std::vector<std::shared_ptr<pfacesInstruction>> instructionList;
  std::shared_ptr<pfacesInstruction> instr_BlockingSyncPoint = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_readNextStateTable = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_writeNextStateTable = std::make_shared<pfacesInstruction>();
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

  size_t x_flat_width;

public:
  /* constructor */
  pfacesKernel_mono_synth(const std::shared_ptr<pfacesKernelLaunchState>& spLaunchState, const std::shared_ptr<pfacesConfigurationReader>& spCfg);

  /* destructor */
  ~pfacesKernel_mono_synth() {
    if (m_basis_csv_file.is_open()) {
      m_basis_csv_file.close();
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

  /* internal helper for safe set */
  int flattenIndex(const int* idx) const;
  int updateSafeSet(int* unsafe_flags);
  void rebuildCoordIndex();

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
  
  // Basis evolution recording
  bool m_record_basis_evolution = false;
  std::ofstream m_basis_csv_file;
  
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

  /* providing implementation of the virtual method: getParameterList*/
  std::pair<std::vector<std::string>, std::vector<std::string>> getParameterList();

  /* the configuration reader */
  const std::shared_ptr<configReader> m_spCfg;

  /* the scope of the kernel */
  std::string m_kernelScope;

  /* problem size : X cardinality for each dimension */
  std::vector<cl_ulong> X_widthPerDimension;
};

}  // namespace mono_synth
