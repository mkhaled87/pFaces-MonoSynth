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


/**********************************************************/
/** pfacesKernel_mono_synth *************************************/
/**********************************************************/
class pfacesKernel_mono_synth : public pfaces2DKernel {
private:
  const char* param_ss_dim = "@@SS_DIM@@";
  const char* param_is_dim = "@@IS_DIM@@";
  const char* param_ss_eta = "@@SS_ETA@@";
  const char* param_ss_lb = "@@SS_LB@@";
  const char* param_ss_ub = "@@SS_UB@@";
  const char* param_tau = "@@SAMPLING_PERIOD@@";
  const char* param_extra_include = "@@EXTRA_INCLUDE@@";
  const char*  cache_file = "transition_cache.bin";


  /* the jobs referenced in the parallel program and the tune program*/
  std::vector<std::shared_ptr<pfacesDeviceExecuteJob>> job_execPrecomputeTransition;
  std::shared_ptr<pfacesDeviceReadJob> job_readNextStateTable;
  std::shared_ptr<pfacesDeviceWriteJob> job_writeNextStateTable;

  /* instructions for the parallel program */
  std::vector<std::shared_ptr<pfacesInstruction>> instructionList;
  std::shared_ptr<pfacesInstruction> instr_BlockingSyncPoint = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_readNextStateTable = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_writeNextStateTable = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_hostFuncSaveTransitions = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_hostFuncLoadNextStateTable = std::make_shared<pfacesInstruction>();
  

  size_t x_flat_width;

public:
  /* constructor */
  pfacesKernel_mono_synth(const std::shared_ptr<pfacesKernelLaunchState>& spLaunchState, const std::shared_ptr<pfacesConfigurationReader>& spCfg);

  /* destructor */
  ~pfacesKernel_mono_synth() = default;

  /* pFaces program configuration */
  void configureParallelProgram(pfacesParallelProgram& parallelProgram);

  /* pFaces program tuning configuration */
  void configureTuneParallelProgram(pfacesParallelProgram& tuneParallelProgram, size_t targetFunctionIdx);

  /* a post-back function to save/load the controller/abstraction after the kernel finishes */
  static size_t saveTransitionTable(void* pPackedKernel, void* pPackedParallelProgram);
  static size_t loadTransitionTable(void* pPackedKernel, void* pPackedParallelProgram);

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
