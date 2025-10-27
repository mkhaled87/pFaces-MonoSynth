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
#define KERNEL_SCOPE_GPU "gpu"
#define KERNEL_SCOPE_CMEM "cmem"
#define KERNEL_SCOPE_GMEM "gmem"

// Kernel function names and their arguments
#define KERNEL_MONO_SYNTH_ASTRACT_FUNC_NAME "abstract"
#define KERNEL_MONO_SYNTH_ASTRACT_FUNC_IDX 0
#define KERNEL_MONO_SYNTH_ASTRACT_FUNC_NUM_ARGS 3
#define KERNEL_MONO_SYNTH_ASTRACT_FUNCARG_XUBAG_GLOBAL 0
#define KERNEL_MONO_SYNTH_ASTRACT_FUNCARG_XUBAG_LOCAL 1
#define KERNEL_MONO_SYNTH_ASTRACT_FUNCARG_ROBAG 2

// Parameters names used in the output file
#define OUT_FILE_PARAM_SAMPLING_PERIOD "sampling-period"
#define OUT_FILE_PARAM_SS_DIMENSION "ss-dimension"
#define OUT_FILE_PARAM_IS_DIMENSION "is-dimension"
#define OUT_FILE_PARAM_SS_ETA "ss-eta"
#define OUT_FILE_PARAM_SS_ERR "ss-err"
#define OUT_FILE_PARAM_SS_STEPS "ss-steps"
#define OUT_FILE_PARAM_IS_ETA "is-eta"
#define OUT_FILE_PARAM_IS_ERR "is-err"
#define OUT_FILE_PARAM_IS_STEPS "is-steps"
#define OUT_FILE_PARAM_SS_LB "ss-lower-point"
#define OUT_FILE_PARAM_SS_UB "ss-upper-point"
#define OUT_FILE_PARAM_IS_LB "is-lower-point"
#define OUT_FILE_PARAM_IS_UB "is-upper-point"
#define OUT_FILE_PARAM_POST_USE_ODE "post-useode"
#define OUT_FILE_PARAM_RADIUS_USE_ODE "radius-useode"
#define OUT_FILE_PARAM_X_WIDTH "x-width"
#define OUT_FILE_PARAM_U_WIDTH "u-width"
#define OUT_FILE_PARAM_XU_BAG_SIZE "xu-bag-size"
#define OUT_FILE_PARAM_CONTENT "content"
#define OUT_FILE_CONTENT_ABSTRACTION "abstraction"
#define OUT_FILE_CONTENT_CONTROLLER "controller"
#define OUT_FILE_CONTENT_ABSTRACTION_AND_CONTROLLER "abstraction+controller"
#define OUT_FILE_PARAM_TARGET_SETS "target-sets"
#define OUT_FILE_PARAM_AVOID_SETS "avoid-sets"
#define OUT_FILE_PARAM_SAFE_SETS "safe-sets"
#define OUT_FILE_PARAM_CONCRETE_NAME "concrete-data-name"
#define OUT_FILE_PARAM_CONCRETE_SIZE "concrete-data-size"
#define OUT_FILE_PARAM_SYMBOLIC_NAME "symbolic-data-name"
#define OUT_FILE_PARAM_SYMBOLIC_SIZE "symbolic-data-size"
#define OUT_FILE_PARAM_SS_STATE_VAR_DYNAMICS "xx%NUM%-dynamics"
#define OUT_FILE_PARAM_SS_STATE_VAR_NUMBER_TOKEN "%NUM%"
#define OUT_FILE_PARAM_USED_KERNEL_NAME "used-kernel-name"

/* The class for the xu_bag define as a strunct in kernels/abstract_growthbound.cl */
#define RW_BAG_CONTROLLER_BIT_INDEX_IN_FLAGS 0
class RW_bag {
public:
  // Data elements as appearing in the RW_bag
  concrete_t* cnc_dest_states_lb;  // per state
  concrete_t* cnc_dest_states_ub;  // per state
  char flags;

  // Data elemets needed by the instance
  size_t m_ssDim;
  size_t m_numMaskChunks;

  // A static function to get the size generically
  static inline size_t getSizeBytes(std::vector<double> params, bool isMemoryEfficient) {

    // in memory efficient version, only flags are there !
    if (isMemoryEfficient)
      return sizeof(flags);

    return 2 * concrete_t_size * (size_t)params[0]  // for: cnc_dest_states_lb + cnc_dest_states_ub
           + 1 * sizeof(flags);                     // for: flags
  }

  // instance methods
  RW_bag(size_t ssDim, size_t maxPostsCount) {
    m_ssDim = ssDim;
    m_numMaskChunks = maxPostsCount / (sizeof(cl_uint) * 8);
  }
  std::vector<concrete_t> getBagElement_LB(const char* pBag) {
    std::vector<concrete_t> ret;

    for (size_t i = 0; i < m_ssDim; i++)
      ret.push_back(((concrete_t*)pBag)[i]);

    return ret;
  }
  std::vector<concrete_t> getBagElement_UB(const char* pBag) {
    std::vector<concrete_t> ret;

    pBag += m_ssDim * sizeof(concrete_t);

    for (size_t i = 0; i < m_ssDim; i++)
      ret.push_back(((concrete_t*)pBag)[i]);

    return ret;
  }

  inline char getBagElement_FLAGS(const char* pBag) {
    pBag += 2 * m_ssDim * sizeof(concrete_t);
    return *pBag;
  }

  inline void setBagElement_FLAGS(char* pBag, const char val) {
    char* pFlags = pBag + 2 * m_ssDim * sizeof(concrete_t);
    *pFlags = val;
    ;
  }

  void printBagInfo(const char* pBag, bool isMemEfficeintMode) {

    size_t bagSize =
      getSizeBytes({(double)m_ssDim, (double)0, (double)m_numMaskChunks * (sizeof(cl_uint) * 8)}, isMemEfficeintMode);

    std::cout << "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;
    std::cout << "RW_Bag @" << (void*)pBag << " [" << bagSize << " bytes]" << std::endl;
    std::cout << "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;
    std::cout << "      LB: ";
    pfacesUtils::PrintVector(getBagElement_LB(pBag));
    std::cout << "      UB: ";
    pfacesUtils::PrintVector(getBagElement_UB(pBag));
    std::cout << "    FLAS: ";
    pfacesUtils::PrintAsBits(getBagElement_FLAGS(pBag));

    std::cout << "   BYTES: ";
    for (size_t i = 0; i < bagSize; i++)
      printf("%2X ", 0xFF & static_cast<unsigned int>(pBag[i]));

    std::cout << std::endl;
    ;
    std::cout << "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;
  }
};
class RO_bag {
public:
  /* problem bases */
  flat_t grid_base_x;
  flat_t grid_base_u;

  static inline size_t getSizeBytes(std::vector<double> params) {
    /* params[0] = flat_type_size*/
    return 2 * (size_t)params[0];
  }

  static void fillData(std::vector<char>& toFill, flat_t& grid_base_x, flat_t& grid_base_u, size_t bigint_size_bytes) {

    if (toFill.size() != getSizeBytes({(double)bigint_size_bytes}))
      throw std::runtime_error("RO_bag::fillData: invalid data size!");

    for (size_t i = 0; i < bigint_size_bytes; i++) {
      toFill[i] = ((char*)grid_base_x.getBlkArray())[i];
      toFill[bigint_size_bytes + i] = ((char*)grid_base_u.getBlkArray())[i];
    }
  }
};


/**********************************************************/
/** pfacesKernel_mono_synth *************************************/
/**********************************************************/
class pfacesKernel_mono_synth : public pfaces2DKernel {
private:
  const char* param_max_posts = "@@MAX_POSTS_COUNT@@";

  const char* param_use_ode_post = "@@DEFINE_USE_ODE_SOLVER_POST@@";
  const char* param_code_only_post = "@@DEFINE_CODE_ONLY_SOLVER_POST@@";
  const char* param_use_ode_radius = "@@DEFINE_USE_ODE_SOLVER_RADIUS@@";
  const char* param_code_only_radius = "@@DEFINE_CODE_ONLY_SOLVER_RADIUS@@";

  const char* param_bigint_size = "@@BIGINT_SIZE@@";

  const char* param_concrete_type = "@@CONCRETE_DATA_TYPE@@";
  const char* param_symbolic_type = "@@SYMBOLIC_DATA_TYPE@@";
  const char* param_flat_type = "@@FLAT_DATA_TYPE@@";

  const char* param_def_use_double = "@@DEFINE_USE_DOUBLE_PRECISION@@";

  const char* param_ss_dim = "@@SSDIM@@";
  const char* param_is_dim = "@@ISDIM@@";

  const char* param_ss_eta = "@@SS_ETA@@";
  const char* param_ss_lb = "@@SS_LB@@";
  const char* param_ss_ub = "@@SS_UB@@";
  const char* param_ss_err = "@@SS_ERR@@";
  const char* param_is_eta = "@@IS_ETA@@";
  const char* param_is_lb = "@@IS_LB@@";
  const char* param_is_ub = "@@IS_UB@@";
  const char* param_is_err = "@@IS_ERR@@";

  const char* param_tau = "@@SAMPLING_PERIOD@@";

  const char* param_extra_include = "@@EXTRA_INCLUDE@@";
  const char* param_postdynamics_initcode = "@@POST_DYNAMICS_INIT_CODE@@";
  const char* param_postdynamics_finishcode = "@@POST_DYNAMICS_FINISH_CODE@@";
  const char* param_radiusdynamics_initcode = "@@RADIUS_DYNAMICS_INIT_CODE@@";
  const char* param_radiusdynamics_finishcode = "@@RADIUS_DYNAMICS_FINISH_CODE@@";

  const char* param_postdynamics = "@@POST_DYNAMICS@@";
  const char* param_radiusdynamics = "@@RADIUS_DYNAMICS@@";

  const char* param_has_target = "@@DEFINE_HAS_TARGET@@";
  const char* param_target_count = "@@TARGET_COUNT@@";
  const char* param_target_data = "@@TARGET_DATA@@";

  const char* param_has_safe = "@@DEFINE_HAS_SAFE@@";
  const char* param_safe_count = "@@SAFE_COUNT@@";
  const char* param_safe_data = "@@SAFE_DATA@@";

  const char* param_has_avoid = "@@DEFINE_HAS_AVOID@@";
  const char* param_avoid_count = "@@AVOID_COUNT@@";
  const char* param_avoid_data = "@@AVOID_DATA@@";

  flat_t x_flat_width;
  flat_t u_flat_width;

  size_t bigint_size;
  size_t bigint_size_bytes;

  /* the jobs referenced in the parallel program and the tune program*/
  std::vector<std::shared_ptr<pfacesDeviceExecuteJob>> perDevAbstractionJobs;
  std::shared_ptr<pfacesDeviceReadJob> readAllDataJob, readResultsBufferJob;
  std::shared_ptr<pfacesDeviceWriteJob> writeAllDataJob, writeResultsBufferJob;

  /* per-job per-task sub-buffer info for the jobs referenced in the parallel program*/
  std::vector<std::pair<size_t, size_t>> perDevAbstractionJob_XUBAG_LOCAL_SubBuffers;

  /* instructions for the parallel program */
  std::vector<std::shared_ptr<pfacesInstruction>> instructionList;
  std::shared_ptr<pfacesInstruction> instr_BlockingSyncPoint = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_LogOn = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_LogOff = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_readAllData = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_writeAllData = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_MsgAbsComplete = std::make_shared<pfacesInstruction>();
  std::shared_ptr<pfacesInstruction> instr_runSynthesis = std::make_shared<pfacesInstruction>();

  /* post-execute function and parals */
  std::vector<std::shared_ptr<void>> postExecuteParams;

  /* host-side synthesis function (runs after abstraction) */
  static size_t runMonotoneSynthesis(void* pPackedKernel, void* pPackedParallelProgram);

public:
  /* constructor */
  pfacesKernel_mono_synth(const std::shared_ptr<pfacesKernelLaunchState>& spLaunchState, const std::shared_ptr<pfacesConfigurationReader>& spCfg);

  /* destructor */
  ~pfacesKernel_mono_synth() = default;

  /* pFaces program configuration */
  void configureParallelProgram(pfacesParallelProgram& parallelProgram);

  /* pFaces program tuning configuration */
  void configureTuneParallelProgram(pfacesParallelProgram& tuneParallelProgram, size_t targetFunctionIdx);

  /* a post-back function to save the controller/abstraction after the kernel finishes */
  static size_t saveData(const pfaces2DKernel& thisKernel,  const pfacesParallelProgram& thisParallelProgram, std::vector<std::shared_ptr<void>>& postExecuteParamsList);

  /* providing implementation of the virtual method: getParameterList*/
  std::pair<std::vector<std::string>, std::vector<std::string>> getParameterList();

  /* the configuration reader */
  const std::shared_ptr<configReader> m_spCfg;

  /* the scope of the kernel */
  std::string m_kernelScope;

  /* problem size : X cardinality for each dimension */
  std::vector<symbolic_t> X_widthPerDimension;

  /* problem size : U cardinality for each dimension */
  std::vector<symbolic_t> U_widthPerDimension;
};

}  // namespace mono_synth
