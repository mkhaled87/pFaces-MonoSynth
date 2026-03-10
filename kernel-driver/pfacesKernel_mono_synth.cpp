/*
* pfacesKernel_mono_synth.cpp
*
*  created on: 02.10.2025
*      author: M. Khaled
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <cstring>
#include <chrono>
#include <algorithm>

#include "pfacesKernel_mono_synth.h"

namespace mono_synth {


/* a call-back function to save the controller/abstraction after the kernel finishes */
size_t pfacesKernel_mono_synth::saveTransitionTable(void* pPackedKernel, void* pPackedParallelProgram) {
	
	// retrieving the required values
	const static pfacesParallelProgram*  pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;
	const char* pDataTransitionTable = pParallelProgram->m_dataPool[0].first;
	int ss_dim = ((pfacesKernel_mono_synth*)(pPackedKernel))->m_spCfg->getSsDim();
	int number_of_states = ((pfacesKernel_mono_synth*)(pPackedKernel))->x_flat_width;
	int number_of_elements = number_of_states*ss_dim;

	// save to file
	const char* file_path = ((pfacesKernel_mono_synth*)(pPackedKernel))->cache_file;
	std::cout << "Saving transitions to file: " << file_path << std::endl;
	std::ofstream cache_out(file_path, std::ios::binary);
	if (cache_out.good()) {
		cache_out.write(reinterpret_cast<const char*>(&number_of_states), sizeof(decltype(number_of_states)));
		cache_out.write(pDataTransitionTable, number_of_elements*sizeof(cl_int));
	}	

	return 0;
}


/* a call-back function to save the controller/abstraction after the kernel finishes */
size_t pfacesKernel_mono_synth::loadTransitionTable(void* pPackedKernel, void* pPackedParallelProgram) {
	
	// retrieving the required values
	const static pfacesParallelProgram*  pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;
	char* pDataTransitionTable = pParallelProgram->m_dataPool[0].first;

	// load from file
	const char* file_path = ((pfacesKernel_mono_synth*)(pPackedKernel))->cache_file;
	int ss_dim = ((pfacesKernel_mono_synth*)(pPackedKernel))->m_spCfg->getSsDim();
	int number_of_states = ((pfacesKernel_mono_synth*)(pPackedKernel))->x_flat_width;
	int number_of_elements = number_of_states*ss_dim;
	std::ifstream cache_in(file_path, std::ios::binary);
	if (cache_in.good()) {
		int cached_total_states;
		cache_in.read(reinterpret_cast<char*>(&cached_total_states), sizeof(int));
		
		if (cached_total_states == number_of_states) {
			cache_in.read(reinterpret_cast<char*>(pDataTransitionTable), sizeof(cl_int) * number_of_elements);
		}
		cache_in.close();
	}
	return 0;
}

/* getting the parameter list for the kernel */
std::pair<std::vector<std::string>, std::vector<std::string>> pfacesKernel_mono_synth::getParameterList() {
	
	std::vector<std::string> params;
	std::vector<std::string> paramvals;

	/* User dynamics code injection */
	std::string dynamics_file = m_spCfg->getUserDynamicsFile();
	std::string dynamics_code = "";
	if (!dynamics_file.empty()) {
		try {
			dynamics_code = pfacesFileIO::readTextFromFile(dynamics_file);
		} catch (...) {
			std::cerr << "[MonoSynth] Error: Could not read user dynamics file: " << dynamics_file << std::endl;
		}
	}
	params.push_back("@@USER_DYNAMICS_CODE@@");
	paramvals.push_back(dynamics_code);

    /* System Dimensions */
    params.push_back("@@STATE_DIM@@");
    paramvals.push_back(std::to_string(m_spCfg->getSsDim()));
    params.push_back("@@INPUT_DIM@@");
    paramvals.push_back(std::to_string(m_spCfg->getIsDim()));
    params.push_back("@@DISTURB_DIM@@");
    paramvals.push_back(std::to_string(m_spCfg->getDisturbDim()));
    params.push_back("@@TOTAL_STATES@@");
    paramvals.push_back(std::to_string(x_flat_width));

    /* Solver Config */
    params.push_back("@@ODE_STEPS@@");
    paramvals.push_back(std::to_string(m_spCfg->getOdeSteps()));
    params.push_back("@@SAMPLING_TIME@@");
    paramvals.push_back(std::to_string(m_spCfg->getSamplingPeriod()) + "f");

    /* State Space Arrays */
    auto lb = m_spCfg->getSsLb();
    auto ub = m_spCfg->getSsUb();
    auto eta = m_spCfg->getSsEta();
    auto pri = m_spCfg->getSsPriorities();

    std::stringstream ss_lb, ss_ub, ss_res, ss_pri;
    ss_lb << std::fixed << std::setprecision(6) << "{";
    ss_ub << std::fixed << std::setprecision(6) << "{";
    ss_res << std::fixed << std::setprecision(6) << "{";
    ss_pri << "{";
    
    for (size_t i = 0; i < m_spCfg->getSsDim(); ++i) {
        if (i > 0) {
            ss_lb << ","; ss_ub << ","; ss_res << ","; ss_pri << ",";
        }
        ss_lb << lb[i] << "f";
        ss_ub << ub[i] << "f";
        ss_res << eta[i] << "f";
        ss_pri << pri[i];
    }
    ss_lb << "}"; ss_ub << "}"; ss_res << "}"; ss_pri << "}";

    params.push_back("@@X_MIN_ARRAY@@");
    paramvals.push_back(ss_lb.str());
    params.push_back("@@X_MAX_ARRAY@@");
    paramvals.push_back(ss_ub.str());
    params.push_back("@@X_RES_ARRAY@@");
    paramvals.push_back(ss_res.str());
    params.push_back("@@X_PRIORITY_ARRAY@@");
    paramvals.push_back(ss_pri.str());

	return std::make_pair(params, paramvals);
}


/* constructor: initiate data and prepare memory maps*/
pfacesKernel_mono_synth::pfacesKernel_mono_synth(const std::shared_ptr<pfacesKernelLaunchState>& spLaunchState, const std::shared_ptr<pfacesConfigurationReader>& spCfg)
	: pfaces2DKernel(spLaunchState->getDefaultSourceFilePath(KERNEL_NAME_MONO_SYNTH)),
	  m_spCfg(std::make_shared<configReader>(spCfg)) {

	m_kernelScope = spLaunchState->getKernelScope();
	
	std::string packPath = spLaunchState->getKernelPackPath();
	if (!packPath.empty() && packPath.back() != '/' && packPath.back() != '\\') {
		packPath += "/";
	}

	// Initialize array limits from config
	MAX_BASIS_ELEMENTS = m_spCfg->getMaxBasisElements();
	// Auto-calculate MAX_STATE_DIM from actual state dimension
	MAX_STATE_DIM = m_spCfg->getSsDim();
	
	// Auto-calculate MAX_BUCKET_SIZE as heuristic
	// Typically, no more than ~2% of basis elements share a coordinate
	MAX_BUCKET_SIZE = std::max(64, MAX_BASIS_ELEMENTS / 5);
	
	// setting the dimensions of the base 2d kernel object
	const size_t ssDim = m_spCfg->getSsDim();
	const size_t isDim = m_spCfg->getIsDim();

	// Convert concrete space into flat space for X
	// MUST DO THIS FIRST to get X_widthPerDimension for MAX_COORD_VALUE calculation
	x_flat_width = pfacesFlatSpace::getFlatWidthFromConcreteSpace(
		m_spCfg->getSsDim(), m_spCfg->getSsEta(), 
		m_spCfg->getSsLb(), m_spCfg->getSsUb(), 
		m_spCfg->getSsErr(), X_widthPerDimension).toInt();
	
	// Auto-calculate MAX_COORD_VALUE as maximum grid size across all dimensions
	MAX_COORD_VALUE = 0;
	for (int i = 0; i < m_spCfg->getSsDim(); ++i) {
		MAX_COORD_VALUE = std::max(MAX_COORD_VALUE, (int)X_widthPerDimension[i]);
	}
	// Add small buffer for safety
	MAX_COORD_VALUE += 1;
	
	// Allocate dynamic arrays (after MAX_COORD_VALUE is known)
    // Pointer initialization (actual pointers assigned in initSafeSet)
    m_safe_set_basis = nullptr;
	m_safe_set_flat_indices = nullptr;
    m_unsafe_mask = std::vector<unsigned char>(MAX_BASIS_ELEMENTS, 0);
	m_neighbor_buffer = std::vector<int>(MAX_BASIS_ELEMENTS * MAX_STATE_DIM * MAX_STATE_DIM, 0);
	m_neighbor_parent_dim = std::vector<int>(MAX_BASIS_ELEMENTS * MAX_STATE_DIM, 0);
	m_neighbor_parent_coord = std::vector<int>(MAX_BASIS_ELEMENTS * MAX_STATE_DIM, 0);
    m_seen_neighbors = std::vector<unsigned char>(x_flat_width, 0);
	
	// Allocate 3D array for coord_buckets
	m_coord_buckets = std::vector<std::vector<std::vector<int>>>(MAX_STATE_DIM, std::vector<std::vector<int>>(MAX_COORD_VALUE, std::vector<int>(MAX_BUCKET_SIZE, 0)));
	
	// Allocate 2D array for bucket_sizes
	m_bucket_sizes = std::vector<std::vector<int>>(MAX_STATE_DIM, std::vector<int>(MAX_COORD_VALUE, 0));

	// Setup basis evolution recording if enabled
	m_record_basis_evolution = m_spCfg->isRecordBasisEvolution();
	if (m_record_basis_evolution) {
		m_basis_csv_file.open("basis_coordinates.csv");
		if (m_basis_csv_file.is_open()) {
			// Write CSV header
			m_basis_csv_file << "iteration";
            for (int i = 0; i < (int)ssDim; ++i) {
                m_basis_csv_file << ",idx" << i;
            }
            m_basis_csv_file << "\n";
			std::cout << "[MonoSynth] Recording basis evolution to basis_coordinates.csv" << std::endl;
		} else {
			std::cerr << "[MonoSynth] Warning: Could not open basis_coordinates.csv for recording" << std::endl;
			m_record_basis_evolution = false;
		}
	}

	// Loading the memory fingerprint of the abstract functions from .mem files
	std::string precomputeMem = packPath + "precompute_transitions.mem";
	std::cout << "[MonoSynth] Loading memory fingerprint from: " << precomputeMem << std::endl;
    auto precomputeArgs = pfacesKernelFunctionArguments::loadFromFile(
        precomputeMem,
        KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NAME,
        {KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNCARG_NEXT_STATE_TABLE_NAME,
         KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNCARG_RUNTIME_PARAMS_NAME},
        true);
    precomputeArgs.m_baseTypeMultiple = { (size_t)(x_flat_width * ssDim), 4 };
	addKernelFunction(pfacesKernelFunction(KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NAME, precomputeArgs));
	
	updateParameters(getParameterList().first, getParameterList().second);

	std::string safetyMem = packPath + "check_basis_safety.mem";
	std::cout << "[MonoSynth] Loading memory fingerprint from: " << safetyMem << std::endl;
    auto safetyArgs = pfacesKernelFunctionArguments::loadFromFile(
        safetyMem,
        KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_NAME,
        { KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_BASIS_FLAT_IDX_NAME, 
          KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_NEXT_STATE_TABLE_NAME, 
          KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_BASIS_LIST_NAME, 
          KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_BASIS_LIST_SIZE_NAME,
          KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_UNSAFE_FLAGS_NAME },
        true);
    safetyArgs.m_baseTypeMultiple = { (size_t)MAX_BASIS_ELEMENTS, (size_t)(x_flat_width * ssDim), (size_t)(MAX_BASIS_ELEMENTS * ssDim), 1, (size_t)MAX_BASIS_ELEMENTS };
	addKernelFunction(pfacesKernelFunction(KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_NAME, safetyArgs));

	// GPU Bitmap kernel (func idx 2)
	std::string bitmapMem = packPath + "build_bitmap.mem";
	std::cout << "[MonoSynth] Loading memory fingerprint from: " << bitmapMem << std::endl;
	auto bitmapArgs = pfacesKernelFunctionArguments::loadFromFile(
		bitmapMem,
		KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNC_NAME,
		{ KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNCARG_BITMAP_NAME,
		  KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNCARG_BASIS_LIST_NAME,
		  KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNCARG_BASIS_LIST_SIZE_NAME },
		true);
	bitmapArgs.m_baseTypeMultiple = { (size_t)x_flat_width, (size_t)(MAX_BASIS_ELEMENTS * ssDim), 1 };
	addKernelFunction(pfacesKernelFunction(KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNC_NAME, bitmapArgs));
}

/* providing implementation of the driver of the kernel */
void pfacesKernel_mono_synth::configureParallelProgram(pfacesParallelProgram& parallelProgram) {
	pfacesParallelAdvisor parallelAdvisor(parallelProgram.getMachine(), parallelProgram.getTargetDevicesIndicies());
	size_t beVerboseLevel = parallelProgram.m_beVerboseLevel;
    const bool doBenchmark = (m_benchmark_count > 1);

	// Distribute jobs
	cl::NDRange ndrPrecompute{x_flat_width, 1, 1}, ndrCheckSafety{(size_t)MAX_BASIS_ELEMENTS, 1, 1}, ndrOffset{0, 0, 0};
	cl::NDRange ndrBuildBitmap{x_flat_width, 1, 1};
	job_execPrecomputeTransition = parallelAdvisor.distributeJob(*this, KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_IDX, ndrPrecompute, ndrOffset, parallelProgram.m_isFixedJobDistribution, parallelProgram.m_fixedJobDistribution, true, false, false);
    job_execCheckBasisSafety = parallelAdvisor.distributeJob(*this, KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_IDX, ndrCheckSafety, ndrOffset, parallelProgram.m_isFixedJobDistribution, parallelProgram.m_fixedJobDistribution, true, false, false);
    job_execBuildBitmap = parallelAdvisor.distributeJob(*this, KERNEL_MONO_SYNTH_BUILD_BITMAP_FUNC_IDX, ndrBuildBitmap, ndrOffset, parallelProgram.m_isFixedJobDistribution, parallelProgram.m_fixedJobDistribution, true, false, false);

	if (beVerboseLevel >= 2)
		parallelAdvisor.printTaskSchedulingReport(parallelProgram.getMachine(), { KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NAME }, { job_execPrecomputeTransition }, x_flat_width);

	// Memory allocation
	std::vector<std::pair<char*, size_t>> dataPool;
	pFacesMemoryAllocationReport memReport = allocateMemory(dataPool, parallelProgram.getMachine(), parallelProgram.getTargetDevicesIndicies(), 1, true);
    if (beVerboseLevel >= 2) memReport.PrintReport();

	// IO jobs
    ///TODO: Use the #defines in the .h file instead of tbe hard-coded numbers below
    const cl::Device& dataAccessDevice = parallelProgram.getTargetDevices()[0];
    job_readNextStateTable = std::make_shared<pfacesDeviceReadJob>(dataAccessDevice, 0, 2, 0);
    job_writeNextStateTable = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 0, 2, 0);
    job_writeRuntimeParams = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 0, 2, 1);
    job_readUnsafeFlags = std::make_shared<pfacesDeviceReadJob>(dataAccessDevice, 1, 5, 4);
    job_writeBasisFlatIdx = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 1, 5, 0);
    job_writeBasisList = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 1, 5, 2);
    job_writeBasisListSize = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 1, 5, 3);

    // GPU Bitmap IO job: read bitmap buffer from device (func2, 3 args, arg0)
    job_readBitmap = std::make_shared<pfacesDeviceReadJob>(dataAccessDevice, 2, 3, 0);

    instr_readNextStateTable->setAsReadDeviceBuffer(job_readNextStateTable);
    instr_writeNextStateTable->setAsWriteDeviceBuffer(job_writeNextStateTable);
    instr_writeRuntimeParams->setAsWriteDeviceBuffer(job_writeRuntimeParams);
    instr_readUnsafeFlags->setAsReadDeviceBuffer(job_readUnsafeFlags);
    instr_writeBasisFlatIdx->setAsWriteDeviceBuffer(job_writeBasisFlatIdx);
    instr_writeBasisList->setAsWriteDeviceBuffer(job_writeBasisList);
    instr_writeBasisListSize->setAsWriteDeviceBuffer(job_writeBasisListSize);
    instr_readBitmap->setAsReadDeviceBuffer(job_readBitmap);

	instr_BlockingSyncPoint->setAsBlockingSyncPoint();
    instr_logOff->setAsLogOff();
    instr_logOn->setAsLogOn();

	// Check if cache file is available and compatible cache
    /// TODO: this will be problematic if we have two examples with same state space size but different dynamics loading from the same file! Maybe use project_name as part of the cache file name?
	bool useCache = false;
	if (!m_skip_cache) {
		std::ifstream cache_check(cache_file, std::ios::binary);
		if (cache_check.good()) {
			int cached_states;
			cache_check.read(reinterpret_cast<char*>(&cached_states), sizeof(int));
			if (cached_states == (int)x_flat_width) useCache = true;
		}
		cache_check.close();
	}

    // Always write runtime_params before precompute_transitions.
    // In direct mode, runtime_params[0] is set by the controller integration
    // via setRuntimeParam0() before running.
	if (dataPool.size() > 1 && dataPool[1].first) {
		float* params = (float*)dataPool[1].first;
        params[0] = m_runtime_param0;  // runtime_params[0] (0 = use compile-time default)
	}
	instructionList.push_back(instr_writeRuntimeParams);

	if (useCache) {
        instructionList.push_back(instr_BlockingSyncPoint);
		instr_hostFuncLoadNextStateTable->setAsHostFunction(pfacesKernel_mono_synth::loadTransitionTable, "loadTransitionTable");
		instructionList.push_back(instr_hostFuncLoadNextStateTable);
		instructionList.push_back(instr_writeNextStateTable);
	} else {
		for (auto& job : job_execPrecomputeTransition) {
			auto instr = std::make_shared<pfacesInstruction>();
			instr->setAsDeviceExecute(job);
			instructionList.push_back(instr);
		}
		if (parallelProgram.countTargetDevices() > 1) instructionList.push_back(instr_BlockingSyncPoint);
		instructionList.push_back(instr_readNextStateTable);
		instructionList.push_back(instr_BlockingSyncPoint);
		if (!m_skip_cache) {
			instr_hostFuncSaveTransitions->setAsHostFunction(pfacesKernel_mono_synth::saveTransitionTable, "saveTransitionTable");
			instructionList.push_back(instr_hostFuncSaveTransitions);
		}
	}

	// Benchmark loop is only useful when benchmarking more than one run.
	// In direct controller mode we set m_benchmark_count=1, so skipping this
	// avoids extra host functions, jumps, and sync points on every synthesis.
    size_t benchmark_loop_start = instructionList.size();
    if (doBenchmark) {
        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();

        instr_hostFuncBenchmarkStart->setAsHostFunction(pfacesKernel_mono_synth::benchmarkStart, "benchmarkStart");
        instructionList.push_back(instr_hostFuncBenchmarkStart);

        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();

        benchmark_loop_start = instructionList.size();
    }

    // Safe set iteration
    instructionList.push_back(std::make_shared<pfacesInstruction>());
    instructionList.back()->setAsBlockingSyncPoint();
    
    instr_hostFuncInitSafeSet->setAsHostFunction(pfacesKernel_mono_synth::initSafeSet, "initSafeSet");
    instructionList.push_back(instr_hostFuncInitSafeSet);
    
    instructionList.push_back(std::make_shared<pfacesInstruction>());
    instructionList.back()->setAsBlockingSyncPoint();

    size_t loop_start = instructionList.size();
    instr_hostFuncPrepareSafeSetIteration->setAsHostFunction(pfacesKernel_mono_synth::prepareSafeSetIteration, "prepareSafeSetIteration");
    instructionList.push_back(instr_hostFuncPrepareSafeSetIteration);
    instructionList.push_back(instr_writeBasisFlatIdx);
    instructionList.push_back(instr_writeBasisList);
    instructionList.push_back(instr_writeBasisListSize);

    for (auto& job : job_execCheckBasisSafety) {
        auto instr = std::make_shared<pfacesInstruction>();
        instr->setAsDeviceExecute(job);
        instructionList.push_back(instr);
    }
    instructionList.push_back(instr_readUnsafeFlags);
    
    instructionList.push_back(std::make_shared<pfacesInstruction>());
    instructionList.back()->setAsBlockingSyncPoint();
    
    instr_hostFuncProcessSafeSetUpdate->setAsHostFunction(pfacesKernel_mono_synth::processSafeSetUpdate, "processSafeSetUpdate");
    instructionList.push_back(instr_hostFuncProcessSafeSetUpdate);

    instr_jumpToSafeSetStart->setAsJumpNe(loop_start);
    instructionList.push_back(instr_jumpToSafeSetStart);

    if (doBenchmark) {
        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();

        instr_hostFuncBenchmarkNext->setAsHostFunction(pfacesKernel_mono_synth::benchmarkNext, "benchmarkNext");
        instructionList.push_back(instr_hostFuncBenchmarkNext);

        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();

        instr_jumpToBenchmarkStart->setAsJumpNe(benchmark_loop_start);
        instructionList.push_back(instr_jumpToBenchmarkStart);
    }
    
    // Build bitmap on GPU after synthesis converges
    //   1. prepareBitmapGPU: write final basis_list_size to pool buffer
    //   2. writeBasisList + writeBasisListSize: transfer final basis to device
    //   3. exec build_bitmap: GPU kernel computes downward closure
    //   4. readBitmap: transfer bitmap from device to host
    //   5. copyBitmapFromGPU: convert int* to uint8_t vector
    instructionList.push_back(std::make_shared<pfacesInstruction>());
    instructionList.back()->setAsBlockingSyncPoint();

    instr_hostFuncPrepareBitmapGPU->setAsHostFunction(pfacesKernel_mono_synth::prepareBitmapGPU, "prepareBitmapGPU");
    instructionList.push_back(instr_hostFuncPrepareBitmapGPU);
    instructionList.push_back(instr_writeBasisList);
    instructionList.push_back(instr_writeBasisListSize);

    instructionList.push_back(std::make_shared<pfacesInstruction>());
    instructionList.back()->setAsBlockingSyncPoint();

    for (auto& job : job_execBuildBitmap) {
        auto instr = std::make_shared<pfacesInstruction>();
        instr->setAsDeviceExecute(job);
        instructionList.push_back(instr);
    }

    instructionList.push_back(instr_readBitmap);

    instructionList.push_back(std::make_shared<pfacesInstruction>());
    instructionList.back()->setAsBlockingSyncPoint();

    instr_hostFuncCopyBitmapFromGPU->setAsHostFunction(pfacesKernel_mono_synth::copyBitmapFromGPU, "copyBitmapFromGPU");
    instructionList.push_back(instr_hostFuncCopyBitmapFromGPU);

    instructionList.push_back(std::make_shared<pfacesInstruction>());
    instructionList.back()->setAsBlockingSyncPoint();
	
	parallelProgram.m_Universal_globalNDRange = parallelProgram.m_Process_globalNDRange = ndrPrecompute;
	parallelProgram.m_Universal_offsetNDRange = parallelProgram.m_Process_offsetNDRange = ndrOffset;
    
	parallelProgram.m_Universal_globalNDRange = parallelProgram.m_Process_globalNDRange = ndrPrecompute;
	parallelProgram.m_Universal_offsetNDRange = parallelProgram.m_Process_offsetNDRange = ndrOffset;
    
    // Point parallelProgram.m_dataPool to our allocated dataPool
    parallelProgram.m_dataPool = dataPool;
	parallelProgram.m_spInstructionList = instructionList;
}

/* Safe Set Host Functions */
///TODO: some functions below are host-side functions. What about moving them above in the file with other host-side functions?
size_t pfacesKernel_mono_synth::initSafeSet(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;

    pKernel->m_ss_dim = pKernel->m_spCfg->getSsDim();
    
    // Assign pointers directly from the data pool
    // Note: Pool[0]=next_state_table, Pool[1]=runtime_params,
    //   Pool[2]=basis_flat_idx, Pool[3]=basis_list, Pool[4]=basis_list_size, Pool[5]=unsafe_flags
    pKernel->m_safe_set_flat_indices = (int*)pParallelProgram->m_dataPool[2].first;
    pKernel->m_safe_set_basis = (int*)pParallelProgram->m_dataPool[3].first;
    
    if (!pKernel->m_safe_set_flat_indices || !pKernel->m_safe_set_basis) {
        std::cerr << "[MonoSynth] Error: Null pointer in dataPool!" << std::endl;
        return 1;
    }

    // Zero the arrays (important as we reuse these pooled buffers)
    std::memset(pKernel->m_safe_set_basis, 0, (size_t)pKernel->MAX_BASIS_ELEMENTS * (size_t)pKernel->MAX_STATE_DIM * sizeof(int));
    std::memset(pKernel->m_safe_set_flat_indices, 0, (size_t)pKernel->MAX_BASIS_ELEMENTS * sizeof(int));
    
    // Clear coordinate buckets (prevents stale indices from previous runs)
    for (int i = 0; i < pKernel->MAX_STATE_DIM; ++i) {
        std::memset(pKernel->m_bucket_sizes[i].data(), 0, pKernel->MAX_COORD_VALUE * sizeof(int));
    }
    
    // Initialize to corner of the box
    for (int i = 0; i < pKernel->m_ss_dim; ++i) {
        pKernel->m_safe_set_basis[i] = (int)pKernel->X_widthPerDimension[i];
    }
    pKernel->m_safe_set_size = 1;
    pKernel->m_safe_set_flat_indices[0] = pKernel->flattenIndex(pKernel->m_safe_set_basis);
    
    pKernel->m_iterations = 0;
    pKernel->m_compute_start = std::chrono::high_resolution_clock::now();

    return 0;
}

size_t pfacesKernel_mono_synth::prepareSafeSetIteration(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;
    const int ss_dim = pKernel->m_ss_dim;

    pKernel->m_iterations++;

    int* pBasisListSize = (int*)pParallelProgram->m_dataPool[4].first;
    if (pBasisListSize) {
        *pBasisListSize = pKernel->m_safe_set_size;
    }

    // Record basis coordinates to CSV if enabled
    if (pKernel->m_record_basis_evolution && pKernel->m_basis_csv_file.is_open()) {
        for (int i = 0; i < pKernel->m_safe_set_size; ++i) {
            pKernel->m_basis_csv_file << pKernel->m_iterations;
            for (int j = 0; j < ss_dim; ++j) {
                pKernel->m_basis_csv_file << "," << pKernel->m_safe_set_basis[i * ss_dim + j];
            }
            pKernel->m_basis_csv_file << "\n";
        }
        pKernel->m_basis_csv_file.flush();  // Ensure data is written immediately
    }

    // Update ND-Range
    cl::NDRange ndRange(pKernel->m_safe_set_size, 1, 1);
    for (auto& job : pKernel->job_execCheckBasisSafety) {
        job->getTasks()[0]->setNdRangeGlobal(ndRange);
    }

    return 0;
}

size_t pfacesKernel_mono_synth::processSafeSetUpdate(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;
    
    int* pUnsafeFlags = (int*)pParallelProgram->m_dataPool[5].first;
    int added = pKernel->updateSafeSet(pUnsafeFlags);

    if (added == 0) {
        auto compute_end = std::chrono::high_resolution_clock::now();
        double time_ms = std::chrono::duration<double, std::milli>(compute_end - pKernel->m_compute_start).count();
        
        pKernel->m_benchmark_total_time_ms += time_ms;
        pKernel->m_benchmark_total_iterations += pKernel->m_iterations;
        pKernel->m_benchmark_current_run++;

        std::cout << "Run " << pKernel->m_benchmark_current_run << "/" << pKernel->m_benchmark_count << ": " 
                  << pKernel->m_iterations << " iterations, " 
                  << pKernel->m_safe_set_size << " basis, " 
                  << (int)time_ms << " ms" << std::endl;
        
        return 0; // Stop inner loop
    }

    return added; // Continue inner loop
}

size_t pfacesKernel_mono_synth::benchmarkStart(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pKernel->m_benchmark_current_run = 0;
    pKernel->m_benchmark_total_time_ms = 0;
    pKernel->m_benchmark_total_iterations = 0;
    std::cout << "\nStarting Performance Benchmark (" << pKernel->m_benchmark_count << " runs)...\n" << std::endl;
    return 0;
}

size_t pfacesKernel_mono_synth::benchmarkNext(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;

    if (pKernel->m_benchmark_current_run < pKernel->m_benchmark_count) {
        return 1; // Continue loop
    } else {
        std::cout << "\nBenchmark Finished!" << std::endl;
        std::cout << "Average: " << pKernel->m_benchmark_total_iterations / pKernel->m_benchmark_count << " iterations, " 
                  << (int)(pKernel->m_benchmark_total_time_ms / pKernel->m_benchmark_count) << " ms" << std::endl;
        return 0; // Stop loop
    }
}

int pfacesKernel_mono_synth::flattenIndex(const int* idx) const {
    int result = 0, multiplier = 1;
    for (int i = 0; i < m_ss_dim; ++i) {
        result += (idx[i] - 1) * multiplier;
        multiplier *= (int)X_widthPerDimension[i];
    }
    return result;
}

int pfacesKernel_mono_synth::updateSafeSet(int* unsafe_flags) {    
    const int ss_dim = m_ss_dim;
    const int total_states = x_flat_width;
    
    // Clear persistent buffers
    std::memset(m_unsafe_mask.data(), 0, m_safe_set_size * sizeof(unsigned char));
    std::memset(m_seen_neighbors.data(), 0, total_states * sizeof(unsigned char));

    // Pass 1: Find unsafe elements and generate unique neighbors
    // Store which dimension was decremented for each neighbor (for indexed redundancy check)
    int neighbor_count = 0;
    for (int i = 0; i < m_safe_set_size; ++i) {
        if (!unsafe_flags[i]) continue;
        m_unsafe_mask[i] = 1;

        for (int j = 0; j < ss_dim; ++j) {
            const int val = m_safe_set_basis[i * ss_dim + j];
            if (val <= 1) continue;

            int coords[MAX_STATE_DIM];
            for (int k = 0; k < ss_dim; ++k) {
                coords[k] = m_safe_set_basis[i * ss_dim + k] - (j == k ? 1 : 0);
            }

            const int flat_idx = flattenIndex(coords);
            if (!m_seen_neighbors[flat_idx]) {
                m_seen_neighbors[flat_idx] = 1;
                int* neighbor = &m_neighbor_buffer[neighbor_count * ss_dim];
                for (int k = 0; k < ss_dim; ++k) {
                    neighbor[k] = coords[k];
                }
                // Store parent info for optimized redundancy check
                m_neighbor_parent_dim[neighbor_count] = j;
                m_neighbor_parent_coord[neighbor_count] = val; // Original coord before decrement
                neighbor_count++;
            }
        }
    }

    // Pass 2: Compact safe elements (remove unsafe)
    int write_pos = 0;
    for (int i = 0; i < m_safe_set_size; ++i) {
        if (!m_unsafe_mask[i]) {
            if (write_pos != i) {
                for (int j = 0; j < ss_dim; ++j) {
                    m_safe_set_basis[write_pos * ss_dim + j] = m_safe_set_basis[i * ss_dim + j];
                }
                m_safe_set_flat_indices[write_pos] = m_safe_set_flat_indices[i];
            }
            write_pos++;
        }
    }
    m_safe_set_size = write_pos;

    // Pass 2.5: Rebuild coordinate index for surviving elements
    rebuildCoordIndex();

    // Pass 3: Add new neighbors using INDEXED redundancy check
    // By theorem: neighbor n = b - e_j can only be dominated by b' where b'[j] = n[j] = b[j] - 1
    int added = 0;
    for (int ni = 0; ni < neighbor_count; ++ni) {
        int* neighbor = &m_neighbor_buffer[ni * ss_dim];
        const int parent_dim = m_neighbor_parent_dim[ni];
        const int target_coord = neighbor[parent_dim]; // = parent_coord - 1
        
        // Only check basis elements with coordinate = target_coord in dimension parent_dim
        bool dominated = false;
        const int bucket_size = m_bucket_sizes[parent_dim][target_coord];
        for (int bi = 0; bi < bucket_size; ++bi) {
            const int basis_idx = m_coord_buckets[parent_dim][target_coord][bi];
            
            // Check if this basis element dominates the neighbor
            bool is_dominated = true;
            for (int j = 0; j < ss_dim; ++j) {
                if (neighbor[j] > m_safe_set_basis[basis_idx * ss_dim + j]) {
                    is_dominated = false;
                    break;
                }
            }
            if (is_dominated) {
                dominated = true;
                break;
            }
        }
        
        if (!dominated) {
            if (m_safe_set_size >= MAX_BASIS_ELEMENTS) break;
            
            // Add to basis
            for (int j = 0; j < ss_dim; ++j) {
                m_safe_set_basis[m_safe_set_size * ss_dim + j] = neighbor[j];
            }
            m_safe_set_flat_indices[m_safe_set_size] = flattenIndex(neighbor);
            
            // Add to coordinate index
            for (int j = 0; j < ss_dim; ++j) {
                const int coord = neighbor[j];
                if (coord < MAX_COORD_VALUE) {
                    int& bs = m_bucket_sizes[j][coord];
                    if (bs < MAX_BUCKET_SIZE) {
                        m_coord_buckets[j][coord][bs++] = m_safe_set_size;
                    }
                }
            }
            
            m_safe_set_size++;
            added++;
        }
    }
    return added;
}

void pfacesKernel_mono_synth::rebuildCoordIndex() {
    const int ss_dim = m_ss_dim;
    for (int i = 0; i < MAX_STATE_DIM; ++i) {
        std::memset(m_bucket_sizes[i].data(), 0, MAX_COORD_VALUE * sizeof(int));
    }
    
    // Populate index from current basis
    for (int i = 0; i < m_safe_set_size; ++i) {
        for (int j = 0; j < ss_dim; ++j) {
            const int coord = m_safe_set_basis[i * ss_dim + j];
            if (coord < MAX_COORD_VALUE) {
                int& bs = m_bucket_sizes[j][coord];
                if (bs < MAX_BUCKET_SIZE) {
                    m_coord_buckets[j][coord][bs++] = i;
                }
            }
        }
    }
}

/* Prepare data for GPU bitmap kernel — called as host function before GPU exec */
size_t pfacesKernel_mono_synth::prepareBitmapGPU(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;
    
    // Write final basis size to pool buffer so it gets transferred to device
    int* pBasisListSize = (int*)pParallelProgram->m_dataPool[4].first;
    if (pBasisListSize) {
        *pBasisListSize = pKernel->m_safe_set_size;
    }
    
    return 0;
}

/* Copy GPU bitmap buffer to host m_bitmap vector — called after GPU exec + read-back */
/* Remaps from kernel convention (column-major, 1-based, priority-reversed) to
   safe_set.h convention (row-major, 0-based, no priority reversal) */
size_t pfacesKernel_mono_synth::copyBitmapFromGPU(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;
    
    const int total = (int)pKernel->x_flat_width;
    const int n_dim = pKernel->m_ss_dim;
    const auto& widths = pKernel->X_widthPerDimension;
    auto priorities = pKernel->m_spCfg->getSsPriorities();
    
    pKernel->m_bitmap.assign(total, 0);
    
    // GPU bitmap is int* (pool[BITMAP_DATA_POOL_IDX]) in kernel convention
    const int* gpuBitmap = (const int*)pParallelProgram->m_dataPool[BITMAP_DATA_POOL_IDX].first;
    
    // Precompute row-major strides for safe_set.h convention
    std::vector<int> row_strides(n_dim);
    row_strides[n_dim - 1] = 1;
    for (int d = n_dim - 2; d >= 0; --d) {
        row_strides[d] = row_strides[d + 1] * (int)widths[d + 1];
    }
    
    int safe_count = 0;
    std::vector<int> kernel_idx(n_dim, 0);
    std::vector<int> ss_idx(n_dim, 0);
    for (int kf = 0; kf < total; ++kf) {
        if (!gpuBitmap[kf]) continue;
        
        // 1. Unflatten in column-major, 1-based (dim 0 fastest)
        int temp = kf;
        for (int d = 0; d < n_dim; ++d) {
            kernel_idx[d] = (temp % (int)widths[d]) + 1;  // 1-based
            temp /= (int)widths[d];
        }
        
        // 2. Convert to safe_set 0-based index (reverse priority 0 dims)
        for (int d = 0; d < n_dim; ++d) {
            if (d < (int)priorities.size() && priorities[d] == 0) {
                // priority 0: kernel idx=1 → x_max → safe_set idx=sizes[d]-1
                ss_idx[d] = (int)widths[d] - kernel_idx[d];
            } else {
                // priority 1: kernel idx=1 → x_min → safe_set idx=0
                ss_idx[d] = kernel_idx[d] - 1;
            }
        }
        
        // 3. Flatten in row-major (last dim fastest)
        int sf = 0;
        for (int d = 0; d < n_dim; ++d) {
            sf += ss_idx[d] * row_strides[d];
        }
        
        if (sf >= 0 && sf < total) {
            pKernel->m_bitmap[sf] = 1;
            safe_count++;
        }
    }
    
    std::cout << "[MonoSynth] GPU Bitmap: " << safe_count << "/" << total 
              << " safe cells (" << (100.0 * safe_count / total) << "%)" << std::endl;
    
    return 0;
}

/* not providing implementation of the virtual method: configureTuneParallelProgram*/
void pfacesKernel_mono_synth::configureTuneParallelProgram(pfacesParallelProgram&, size_t) {
}

} // namespace mono_synth

PFACES_REGISTER_LOADABLE_KERNEL(mono_synth::pfacesKernel_mono_synth)



