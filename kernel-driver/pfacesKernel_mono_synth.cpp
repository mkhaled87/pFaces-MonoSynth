/*
* pfacesKernel_mono_synth.cpp
*
*  created on: 02.10.2025
*      author: M. Khaled
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <cstring>
#include <chrono>
#include <algorithm>

#include "pfacesKernel_mono_synth.h"
#include <filesystem>
namespace mono_synth {


/* a call-back function to save the controller/abstraction after the kernel finishes */
size_t pfacesKernel_mono_synth::saveTransitionTable(void* pPackedKernel, void* pPackedParallelProgram) {
	
	// retrieving the required values
	const static pfacesParallelProgram*  pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;
	const char* pDataTransitionTable = pParallelProgram->m_dataPool[0].first;
	int ss_dim = ((pfacesKernel_mono_synth*)(pPackedKernel))->m_spCfg->getSsDim();
	// int number_of_states = number_of_elements/ss_dim;
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
	// auto transition_table_size = pParallelProgram->m_dataPool[0].second;

	// save to file
	const char* file_path = ((pfacesKernel_mono_synth*)(pPackedKernel))->cache_file;
	// int number_of_elements = transition_table_size/sizeof(cl_int);
	int ss_dim = ((pfacesKernel_mono_synth*)(pPackedKernel))->m_spCfg->getSsDim();
	// int number_of_states = number_of_elements/ss_dim;
	int number_of_states = ((pfacesKernel_mono_synth*)(pPackedKernel))->x_flat_width;
	int number_of_elements = number_of_states*ss_dim;

	// load
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

std::pair<std::vector<std::string>, std::vector<std::string>> pfacesKernel_mono_synth::getParameterList() {
	
	std::vector<std::string> params;
	std::vector<std::string> paramvals;

	/* X and U dimensions */
	params.push_back(param_ss_dim);
	paramvals.push_back(std::to_string(m_spCfg->getSsDim()));
	params.push_back(param_is_dim);
	paramvals.push_back(std::to_string(m_spCfg->getIsDim()));

	/* X params : Eta */
	std::stringstream ss_eta_ss;
	pfacesUtils::PrintVector(m_spCfg->getSsEta(), ',', false, ss_eta_ss);
	std::string ss_eta_str = ss_eta_ss.str();
	params.push_back(param_ss_eta);
	paramvals.push_back(ss_eta_str);

	/* X params : LB */
	std::stringstream ss_lb_ss;
	pfacesUtils::PrintVector(m_spCfg->getSsLb(), ',', false, ss_lb_ss);
	std::string ss_lb_str = ss_lb_ss.str();
	params.push_back(param_ss_lb);
	paramvals.push_back(ss_lb_str);

	/* X params : UB */
	std::stringstream ss_ub_ss;
	pfacesUtils::PrintVector(m_spCfg->getSsUb(), ',', false, ss_ub_ss);
	std::string ss_ub_str = ss_ub_ss.str();
	params.push_back(param_ss_ub);
	paramvals.push_back(ss_ub_str);

	return std::make_pair(params, paramvals);
}


/* constructor: initiate data and prepare memory maps*/
pfacesKernel_mono_synth::pfacesKernel_mono_synth(const std::shared_ptr<pfacesKernelLaunchState>& spLaunchState, const std::shared_ptr<pfacesConfigurationReader>& spCfg)
	: pfaces2DKernel(spLaunchState->getDefaultSourceFilePath(KERNEL_NAME_MONO_SYNTH)),
	  m_spCfg(std::make_shared<configReader>(spCfg)) {

	m_kernelScope = spLaunchState->getKernelScope();

	// setting the dimensions of the base 2d kernel object
	size_t ssDim = m_spCfg->getSsDim();
	size_t isDim = m_spCfg->getIsDim();

	// Convert concrete space into flat space for X
	x_flat_width = pfacesFlatSpace::getFlatWidthFromConcreteSpace(
		m_spCfg->getSsDim(), m_spCfg->getSsEta(), 
		m_spCfg->getSsLb(), m_spCfg->getSsUb(), 
		m_spCfg->getSsErr(), X_widthPerDimension).toInt();

	// Loading the memory fingerprint of the abstract functions from .mem files
    auto precomputeArgs = pfacesKernelFunctionArguments::loadFromFile(
        spLaunchState->getKernelPackPath() + "precompute_transitions.mem",
        KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NAME,
        {KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNCARG_NEXT_STATE_TABLE_NAME},
        false);
    precomputeArgs.m_baseTypeMultiple = { (size_t)(x_flat_width * ssDim) };
	addKernelFunction(pfacesKernelFunction(KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NAME, precomputeArgs));
	
	updateParameters(getParameterList().first, getParameterList().second);

    auto safetyArgs = pfacesKernelFunctionArguments::loadFromFile(
        spLaunchState->getKernelPackPath() + "check_basis_safety.mem",
        KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_NAME,
        { KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_BASIS_FLAT_IDX_NAME, 
          KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_NEXT_STATE_TABLE_NAME, 
          KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_BASIS_LIST_NAME, 
          KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_BASIS_LIST_SIZE_NAME,
          KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNCARG_UNSAFE_FLAGS_NAME },
        false);
    safetyArgs.m_baseTypeMultiple = { 2000, (size_t)(x_flat_width * ssDim), (size_t)(2000 * ssDim), 1, 2000 };
	addKernelFunction(pfacesKernelFunction(KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_NAME, safetyArgs));
}

/* providing implementation of the driver of the kernel */
void pfacesKernel_mono_synth::configureParallelProgram(pfacesParallelProgram& parallelProgram) {
	pfacesParallelAdvisor parallelAdvisor(parallelProgram.getMachine(), parallelProgram.getTargetDevicesIndicies());
	size_t beVerboseLevel = parallelProgram.m_beVerboseLevel;

	// Distribute jobs
    // TODO: Replace OpenCL with pFaces API
	cl::NDRange ndrPrecompute{x_flat_width, 1, 1}, ndrCheckSafety{2000, 1, 1}, ndrOffset{0, 0, 0};
	job_execPrecomputeTransition = parallelAdvisor.distributeJob(*this, KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_IDX, ndrPrecompute, ndrOffset, parallelProgram.m_isFixedJobDistribution, parallelProgram.m_fixedJobDistribution, true, false, false);
    job_execCheckBasisSafety = parallelAdvisor.distributeJob(*this, KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_IDX, ndrCheckSafety, ndrOffset, parallelProgram.m_isFixedJobDistribution, parallelProgram.m_fixedJobDistribution, true, false, false);

	if (beVerboseLevel >= 2)
		parallelAdvisor.printTaskSchedulingReport(parallelProgram.getMachine(), { KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NAME }, { job_execPrecomputeTransition }, x_flat_width);

	// Memory allocation
	std::vector<std::pair<char*, size_t>> dataPool;
	pFacesMemoryAllocationReport memReport = allocateMemory(dataPool, parallelProgram.getMachine(), parallelProgram.getTargetDevicesIndicies(), 1, false);
    if (beVerboseLevel >= 2) memReport.PrintReport();
    // TODO: Replace OpenCL with pFaces API
	const cl::Device& dataAccessDevice = parallelProgram.getTargetDevices()[0];

	// IO jobs
    job_readNextStateTable = std::make_shared<pfacesDeviceReadJob>(dataAccessDevice, 0, 1, 0);
    job_writeNextStateTable = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 0, 1, 0);
    job_readUnsafeFlags = std::make_shared<pfacesDeviceReadJob>(dataAccessDevice, 1, 5, 4);
    job_writeBasisFlatIdx = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 1, 5, 0);
    job_writeBasisList = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 1, 5, 2);
    job_writeBasisListSize = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 1, 5, 3);

    instr_readNextStateTable->setAsReadDeviceBuffer(job_readNextStateTable);
    instr_writeNextStateTable->setAsWriteDeviceBuffer(job_writeNextStateTable);
    instr_readUnsafeFlags->setAsReadDeviceBuffer(job_readUnsafeFlags);
    instr_writeBasisFlatIdx->setAsWriteDeviceBuffer(job_writeBasisFlatIdx);
    instr_writeBasisList->setAsWriteDeviceBuffer(job_writeBasisList);
    instr_writeBasisListSize->setAsWriteDeviceBuffer(job_writeBasisListSize);

	instr_BlockingSyncPoint->setAsBlockingSyncPoint();
    instr_logOff->setAsLogOff();
    instr_logOn->setAsLogOn();

	// Cache check
	bool useCache = false;
	std::ifstream cache_check(cache_file, std::ios::binary);
	if (cache_check.good()) {
		int cached_states;
		cache_check.read(reinterpret_cast<char*>(&cached_states), sizeof(int));
		if (cached_states == (int)x_flat_width) useCache = true;
	}
	cache_check.close();

	if (useCache) {
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
		instr_hostFuncSaveTransitions->setAsHostFunction(pfacesKernel_mono_synth::saveTransitionTable, "saveTransitionTable");
		instructionList.push_back(instr_hostFuncSaveTransitions);
	}

    // Benchmark initialization
    instructionList.push_back(std::make_shared<pfacesInstruction>());
    instructionList.back()->setAsBlockingSyncPoint();
    
    instr_hostFuncBenchmarkStart->setAsHostFunction(pfacesKernel_mono_synth::benchmarkStart, "benchmarkStart");
    instructionList.push_back(instr_hostFuncBenchmarkStart);
    
    instructionList.push_back(std::make_shared<pfacesInstruction>());
    instructionList.back()->setAsBlockingSyncPoint();

    size_t benchmark_loop_start = instructionList.size();

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

    // Benchmark next run
    instructionList.push_back(std::make_shared<pfacesInstruction>());
    instructionList.back()->setAsBlockingSyncPoint();

    instr_hostFuncBenchmarkNext->setAsHostFunction(pfacesKernel_mono_synth::benchmarkNext, "benchmarkNext");
    instructionList.push_back(instr_hostFuncBenchmarkNext);
    
    instructionList.push_back(std::make_shared<pfacesInstruction>());
    instructionList.back()->setAsBlockingSyncPoint();

    instr_jumpToBenchmarkStart->setAsJumpNe(benchmark_loop_start);
    instructionList.push_back(instr_jumpToBenchmarkStart);
    
    instructionList.push_back(std::make_shared<pfacesInstruction>());
    instructionList.back()->setAsBlockingSyncPoint();
	
	parallelProgram.m_Universal_globalNDRange = parallelProgram.m_Process_globalNDRange = ndrPrecompute;
	parallelProgram.m_Universal_offsetNDRange = parallelProgram.m_Process_offsetNDRange = ndrOffset;
    parallelProgram.m_compilerDefinesList.push_back({"SS_DIM", std::to_string(m_spCfg->getSsDim())});
    parallelProgram.m_compilerDefinesList.push_back({"TOTAL_STATES", std::to_string(x_flat_width)});
	
    // Point parallelProgram.m_dataPool to our allocated dataPool
    parallelProgram.m_dataPool = dataPool;
	parallelProgram.m_spInstructionList = instructionList;
}

/* Safe Set Host Functions */

size_t pfacesKernel_mono_synth::initSafeSet(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;
    
    pKernel->m_ss_dim = pKernel->m_spCfg->getSsDim();
    
    // Allocate seen_neighbors once (persists across benchmark runs)
    if (!pKernel->m_seen_neighbors) {
        pKernel->m_seen_neighbors = new unsigned char[pKernel->x_flat_width];
    }
    
    // Zero the fixed arrays (fast memset)
    std::memset(pKernel->m_safe_set_basis, 0, sizeof(pKernel->m_safe_set_basis));
    std::memset(pKernel->m_safe_set_flat_indices, 0, sizeof(pKernel->m_safe_set_flat_indices));
    
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

    // Direct copy to data pool (no intermediate vectors)
    int* pBasisFlatIdx = (int*)pParallelProgram->m_dataPool[1].first;
    int* pBasisList = (int*)pParallelProgram->m_dataPool[2].first;
    int* pBasisListSize = (int*)pParallelProgram->m_dataPool[3].first;
    
    std::memcpy(pBasisFlatIdx, pKernel->m_safe_set_flat_indices, pKernel->m_safe_set_size * sizeof(int));
    std::memcpy(pBasisList, pKernel->m_safe_set_basis, pKernel->m_safe_set_size * ss_dim * sizeof(int));
    *pBasisListSize = pKernel->m_safe_set_size;

    // Update ND-Range

    //TODO: Replace OPENCL with PFACES API
    cl::NDRange ndRange(pKernel->m_safe_set_size, 1, 1);
    for (auto& job : pKernel->job_execCheckBasisSafety) {
        job->getTasks()[0]->setNdRangeGlobal(ndRange);
    }

    return 0;
}

size_t pfacesKernel_mono_synth::processSafeSetUpdate(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;
    
    int* pUnsafeFlags = (int*)pParallelProgram->m_dataPool[4].first;
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
    
    // Clear persistent buffers (fast memset, no allocation)
    std::memset(m_unsafe_mask, 0, m_safe_set_size);
    std::memset(m_seen_neighbors, 0, total_states);

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
    
    // Clear all bucket sizes
    std::memset(m_bucket_sizes, 0, sizeof(m_bucket_sizes));
    
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

/* not providing implementation of the virtual method: configureTuneParallelProgram*/
void pfacesKernel_mono_synth::configureTuneParallelProgram(pfacesParallelProgram&, size_t) {
}

} // namespace mono_synth

PFACES_REGISTER_LOADABLE_KERNEL(mono_synth::pfacesKernel_mono_synth)



