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
	cl::NDRange ndrPrecompute{x_flat_width, 1, 1}, ndrCheckSafety{2000, 1, 1}, ndrOffset{0, 0, 0};
	job_execPrecomputeTransition = parallelAdvisor.distributeJob(*this, KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_IDX, ndrPrecompute, ndrOffset, parallelProgram.m_isFixedJobDistribution, parallelProgram.m_fixedJobDistribution, true, false, false);
    job_execCheckBasisSafety = parallelAdvisor.distributeJob(*this, KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_IDX, ndrCheckSafety, ndrOffset, parallelProgram.m_isFixedJobDistribution, parallelProgram.m_fixedJobDistribution, true, false, false);

	if (beVerboseLevel >= 2)
		parallelAdvisor.printTaskSchedulingReport(parallelProgram.getMachine(), { KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NAME }, { job_execPrecomputeTransition }, x_flat_width);

	// Memory allocation
	std::vector<std::pair<char*, size_t>> dataPool;
	pFacesMemoryAllocationReport memReport = allocateMemory(dataPool, parallelProgram.getMachine(), parallelProgram.getTargetDevicesIndicies(), 1, false);
    if (beVerboseLevel >= 2) memReport.PrintReport();
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

    // Safe set iteration
    instructionList.push_back(instr_BlockingSyncPoint);
    instr_hostFuncInitSafeSet->setAsHostFunction(pfacesKernel_mono_synth::initSafeSet, "initSafeSet");
    instructionList.push_back(instr_hostFuncInitSafeSet);
    
    instructionList.push_back(instr_logOff);
    instructionList.push_back(instr_BlockingSyncPoint);

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
    instructionList.push_back(instr_BlockingSyncPoint);
    instr_hostFuncProcessSafeSetUpdate->setAsHostFunction(pfacesKernel_mono_synth::processSafeSetUpdate, "processSafeSetUpdate");
    instructionList.push_back(instr_hostFuncProcessSafeSetUpdate);

    instr_jumpToSafeSetStart->setAsJumpNe(loop_start);
    instructionList.push_back(instr_jumpToSafeSetStart);
    instructionList.push_back(instr_logOn);
	instructionList.push_back(instr_BlockingSyncPoint);
	
	parallelProgram.m_Universal_globalNDRange = parallelProgram.m_Process_globalNDRange = ndrPrecompute;
	parallelProgram.m_Universal_offsetNDRange = parallelProgram.m_Process_offsetNDRange = ndrOffset;
    parallelProgram.m_compilerDefinesList.push_back({"SS_DIM", std::to_string(m_spCfg->getSsDim())});
    parallelProgram.m_compilerDefinesList.push_back({"TOTAL_STATES", std::to_string(x_flat_width)});
	parallelProgram.m_dataPool = dataPool;
	parallelProgram.m_spInstructionList = instructionList;
}

/* Safe Set Host Functions */

size_t pfacesKernel_mono_synth::initSafeSet(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    int ss_dim = pKernel->m_spCfg->getSsDim();
    const int max_basis_elements = 2000;

    pKernel->m_safe_set_basis.assign(max_basis_elements * ss_dim, 0);
    pKernel->m_safe_set_flat_indices.assign(max_basis_elements, 0);
    
    // Corner of the box
    std::vector<cl_ulong> X_width = pKernel->X_widthPerDimension;
    for (int i = 0; i < ss_dim; ++i) {
        pKernel->m_safe_set_basis[i] = (int)X_width[i];
    }
    pKernel->m_safe_set_size = 1;
    pKernel->m_safe_set_flat_indices[0] = pKernel->flattenIndex(pKernel->m_safe_set_basis.data());
    
    pKernel->m_iterations = 0;
    pKernel->m_compute_start = std::chrono::high_resolution_clock::now();

    return 0;
}

size_t pfacesKernel_mono_synth::prepareSafeSetIteration(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;
    int ss_dim = pKernel->m_spCfg->getSsDim();

    pKernel->m_iterations++;

    // Copy to pool
    int* pBasisFlatIdx = (int*)pParallelProgram->m_dataPool[1].first;
    int* pBasisList = (int*)pParallelProgram->m_dataPool[2].first;
    int* pBasisListSize = (int*)pParallelProgram->m_dataPool[3].first;
    
    std::memcpy(pBasisFlatIdx, pKernel->m_safe_set_flat_indices.data(), pKernel->m_safe_set_size * sizeof(int));
    std::memcpy(pBasisList, pKernel->m_safe_set_basis.data(), pKernel->m_safe_set_size * ss_dim * sizeof(int));
    *pBasisListSize = pKernel->m_safe_set_size;

    // Update ND-Range
    cl::NDRange ndRange((size_t)((pKernel->m_safe_set_size + 127) / 128) * 128, 1, 1);
    for (auto& job : pKernel->job_execCheckBasisSafety) {
        job->getTasks()[0]->setNdRangeGlobal(ndRange);
    }

    return 0;
}

size_t pfacesKernel_mono_synth::processSafeSetUpdate(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;
    
    int ss_dim = pKernel->m_spCfg->getSsDim();
    int total_states = pKernel->x_flat_width;
    const int max_basis_elements = 2000;
    int* pUnsafeFlags = (int*)pParallelProgram->m_dataPool[4].first;

    int added = pKernel->updateSafeSet(pUnsafeFlags, pKernel->m_safe_set_basis, pKernel->m_safe_set_flat_indices, 
                                      pKernel->m_safe_set_size, ss_dim, total_states, max_basis_elements);

    std::cout << "Iteration " << pKernel->m_iterations << ": Basis size = " << pKernel->m_safe_set_size << "\r" << std::flush;

    if (added == 0 || pKernel->m_safe_set_size >= max_basis_elements) {
        auto compute_end = std::chrono::high_resolution_clock::now();
        double time_ms = std::chrono::duration<double, std::milli>(compute_end - pKernel->m_compute_start).count();
        std::cout << "\nSafe Set Computation Finished [Iterations: " << pKernel->m_iterations 
                  << ", Basis: " << pKernel->m_safe_set_size 
                  << ", Time: " << (int)time_ms << " ms]" << std::endl;
        return 0; // Stop loop
    }

    return added; // Continue loop
}

int pfacesKernel_mono_synth::flattenIndex(const int* idx) const {
    int result = 0, multiplier = 1;
    int state_dim = m_spCfg->getSsDim();
    for (int i = 0; i < state_dim; ++i) {
        result += (idx[i] - 1) * multiplier;
        multiplier *= (int)X_widthPerDimension[i];
    }
    return result;
}

bool pfacesKernel_mono_synth::xInSafeSet(const int* x_idx, const std::vector<int>& safe_set_basis, int safe_set_size, int state_dim) const {
    if (x_idx[0] == -1) return false;
    for (int i = safe_set_size - 1; i >= 0; --i) {
        bool dominated = true;
        for (int j = 0; j < state_dim; ++j) {
            if (x_idx[j] > safe_set_basis[i * state_dim + j]) {
                dominated = false;
                break;
            }
        }
        if (dominated) return true;
    }
    return false;
}

int pfacesKernel_mono_synth::updateSafeSet(int* unsafe_flags, std::vector<int>& safe_set_basis, std::vector<int>& safe_set_flat_indices, 
                                          int& safe_set_size, int state_dim, int total_states, int max_basis_elements) {
    const int MAX_STATE_DIM = 3;
    std::vector<unsigned char> unsafe_mask(safe_set_size, 0);
    std::vector<unsigned char> seen_neighbors(total_states, 0);
    std::vector<int> neighbor_buffer(max_basis_elements * state_dim * state_dim, 0);

    int neighbor_count = 0;
    for (int i = 0; i < safe_set_size; ++i) {
        if (!unsafe_flags[i]) continue;
        unsafe_mask[i] = 1;

        for (int j = 0; j < state_dim; ++j) {
            int val = safe_set_basis[i * state_dim + j];
            if (val <= 1) continue;

            int coords[MAX_STATE_DIM];
            for (int k = 0; k < state_dim; ++k) {
                coords[k] = safe_set_basis[i * state_dim + k] - (j == k ? 1 : 0);
            }

            int flat_idx = flattenIndex(coords);
            if (!seen_neighbors[flat_idx]) {
                seen_neighbors[flat_idx] = 1;
                int* neighbor = &neighbor_buffer[neighbor_count * state_dim];
                for (int k = 0; k < state_dim; ++k) {
                    neighbor[k] = coords[k];
                }
                neighbor_count++;
            }
        }
    }

    int write_pos = 0;
    for (int i = 0; i < safe_set_size; ++i) {
        if (!unsafe_mask[i]) {
            if (write_pos != i) {
                for (int j = 0; j < state_dim; ++j) {
                    safe_set_basis[write_pos * state_dim + j] = safe_set_basis[i * state_dim + j];
                }
                safe_set_flat_indices[write_pos] = safe_set_flat_indices[i];
            }
            write_pos++;
        }
    }
    safe_set_size = write_pos;

    int added = 0;
    for (int ni = 0; ni < neighbor_count; ++ni) {
        int* neighbor = &neighbor_buffer[ni * state_dim];
        if (!xInSafeSet(neighbor, safe_set_basis, safe_set_size, state_dim)) {
            if (safe_set_size >= max_basis_elements) break;
            for (int j = 0; j < state_dim; ++j) {
                safe_set_basis[safe_set_size * state_dim + j] = neighbor[j];
            }
            safe_set_flat_indices[safe_set_size] = flattenIndex(neighbor);
            safe_set_size++;
            added++;
        }
    }
    return added;
}

/* not providing implementation of the virtual method: configureTuneParallelProgram*/
void pfacesKernel_mono_synth::configureTuneParallelProgram(pfacesParallelProgram&, size_t) {
}

} // namespace mono_synth

PFACES_REGISTER_LOADABLE_KERNEL(mono_synth::pfacesKernel_mono_synth)



