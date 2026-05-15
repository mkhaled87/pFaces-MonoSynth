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
#include <cstdio>
#include <chrono>
#include <algorithm>

#include "pfacesKernel_mono_synth.h"

namespace mono_synth {

static inline bool getPackedBit(const std::vector<cl_uint>& words, size_t idx) {
    return (words[idx >> 5] & (1u << (idx & 31u))) != 0;
}


/* a call-back function to save the controller/abstraction after the kernel finishes */
size_t pfacesKernel_mono_synth::saveTransitionTable(void* pPackedKernel, void* pPackedParallelProgram) {
	
	// retrieving the required values
	const static pfacesParallelProgram*  pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;
	const char* pDataTransitionTable = pParallelProgram->m_dataPool[0].first;
	int ss_dim = ((pfacesKernel_mono_synth*)(pPackedKernel))->m_spCfg->getSsDim();
	int number_of_states = ((pfacesKernel_mono_synth*)(pPackedKernel))->x_flat_width;
	int number_of_elements = number_of_states;

	// save to file
	const char* file_path = ((pfacesKernel_mono_synth*)(pPackedKernel))->cache_file;
	std::cout << "Saving transitions to file: " << file_path << std::endl;
	std::ofstream cache_out(file_path, std::ios::binary);
	if (cache_out.good()) {
		cache_out.write(reinterpret_cast<const char*>(&number_of_states), sizeof(decltype(number_of_states)));
		cache_out.write(pDataTransitionTable, number_of_elements*sizeof(cl_uint));
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
	int number_of_elements = number_of_states;
	std::ifstream cache_in(file_path, std::ios::binary);
	if (cache_in.good()) {
		int cached_total_states;
		cache_in.read(reinterpret_cast<char*>(&cached_total_states), sizeof(int));
		
		if (cached_total_states == number_of_states) {
			cache_in.read(reinterpret_cast<char*>(pDataTransitionTable), sizeof(cl_uint) * number_of_elements);
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
    params.push_back("@@BITMAP_WORD_COUNT@@");
    paramvals.push_back(std::to_string(m_bitmap_word_count));

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

    // TT-only GPU kernel parameters
    params.push_back("@@THRESHOLD_D_STAR@@");
    paramvals.push_back(std::to_string(m_threshold_d_star));
    params.push_back("@@THRESHOLD_TABLE_SIZE@@");
    paramvals.push_back(std::to_string(m_threshold_table_size));

    // Integer grid sizes array (avoids float→int conversion on GPU)
    std::stringstream ss_grid;
    ss_grid << "{";
    for (size_t i = 0; i < m_spCfg->getSsDim(); ++i) {
        if (i > 0) ss_grid << ",";
        ss_grid << (int)X_widthPerDimension[i];
    }
    ss_grid << "}";
    params.push_back("@@GRID_SIZES_ARRAY@@");
    paramvals.push_back(ss_grid.str());

    // Inline dynamics: compute transitions on-the-fly in the GFP kernel.
    params.push_back("@@USE_INLINE_DYNAMICS@@");
    paramvals.push_back(std::to_string(
        (m_use_inline_dynamics && ((m_use_tt_only && m_use_tt_only_gpu) || m_use_bitmap_gfp)) ? 1 : 0));

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
	
	// Read benchmark count from config (default: 10)
	m_benchmark_count = (int)m_spCfg->getBenchmarkCount();
	
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
		m_spCfg->getSsErr(), X_widthPerDimension).toUnsignedLong();
	
	// Read mode flags early so they can be used for memory allocation decisions
	m_use_tt_only = m_spCfg->isUseTTOnly();
	m_use_tt_only_gpu = m_spCfg->isUseTTOnlyGPU();
	m_use_inline_dynamics = m_spCfg->isUseInlineDynamics();
	m_use_prefix_sweep = m_spCfg->isUsePrefixSweep();
	m_use_bitmap_gfp = m_spCfg->isUseBitmapGFP();
	m_extract_basis = m_spCfg->isExtractBasis();
	m_bitmap_word_count = (x_flat_width + 31u) / 32u;
	const bool use_inline_transition_free =
		m_use_inline_dynamics && ((m_use_tt_only && m_use_tt_only_gpu) || m_use_bitmap_gfp);

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
    // Basis-method-only arrays: skip in TT-only GPU mode to avoid OOM
    // (e.g. coord_buckets at N=10000: 3×10001×1M×4B = 120 GB)
    if (!((m_use_tt_only && m_use_tt_only_gpu) || m_use_bitmap_gfp)) {
        m_unsafe_mask = std::vector<unsigned char>(MAX_BASIS_ELEMENTS, 0);
        m_neighbor_buffer = std::vector<int>(MAX_BASIS_ELEMENTS * MAX_STATE_DIM * MAX_STATE_DIM, 0);
        m_neighbor_parent_dim = std::vector<int>(MAX_BASIS_ELEMENTS * MAX_STATE_DIM, 0);
        m_neighbor_parent_coord = std::vector<int>(MAX_BASIS_ELEMENTS * MAX_STATE_DIM, 0);
        m_seen_neighbors = std::vector<unsigned char>(x_flat_width, 0);
        // Allocate 3D array for coord_buckets
        m_coord_buckets = std::vector<std::vector<std::vector<int>>>(MAX_STATE_DIM, std::vector<std::vector<int>>(MAX_COORD_VALUE, std::vector<int>(MAX_BUCKET_SIZE, 0)));
        // Allocate 2D array for bucket_sizes
        m_bucket_sizes = std::vector<std::vector<int>>(MAX_STATE_DIM, std::vector<int>(MAX_COORD_VALUE, 0));
    }

	// Initialize threshold table for CPU-side O(1) safety check
	m_use_threshold_table = m_spCfg->isUseThresholdTable();
	const int d_star_override = m_spCfg->getThresholdDStarOverride();
	if (d_star_override >= 0 && d_star_override < (int)ssDim) {
		m_threshold_d_star = d_star_override;
		std::cout << "[MonoSynth] threshold_d_star override enabled: d*=" << m_threshold_d_star << std::endl;
	} else {
		m_threshold_d_star = 0;
		for (int d = 1; d < (int)ssDim; ++d) {
			if (X_widthPerDimension[d] > X_widthPerDimension[m_threshold_d_star])
				m_threshold_d_star = d;
		}
	}
	m_threshold_orig_to_key_stride.resize(ssDim, 0);
	int tbl_stride = 1;
	for (int d = 0; d < (int)ssDim; ++d) {
		if (d == m_threshold_d_star) continue;
		m_threshold_key_dim_indices.push_back(d);
		m_threshold_key_dims.push_back((int)X_widthPerDimension[d]);
		m_threshold_orig_to_key_stride[d] = tbl_stride;
		tbl_stride *= (int)X_widthPerDimension[d];
	}
	m_threshold_table_size = tbl_stride;
	m_threshold_table.resize(m_threshold_table_size, 0);
	if (m_use_threshold_table) {
		std::cout << "[MonoSynth] Using CPU threshold table safety check (d*=" << m_threshold_d_star
				  << ", table=" << m_threshold_table_size << " entries, "
				  << m_threshold_table_size * (int)sizeof(int) << " bytes)" << std::endl;
	}

	// TT-only mode: entire GFP iteration via threshold table, no basis tracking
	// (flags already read above for early memory allocation decisions)
	if (m_use_tt_only) {
		m_use_threshold_table = true;  // TT-only implies threshold table
		std::cout << "[MonoSynth] TT-only mode (" << (m_use_tt_only_gpu ? "GPU" : "CPU") << "): threshold-table-only GFP iteration (no neighbor generation)" << std::endl;
		if (m_use_inline_dynamics && m_use_tt_only_gpu) {
			std::cout << "[MonoSynth] Inline dynamics enabled: computing transitions on-the-fly during GFP (skipping precompute kernel)" << std::endl;
		}
		std::cout << "[MonoSynth] Prefix-max sweep: " << (m_use_prefix_sweep ? "ENABLED" : "disabled (relying on monotone boundary handling)") << std::endl;
	}
	if (m_use_bitmap_gfp) {
		m_use_threshold_table = false;
		std::cout << "[MonoSynth] Bitmap GFP mode: full-state bit-packed GFP iteration"
		          << " (" << m_bitmap_word_count << " uint32 words, "
		          << (m_bitmap_word_count * sizeof(cl_uint)) << " bytes per bitmap)" << std::endl;
		if (m_use_inline_dynamics) {
			std::cout << "[MonoSynth] Inline dynamics enabled: bitmap GFP computes transitions on-the-fly (skipping precompute kernel)" << std::endl;
		}
	}

	// Setup basis evolution recording if enabled
	m_record_basis_evolution = m_spCfg->isRecordBasisEvolution();

	// Skip transition cache if save_transitions is false
	if (!m_spCfg->isSaveTransitions()) {
		m_skip_cache = true;
	}

	// Always write per-iteration timing stats (lightweight: one row per iteration)
	m_iteration_stats_csv_file.open("iteration_stats.csv");
	if (m_iteration_stats_csv_file.is_open()) {
		m_iteration_stats_csv_file
			<< "run,iteration,basis_in,unsafe_count,neighbor_count,added_count,basis_out,"
			<< "iter_total_ms,safety_phase_ms,csv_io_ms,tt_build_ms,tt_lookup_ms,host_update_ms,clear_ms,neighbor_gen_ms,"
			<< "compact_ms,rebuild_ms,add_neighbors_ms\n";
	} else {
		std::cerr << "[MonoSynth] Warning: Could not open iteration_stats.csv" << std::endl;
	}

	// Optionally write full per-element basis evolution (expensive: one row per basis element per iteration)
	if (m_record_basis_evolution) {
		m_basis_csv_buf.resize(256 * 1024);
		m_basis_csv_file.rdbuf()->pubsetbuf(m_basis_csv_buf.data(), m_basis_csv_buf.size());
		m_basis_csv_file.open("basis_coordinates.csv");
		if (m_basis_csv_file.is_open()) {
			m_basis_csv_file << "iteration";
			for (int i = 0; i < (int)ssDim; ++i) {
				m_basis_csv_file << ",idx" << i;
			}
			m_basis_csv_file << "\n";
			std::cout << "[MonoSynth] Recording basis evolution to basis_coordinates.csv" << std::endl;
		} else {
			std::cerr << "[MonoSynth] Warning: Could not open basis_coordinates.csv" << std::endl;
			m_record_basis_evolution = false;
		}
	}

	// Optionally write threshold table evolution (TT-only mode)
	if (m_record_basis_evolution && m_use_tt_only) {
		m_threshold_csv_buf.resize(256 * 1024);
		m_threshold_csv_file.rdbuf()->pubsetbuf(m_threshold_csv_buf.data(), m_threshold_csv_buf.size());
		m_threshold_csv_file.open("threshold_evolution.csv");
		if (m_threshold_csv_file.is_open()) {
			m_threshold_csv_file << "iteration,key_flat,tau";
			for (int d = 0; d < (int)ssDim; ++d) {
				if (d == m_threshold_d_star) continue;
				m_threshold_csv_file << ",idx" << d;
			}
			m_threshold_csv_file << ",idx" << m_threshold_d_star << "\n";
			m_threshold_csv_file.flush();
			std::cout << "[MonoSynth] Recording threshold evolution to threshold_evolution.csv" << std::endl;
		} else {
			std::cerr << "[MonoSynth] Warning: Could not open threshold_evolution.csv" << std::endl;
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
    precomputeArgs.m_baseTypeMultiple = { (size_t)x_flat_width, 4 };
	// In inline dynamics mode, the transition table is unused. Allocate minimal buffer
	// to avoid OOM on very large grids (3B+ cells × 3 dims × 4 bytes = 36+ GB).
	if (use_inline_transition_free) {
		precomputeArgs.m_baseTypeMultiple[0] = 1;
	}
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
    safetyArgs.m_baseTypeMultiple = { (size_t)MAX_BASIS_ELEMENTS, (size_t)x_flat_width, (size_t)(MAX_BASIS_ELEMENTS * ssDim), 1, (size_t)MAX_BASIS_ELEMENTS };
	if (m_use_bitmap_gfp) {
		safetyArgs.m_baseTypeMultiple = { 1, 1, 1, 1, 1 };
	} else if (use_inline_transition_free) {
		safetyArgs.m_baseTypeMultiple[1] = 1;
	}
	addKernelFunction(pfacesKernelFunction(KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_NAME, safetyArgs));

	// GPU TT-only column update kernel (func idx 2)
	std::string ttColMem = packPath + "tt_only_column_update.mem";
	std::cout << "[MonoSynth] Loading memory fingerprint from: " << ttColMem << std::endl;
	auto ttColArgs = pfacesKernelFunctionArguments::loadFromFile(
		ttColMem,
		KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNC_NAME,
		{ "next_state_table", "threshold_table_in", "threshold_table_out", "changed_flag", "runtime_params" },
		true);
	ttColArgs.m_baseTypeMultiple = { (size_t)x_flat_width, (size_t)m_threshold_table_size, (size_t)m_threshold_table_size, 1, 4 };
	if (m_use_bitmap_gfp) {
		ttColArgs.m_baseTypeMultiple = { 1, 1, 1, 1, 4 };
	} else if (use_inline_transition_free) {
		ttColArgs.m_baseTypeMultiple[0] = 1;
	}
	addKernelFunction(pfacesKernelFunction(KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNC_NAME, ttColArgs));

	// GPU TT-only prefix-max sweep kernel (func idx 3)
	std::string ttPfxMem = packPath + "tt_only_prefix_max.mem";
	std::cout << "[MonoSynth] Loading memory fingerprint from: " << ttPfxMem << std::endl;
	auto ttPfxArgs = pfacesKernelFunctionArguments::loadFromFile(
		ttPfxMem,
		KERNEL_MONO_SYNTH_TT_PREFIX_MAX_FUNC_NAME,
		{ "threshold_table", "sweep_params" },
		true);
	ttPfxArgs.m_baseTypeMultiple = { (size_t)(m_use_bitmap_gfp ? 1 : m_threshold_table_size), 2 };
	addKernelFunction(pfacesKernelFunction(KERNEL_MONO_SYNTH_TT_PREFIX_MAX_FUNC_NAME, ttPfxArgs));

	// GPU bitmap GFP iterate kernel (func idx 4)
	std::string bitmapIterMem = packPath + "bitmap_gfp_iterate.mem";
	std::cout << "[MonoSynth] Loading memory fingerprint from: " << bitmapIterMem << std::endl;
	auto bitmapIterArgs = pfacesKernelFunctionArguments::loadFromFile(
		bitmapIterMem,
		KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNC_NAME,
		{ "next_state_table", "bitmap_in", "bitmap_out", "changed_flag", "runtime_params" },
		true);
	size_t bitmap_words_alloc = m_use_bitmap_gfp ? m_bitmap_word_count : 1;
	bitmapIterArgs.m_baseTypeMultiple = {
		use_inline_transition_free ? (size_t)1 : (size_t)x_flat_width,
		bitmap_words_alloc,
		bitmap_words_alloc,
		1,
		4
	};
	addKernelFunction(pfacesKernelFunction(KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNC_NAME, bitmapIterArgs));

	// GPU bitmap GFP advance kernel (func idx 5)
	std::string bitmapAdvanceMem = packPath + "bitmap_gfp_advance.mem";
	std::cout << "[MonoSynth] Loading memory fingerprint from: " << bitmapAdvanceMem << std::endl;
	auto bitmapAdvanceArgs = pfacesKernelFunctionArguments::loadFromFile(
		bitmapAdvanceMem,
		KERNEL_MONO_SYNTH_BITMAP_GFP_ADVANCE_FUNC_NAME,
		{ "bitmap_in", "bitmap_out", "changed_flag" },
		true);
	bitmapAdvanceArgs.m_baseTypeMultiple = { bitmap_words_alloc, bitmap_words_alloc, 1 };
	addKernelFunction(pfacesKernelFunction(KERNEL_MONO_SYNTH_BITMAP_GFP_ADVANCE_FUNC_NAME, bitmapAdvanceArgs));

	// GPU bitmap GFP lower-closure prefix sweep kernel (func idx 6)
	std::string bitmapPrefixMem = packPath + "bitmap_gfp_prefix.mem";
	std::cout << "[MonoSynth] Loading memory fingerprint from: " << bitmapPrefixMem << std::endl;
	auto bitmapPrefixArgs = pfacesKernelFunctionArguments::loadFromFile(
		bitmapPrefixMem,
		KERNEL_MONO_SYNTH_BITMAP_GFP_PREFIX_FUNC_NAME,
		{ "bitmap", "sweep_params" },
		true);
	bitmapPrefixArgs.m_baseTypeMultiple = { bitmap_words_alloc, 2 };
	addKernelFunction(pfacesKernelFunction(KERNEL_MONO_SYNTH_BITMAP_GFP_PREFIX_FUNC_NAME, bitmapPrefixArgs));
}

/* providing implementation of the driver of the kernel */
void pfacesKernel_mono_synth::configureParallelProgram(pfacesParallelProgram& parallelProgram) {
	pfacesParallelAdvisor parallelAdvisor(parallelProgram.getMachine(), parallelProgram.getTargetDevicesIndicies());
	size_t beVerboseLevel = parallelProgram.m_beVerboseLevel;
    const bool doBenchmark = (m_benchmark_count > 1);
	const bool use_inline_transition_free =
		m_use_inline_dynamics && ((m_use_tt_only && m_use_tt_only_gpu) || m_use_bitmap_gfp);

	// Distribute jobs
	cl::NDRange ndrPrecompute{x_flat_width, 1, 1}, ndrCheckSafety{(size_t)MAX_BASIS_ELEMENTS, 1, 1}, ndrOffset{0, 0, 0};
	job_execPrecomputeTransition = parallelAdvisor.distributeJob(*this, KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_IDX, ndrPrecompute, ndrOffset, parallelProgram.m_isFixedJobDistribution, parallelProgram.m_fixedJobDistribution, true, false, false);
    job_execCheckBasisSafety = parallelAdvisor.distributeJob(*this, KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_IDX, ndrCheckSafety, ndrOffset, parallelProgram.m_isFixedJobDistribution, parallelProgram.m_fixedJobDistribution, true, false, false);
	if (m_use_bitmap_gfp) {
		cl::NDRange ndrBitmapWords{m_bitmap_word_count, 1, 1};
		const size_t bitmap_iter_chunk_cells = 1000000000ULL;
		for (size_t chunk_start = 0; chunk_start < x_flat_width; chunk_start += bitmap_iter_chunk_cells) {
			const size_t chunk_cells = std::min(bitmap_iter_chunk_cells, x_flat_width - chunk_start);
			cl::NDRange ndrBitmapIter{chunk_cells, 1, 1};
			cl::NDRange ndrBitmapIterOffset{chunk_start, 0, 0};
			auto chunkJobs = parallelAdvisor.distributeJob(*this, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNC_IDX, ndrBitmapIter, ndrBitmapIterOffset, parallelProgram.m_isFixedJobDistribution, parallelProgram.m_fixedJobDistribution, true, false, false);
			job_execBitmapGFPIterate.insert(job_execBitmapGFPIterate.end(), chunkJobs.begin(), chunkJobs.end());
		}
		job_execBitmapGFPAdvance = parallelAdvisor.distributeJob(*this, KERNEL_MONO_SYNTH_BITMAP_GFP_ADVANCE_FUNC_IDX, ndrBitmapWords, ndrOffset, parallelProgram.m_isFixedJobDistribution, parallelProgram.m_fixedJobDistribution, true, false, false);
		const int ss_dim = (int)m_spCfg->getSsDim();
		job_execBitmapGFPPrefix.resize(ss_dim);
		for (int d = 0; d < ss_dim; ++d) {
			size_t num_fibers = x_flat_width / (size_t)X_widthPerDimension[d];
			cl::NDRange ndrPrefix{num_fibers, 1, 1};
			job_execBitmapGFPPrefix[d] = parallelAdvisor.distributeJob(*this, KERNEL_MONO_SYNTH_BITMAP_GFP_PREFIX_FUNC_IDX, ndrPrefix, ndrOffset, parallelProgram.m_isFixedJobDistribution, parallelProgram.m_fixedJobDistribution, true, false, false);
		}
	}

	if (beVerboseLevel >= 2)
		parallelAdvisor.printTaskSchedulingReport(parallelProgram.getMachine(), { KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NAME }, { job_execPrecomputeTransition }, x_flat_width);

	// Memory allocation
	std::vector<std::pair<char*, size_t>> dataPool;
	pFacesMemoryAllocationReport memReport = allocateMemory(dataPool, parallelProgram.getMachine(), parallelProgram.getTargetDevicesIndicies(), 1, true);
    if (beVerboseLevel >= 2) memReport.PrintReport();

	// IO jobs
    const cl::Device& dataAccessDevice = parallelProgram.getTargetDevices()[0];
    job_readNextStateTable = std::make_shared<pfacesDeviceReadJob>(dataAccessDevice, 0, 2, 0);
    job_writeNextStateTable = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 0, 2, 0);
    job_writeRuntimeParams = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 0, 2, 1);
    job_readUnsafeFlags = std::make_shared<pfacesDeviceReadJob>(dataAccessDevice, 1, 5, 4);
    job_writeBasisFlatIdx = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 1, 5, 0);
    job_writeBasisList = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 1, 5, 2);
    job_writeBasisListSize = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 1, 5, 3);
	if (m_use_bitmap_gfp) {
		job_writeBitmapIn = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNC_IDX, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNC_NUM_ARGS, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNCARG_BITMAP_IN_IDX);
		job_readBitmapIn = std::make_shared<pfacesDeviceReadJob>(dataAccessDevice, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNC_IDX, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNC_NUM_ARGS, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNCARG_BITMAP_IN_IDX);
		job_writeBitmapOut = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNC_IDX, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNC_NUM_ARGS, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNCARG_BITMAP_OUT_IDX);
		job_writeBitmapChangedFlag = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNC_IDX, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNC_NUM_ARGS, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNCARG_CHANGED_FLAG_IDX);
		job_readBitmapChangedFlag = std::make_shared<pfacesDeviceReadJob>(dataAccessDevice, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNC_IDX, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNC_NUM_ARGS, KERNEL_MONO_SYNTH_BITMAP_GFP_ITERATE_FUNCARG_CHANGED_FLAG_IDX);
		const int ss_dim = (int)m_spCfg->getSsDim();
		job_writeBitmapSweepParams.resize(ss_dim);
		for (int d = 0; d < ss_dim; ++d) {
			job_writeBitmapSweepParams[d] = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, KERNEL_MONO_SYNTH_BITMAP_GFP_PREFIX_FUNC_IDX, KERNEL_MONO_SYNTH_BITMAP_GFP_PREFIX_FUNC_NUM_ARGS, KERNEL_MONO_SYNTH_BITMAP_GFP_PREFIX_FUNCARG_SWEEP_PARAMS_IDX);
		}
	}

    instr_readNextStateTable->setAsReadDeviceBuffer(job_readNextStateTable);
    instr_writeNextStateTable->setAsWriteDeviceBuffer(job_writeNextStateTable);
    instr_writeRuntimeParams->setAsWriteDeviceBuffer(job_writeRuntimeParams);
    instr_readUnsafeFlags->setAsReadDeviceBuffer(job_readUnsafeFlags);
    instr_writeBasisFlatIdx->setAsWriteDeviceBuffer(job_writeBasisFlatIdx);
    instr_writeBasisList->setAsWriteDeviceBuffer(job_writeBasisList);
    instr_writeBasisListSize->setAsWriteDeviceBuffer(job_writeBasisListSize);
	if (m_use_bitmap_gfp) {
		instr_writeBitmapIn->setAsWriteDeviceBuffer(job_writeBitmapIn);
		instr_readBitmapIn->setAsReadDeviceBuffer(job_readBitmapIn);
		instr_writeBitmapOut->setAsWriteDeviceBuffer(job_writeBitmapOut);
		instr_writeBitmapChangedFlag->setAsWriteDeviceBuffer(job_writeBitmapChangedFlag);
		instr_readBitmapChangedFlag->setAsReadDeviceBuffer(job_readBitmapChangedFlag);
	}

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

	if (use_inline_transition_free) {
		// Inline dynamics mode: no precompute needed.
		// Transitions are computed on-the-fly in the selected GFP kernel.
		instructionList.push_back(std::make_shared<pfacesInstruction>());
		instructionList.back()->setAsBlockingSyncPoint();
		auto instr_noPrecompute = std::make_shared<pfacesInstruction>();
		instr_noPrecompute->setAsHostFunction([](void* pK, void*) -> size_t {
			auto* k = (pfacesKernel_mono_synth*)pK;
			k->m_precompute_ms = 0.0;
			return 0;
		}, "inlineDynamicsNoPrecompute");
		instructionList.push_back(instr_noPrecompute);
	} else if (useCache) {
        instructionList.push_back(instr_BlockingSyncPoint);
		instr_hostFuncLoadNextStateTable->setAsHostFunction(pfacesKernel_mono_synth::loadTransitionTable, "loadTransitionTable");
		instructionList.push_back(instr_hostFuncLoadNextStateTable);
		instructionList.push_back(instr_writeNextStateTable);
	} else {
		// --- Conditional precompute skip (for RT controller re-synthesis) ---
		// checkSkipPrecompute returns 1 → skip precompute (jump past it).
		// Returns 0 → fall through and execute precompute.
		instructionList.push_back(instr_BlockingSyncPoint);
		auto instr_checkSkip = std::make_shared<pfacesInstruction>();
		instr_checkSkip->setAsHostFunction(pfacesKernel_mono_synth::checkSkipPrecompute, "checkSkipPrecompute");
		instructionList.push_back(instr_checkSkip);
		// Mark jump target placeholder — will be patched after precompute block
		size_t jump_skip_precompute_idx = instructionList.size();
		auto instr_jumpSkip = std::make_shared<pfacesInstruction>();
		instructionList.push_back(instr_jumpSkip);  // placeholder

		// Start timing precompute
		instructionList.push_back(std::make_shared<pfacesInstruction>());
		instructionList.back()->setAsBlockingSyncPoint();
		auto instr_preTimerStart = std::make_shared<pfacesInstruction>();
		instr_preTimerStart->setAsHostFunction([](void* pK, void*) -> size_t {
			auto* k = (pfacesKernel_mono_synth*)pK;
			k->m_phase_timer = std::chrono::high_resolution_clock::now();
			return 0;
		}, "precomputeTimerStart");
		instructionList.push_back(instr_preTimerStart);

		for (auto& job : job_execPrecomputeTransition) {
			auto instr = std::make_shared<pfacesInstruction>();
			instr->setAsDeviceExecute(job);
			instructionList.push_back(instr);
		}
		instructionList.push_back(instr_BlockingSyncPoint);

		// In GPU TT-only mode, skip reading transition table back to host.
		// The column_update kernel reads next_state_table directly from device memory.
		if (!((m_use_tt_only && m_use_tt_only_gpu) || m_use_bitmap_gfp)) {
			instructionList.push_back(instr_readNextStateTable);
			instructionList.push_back(instr_BlockingSyncPoint);

			if (!m_skip_cache) {
				instr_hostFuncSaveTransitions->setAsHostFunction(pfacesKernel_mono_synth::saveTransitionTable, "saveTransitionTable");
				instructionList.push_back(instr_hostFuncSaveTransitions);
			}
		}

		// End timing precompute
		instructionList.push_back(std::make_shared<pfacesInstruction>());
		instructionList.back()->setAsBlockingSyncPoint();
		auto instr_preTimerEnd = std::make_shared<pfacesInstruction>();
		instr_preTimerEnd->setAsHostFunction(pfacesKernel_mono_synth::timerAfterPrecompute, "timerAfterPrecompute");
		instructionList.push_back(instr_preTimerEnd);

		// Patch the jump: skip to here when checkSkipPrecompute returned 1
		size_t after_precompute_idx = instructionList.size();
		instr_jumpSkip->setAsJumpNe(after_precompute_idx);
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

        benchmark_loop_start = instructionList.size();
    }

    // Safe set iteration — two modes:
    // (A) GPU TT-only: binary-search columns + prefix-max sweep on device
    // (B) Standard: basis-tracking with optional CPU threshold table
    if (m_use_bitmap_gfp) {
        // === GPU bitmap GFP iteration ===
        // Data pool indices for bitmap buffers:
        //   func4 (bitmap_gfp_iterate): arg0=next_state_table(resident),
        //   arg1=bitmap_in, arg2=bitmap_out, arg3=changed_flag,
        //   arg4=runtime_params(resident).
        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();

        instr_hostFuncInitBitmapGFP->setAsHostFunction(pfacesKernel_mono_synth::initBitmapGFP, "initBitmapGFP");
        instructionList.push_back(instr_hostFuncInitBitmapGFP);
        instructionList.push_back(instr_writeBitmapIn);
        instructionList.push_back(instr_writeBitmapOut);
        instructionList.push_back(instr_writeBitmapChangedFlag);
        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();

        size_t bitmap_loop_start = instructionList.size();

        instr_hostFuncPrepareBitmapGFPIteration->setAsHostFunction(pfacesKernel_mono_synth::prepareBitmapGFPIteration, "prepareBitmapGFPIteration");
        instructionList.push_back(instr_hostFuncPrepareBitmapGFPIteration);
        instructionList.push_back(instr_writeBitmapChangedFlag);

        for (auto& job : job_execBitmapGFPIterate) {
            auto instr = std::make_shared<pfacesInstruction>();
            instr->setAsDeviceExecute(job);
            instructionList.push_back(instr);
        }

        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();

        // Canonicalize the predecessor result as a lower-closed bitmap, matching
        // the invariant used by the threshold-table and basis methods.
        m_bitmap_gfp_sweep_dim_idx = 0;
        for (int d = 0; d < (int)m_spCfg->getSsDim(); ++d) {
            auto instr_set_sweep = std::make_shared<pfacesInstruction>();
            instr_set_sweep->setAsHostFunction(pfacesKernel_mono_synth::setBitmapSweepParams, "setBitmapSweepParams");
            instructionList.push_back(instr_set_sweep);

            auto instr_ws = std::make_shared<pfacesInstruction>();
            instr_ws->setAsWriteDeviceBuffer(job_writeBitmapSweepParams[d]);
            instructionList.push_back(instr_ws);

            for (auto& job : job_execBitmapGFPPrefix[d]) {
                auto instr = std::make_shared<pfacesInstruction>();
                instr->setAsDeviceExecute(job);
                instructionList.push_back(instr);
            }

            instructionList.push_back(std::make_shared<pfacesInstruction>());
            instructionList.back()->setAsBlockingSyncPoint();
        }

        // Keep both bitmaps resident. This device-side advance copies
        // S_{k+1} into bitmap_in, clears bitmap_out for the next pass, and
        // compares the full post-closure bitmap against S_k for convergence.
        for (auto& job : job_execBitmapGFPAdvance) {
            auto instr = std::make_shared<pfacesInstruction>();
            instr->setAsDeviceExecute(job);
            instructionList.push_back(instr);
        }
        instructionList.push_back(instr_readBitmapChangedFlag);
        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();

        instr_hostFuncProcessBitmapGFPUpdate->setAsHostFunction(pfacesKernel_mono_synth::processBitmapGFPUpdate, "processBitmapGFPUpdate");
        instructionList.push_back(instr_hostFuncProcessBitmapGFPUpdate);

        auto instr_jumpBitmap = std::make_shared<pfacesInstruction>();
        instr_jumpBitmap->setAsJumpNe(bitmap_loop_start);
        instructionList.push_back(instr_jumpBitmap);

        // Final bitmap is resident in bitmap_in after the last advance.
        instructionList.push_back(instr_readBitmapIn);
        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();
        instr_hostFuncFinalizeBitmapGFP->setAsHostFunction(pfacesKernel_mono_synth::finalizeBitmapGFP, "finalizeBitmapGFP");
        instructionList.push_back(instr_hostFuncFinalizeBitmapGFP);

        if (doBenchmark) {
            instructionList.push_back(std::make_shared<pfacesInstruction>());
            instructionList.back()->setAsBlockingSyncPoint();

            instr_hostFuncBenchmarkNext->setAsHostFunction(pfacesKernel_mono_synth::benchmarkNext, "benchmarkNext");
            instructionList.push_back(instr_hostFuncBenchmarkNext);

            instr_jumpToBenchmarkStart->setAsJumpNe(benchmark_loop_start);
            instructionList.push_back(instr_jumpToBenchmarkStart);
        }
    } else if (m_use_tt_only && m_use_tt_only_gpu) {
        // === GPU TT-only iteration ===
        // Data pool indices for TT-only buffers:
        //   func2 (tt_only_column_update): arg0=next_state_table(resident), arg1=tt_in(Pool[6]), arg2=tt_out(Pool[7]), arg3=changed_flag(Pool[8])
        //   func3 (tt_only_prefix_max):    arg0=threshold_table(resident=func2.arg2=Pool[7]), arg1=sweep_params(Pool[9])
        cl::NDRange ndrTTCol{(size_t)m_threshold_table_size, 1, 1};
        job_execTTColumnUpdate = parallelAdvisor.distributeJob(*this, KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNC_IDX, ndrTTCol, ndrOffset, parallelProgram.m_isFixedJobDistribution, parallelProgram.m_fixedJobDistribution, true, false, false);

        // Distribute prefix-max jobs (one per key dimension, different NDRange each).
        // Only prepared if the sweep is enabled; otherwise the column update alone
        // produces a lower-closed threshold (see Assumption 1 / paper Sec. III).
        const int num_key_dims = (int)m_threshold_key_dims.size();
        if (m_use_prefix_sweep) {
            job_execTTPrefixMax.resize(num_key_dims);
            for (int ki = 0; ki < num_key_dims; ++ki) {
                size_t num_fibers = (size_t)m_threshold_table_size / (size_t)m_threshold_key_dims[ki];
                cl::NDRange ndrPfx{num_fibers, 1, 1};
                job_execTTPrefixMax[ki] = parallelAdvisor.distributeJob(*this, KERNEL_MONO_SYNTH_TT_PREFIX_MAX_FUNC_IDX, ndrPfx, ndrOffset, parallelProgram.m_isFixedJobDistribution, parallelProgram.m_fixedJobDistribution, true, false, false);
            }
        }

        // IO jobs for TT-only buffers
        job_writeTTIn = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNC_IDX, KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNC_NUM_ARGS, KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNCARG_TT_IN_IDX);
        job_readTTOut = std::make_shared<pfacesDeviceReadJob>(dataAccessDevice, KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNC_IDX, KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNC_NUM_ARGS, KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNCARG_TT_OUT_IDX);
        job_writeChangedFlag = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNC_IDX, KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNC_NUM_ARGS, KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNCARG_CHANGED_FLAG_IDX);
        job_readChangedFlag = std::make_shared<pfacesDeviceReadJob>(dataAccessDevice, KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNC_IDX, KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNC_NUM_ARGS, KERNEL_MONO_SYNTH_TT_COLUMN_UPDATE_FUNCARG_CHANGED_FLAG_IDX);

        // Sweep params: one write job per key dimension.
        // We reuse the same buffer but write different params each time.
        if (m_use_prefix_sweep) {
            job_writeSweepParams.resize(num_key_dims);
            for (int ki = 0; ki < num_key_dims; ++ki) {
                job_writeSweepParams[ki] = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, KERNEL_MONO_SYNTH_TT_PREFIX_MAX_FUNC_IDX, KERNEL_MONO_SYNTH_TT_PREFIX_MAX_FUNC_NUM_ARGS, KERNEL_MONO_SYNTH_TT_PREFIX_MAX_FUNCARG_SWEEP_PARAMS_IDX);
            }
        }

        instr_writeTTIn->setAsWriteDeviceBuffer(job_writeTTIn);
        instr_readTTOut->setAsReadDeviceBuffer(job_readTTOut);
        instr_writeChangedFlag->setAsWriteDeviceBuffer(job_writeChangedFlag);
        instr_readChangedFlag->setAsReadDeviceBuffer(job_readChangedFlag);

        // Init: fill threshold table with N_d* and write to device
        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();

        instr_hostFuncInitTTGPU->setAsHostFunction(pfacesKernel_mono_synth::initTTGPU, "initTTGPU");
        instructionList.push_back(instr_hostFuncInitTTGPU);

        // --- iteration loop start ---
        size_t tt_loop_start = instructionList.size();

        // (a) prepareTTGPUIteration: copy m_threshold_table → pool tt_in, zero changed_flag
        instr_hostFuncPrepareTTGPUIteration->setAsHostFunction(pfacesKernel_mono_synth::prepareTTGPUIteration, "prepareTTGPUIteration");
        instructionList.push_back(instr_hostFuncPrepareTTGPUIteration);

        // (b) Write tt_in + changed_flag to device
        instructionList.push_back(instr_writeTTIn);
        instructionList.push_back(instr_writeChangedFlag);

        // (c) Execute column update kernel
        for (auto& job : job_execTTColumnUpdate) {
            auto instr = std::make_shared<pfacesInstruction>();
            instr->setAsDeviceExecute(job);
            instructionList.push_back(instr);
        }

        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();

        // (d) Execute prefix-max sweep for each key dimension (if enabled).
        //     Each sweep needs its own host function to set sweep_params,
        //     then write → execute → sync. Under Assumption 1 the sweep is
        //     a no-op and is skipped entirely at this scheduling stage.
        if (m_use_prefix_sweep) {
            for (int ki = 0; ki < num_key_dims; ++ki) {
                auto instr_set_sweep = std::make_shared<pfacesInstruction>();
                instr_set_sweep->setAsHostFunction(pfacesKernel_mono_synth::setSweepParams, "setSweepParams");
                instructionList.push_back(instr_set_sweep);

                auto instr_ws = std::make_shared<pfacesInstruction>();
                instr_ws->setAsWriteDeviceBuffer(job_writeSweepParams[ki]);
                instructionList.push_back(instr_ws);

                for (auto& job : job_execTTPrefixMax[ki]) {
                    auto instr = std::make_shared<pfacesInstruction>();
                    instr->setAsDeviceExecute(job);
                    instructionList.push_back(instr);
                }

                instructionList.push_back(std::make_shared<pfacesInstruction>());
                instructionList.back()->setAsBlockingSyncPoint();
            }
        }

        // (e) Read TT_out + changed_flag back to host
        instructionList.push_back(instr_readTTOut);
        instructionList.push_back(instr_readChangedFlag);

        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();

        // (f) processTTGPUUpdate: copy pool tt_out → m_threshold_table, check convergence
        instr_hostFuncProcessTTGPUUpdate->setAsHostFunction(pfacesKernel_mono_synth::processTTGPUUpdate, "processTTGPUUpdate");
        instructionList.push_back(instr_hostFuncProcessTTGPUUpdate);

        // Jump back to loop start if not converged
        auto instr_jumpTT = std::make_shared<pfacesInstruction>();
        instr_jumpTT->setAsJumpNe(tt_loop_start);
        instructionList.push_back(instr_jumpTT);

        // Benchmark loop
        if (doBenchmark) {
            instructionList.push_back(std::make_shared<pfacesInstruction>());
            instructionList.back()->setAsBlockingSyncPoint();

            instr_hostFuncBenchmarkNext->setAsHostFunction(pfacesKernel_mono_synth::benchmarkNext, "benchmarkNext");
            instructionList.push_back(instr_hostFuncBenchmarkNext);

            instr_jumpToBenchmarkStart->setAsJumpNe(benchmark_loop_start);
            instructionList.push_back(instr_jumpToBenchmarkStart);
        }
    } else {
    // === Standard safe set iteration ===
    {
        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();
        
        instr_hostFuncInitSafeSet->setAsHostFunction(pfacesKernel_mono_synth::initSafeSet, "initSafeSet");
        instructionList.push_back(instr_hostFuncInitSafeSet);
        
        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();

        size_t loop_start = instructionList.size();
        instr_hostFuncPrepareSafeSetIteration->setAsHostFunction(pfacesKernel_mono_synth::prepareSafeSetIteration, "prepareSafeSetIteration");
        instructionList.push_back(instr_hostFuncPrepareSafeSetIteration);

        if (!m_use_threshold_table) {
        instructionList.push_back(instr_writeBasisFlatIdx);
        instructionList.push_back(instr_writeBasisList);
        instructionList.push_back(instr_writeBasisListSize);

        for (auto& job : job_execCheckBasisSafety) {
            auto instr = std::make_shared<pfacesInstruction>();
            instr->setAsDeviceExecute(job);
            instructionList.push_back(instr);
        }
        instructionList.push_back(instr_readUnsafeFlags);
        }
        
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

            instr_jumpToBenchmarkStart->setAsJumpNe(benchmark_loop_start);
            instructionList.push_back(instr_jumpToBenchmarkStart);
        }
    }
    }
    
    // (Precompute timing is folded into checkSkipPrecompute/initTTGPU — no extra sync points)
    // Post-GFP timing + sync: ensure GPU has finished before proceeding
    {
        instructionList.push_back(std::make_shared<pfacesInstruction>());
        instructionList.back()->setAsBlockingSyncPoint();
        auto instr_gfpTimer = std::make_shared<pfacesInstruction>();
        instr_gfpTimer->setAsHostFunction(pfacesKernel_mono_synth::timerAfterGFP, "timerAfterGFP");
        instructionList.push_back(instr_gfpTimer);
    }

    instructionList.push_back(std::make_shared<pfacesInstruction>());
    instructionList.back()->setAsBlockingSyncPoint();
	
	parallelProgram.m_Universal_globalNDRange = parallelProgram.m_Process_globalNDRange = ndrPrecompute;
	parallelProgram.m_Universal_offsetNDRange = parallelProgram.m_Process_offsetNDRange = ndrOffset;
    
    // Point parallelProgram.m_dataPool to our allocated dataPool
    parallelProgram.m_dataPool = dataPool;
	parallelProgram.m_spInstructionList = instructionList;
}

/* Safe Set Host Functions */
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

    // Start timing from here (includes seeding scan and array init)
    pKernel->m_compute_start = std::chrono::high_resolution_clock::now();

    // TT-only mode: initialize threshold table, skip basis tracking
    if (pKernel->m_use_tt_only) {
        const int N_dstar = (int)pKernel->X_widthPerDimension[pKernel->m_threshold_d_star];
        std::fill(pKernel->m_threshold_table.begin(), pKernel->m_threshold_table.end(), N_dstar);
        pKernel->m_safe_set_size = 0;
        pKernel->m_iterations = 0;
        std::cout << "[MonoSynth] TT-only: initialized " << pKernel->m_threshold_table_size
                  << " columns to height " << N_dstar << std::endl;
        return 0;
    }

    // Zero the arrays (important as we reuse these pooled buffers)
    std::memset(pKernel->m_safe_set_basis, 0, (size_t)pKernel->MAX_BASIS_ELEMENTS * (size_t)pKernel->MAX_STATE_DIM * sizeof(int));
    std::memset(pKernel->m_safe_set_flat_indices, 0, (size_t)pKernel->MAX_BASIS_ELEMENTS * sizeof(int));
    
    // Clear coordinate buckets (prevents stale indices from previous runs)
    for (int i = 0; i < pKernel->MAX_STATE_DIM; ++i) {
        std::memset(pKernel->m_bucket_sizes[i].data(), 0, pKernel->MAX_COORD_VALUE * sizeof(int));
    }
    
    const bool use_seeding = pKernel->m_spCfg->isBoundarySeeding();
    
    if (use_seeding) {
        // ----------------------------------------------------------
        // Transition-Table Boundary Seeding
        //
        // Instead of initializing with the single corner element, we
        // extract the maximal antichain of S_0 = {q : T[q] != bot},
        // the set of cells with valid (in-bounds) successors.
        //
        // For monotone systems, S_0 = Pre^1(X) is downward-closed
        // (Theorem 1), so its maximal elements form an antichain.
        // Since K_* ⊆ S_0 ⊆ X, starting from Bas(S_0) instead of
        // Bas(X) = {corner} skips all trivially-unsafe iterations
        // without affecting the fixed point.
        // ----------------------------------------------------------
        const unsigned int* table = (const unsigned int*)pParallelProgram->m_dataPool[0].first;
        const int ss_dim = pKernel->m_ss_dim;
        const int total = (int)pKernel->x_flat_width;
        
        // Pre-compute strides for dimension-wise neighbor lookup
        // flat_idx = sum_d (idx[d]-1) * stride[d],  stride[0]=1
        int strides[16]; // support up to 16 dimensions
        strides[0] = 1;
        for (int d = 1; d < ss_dim; ++d) {
            strides[d] = strides[d-1] * (int)pKernel->X_widthPerDimension[d-1];
        }
        
        int basis_count = 0;
        
        for (int fi = 0; fi < total; ++fi) {
            // Check if this cell has a valid transition (T[q] != bot)
            // flat_succ == 0xFFFFFFFF means invalid
            if (table[fi] == 0xFFFFFFFFu) continue;
            
            // Check if this cell is maximal in S_0: for every dimension j,
            // either q_j = N_j (at grid boundary) or the upper neighbor
            // q + e_j has an invalid transition.
            // For a downward-closed set, this is equivalent to being
            // a maximal element of the set.
            bool is_maximal = true;
            for (int j = 0; j < ss_dim; ++j) {
                // Extract coordinate idx_j (0-based position in dim j)
                int pos_j = (fi / strides[j]) % (int)pKernel->X_widthPerDimension[j];
                int idx_j = pos_j + 1; // 1-based
                
                if (idx_j < (int)pKernel->X_widthPerDimension[j]) {
                    // Upper neighbor exists; check its transition
                    int upper_fi = fi + strides[j];
                    if (table[upper_fi] != 0xFFFFFFFFu) {
                        // Upper neighbor is also valid => not maximal
                        is_maximal = false;
                        break;
                    }
                }
            }
            
            if (is_maximal) {
                if (basis_count >= pKernel->MAX_BASIS_ELEMENTS) {
                    std::cerr << "[MonoSynth] Warning: Seeded basis exceeds MAX_BASIS_ELEMENTS ("
                              << pKernel->MAX_BASIS_ELEMENTS << "), truncating." << std::endl;
                    break;
                }
                
                // Unflatten fi to multi-index (1-based)
                int temp = fi;
                for (int d = 0; d < ss_dim; ++d) {
                    int N_d = (int)pKernel->X_widthPerDimension[d];
                    pKernel->m_safe_set_basis[basis_count * ss_dim + d] = (temp % N_d) + 1;
                    temp /= N_d;
                }
                pKernel->m_safe_set_flat_indices[basis_count] = fi;
                basis_count++;
            }
        }
        
        pKernel->m_safe_set_size = basis_count;
        std::cout << "[MonoSynth] Boundary seeding: initialized with "
                  << basis_count << " basis elements (vs. 1 for corner init)" << std::endl;
    } else {
        // Original corner initialization: B = {(N1, N2, ..., Nn)}
        for (int i = 0; i < pKernel->m_ss_dim; ++i) {
            pKernel->m_safe_set_basis[i] = (int)pKernel->X_widthPerDimension[i];
        }
        pKernel->m_safe_set_size = 1;
        pKernel->m_safe_set_flat_indices[0] = pKernel->flattenIndex(pKernel->m_safe_set_basis);
    }
    
    pKernel->m_iterations = 0;

    return 0;
}

size_t pfacesKernel_mono_synth::prepareSafeSetIteration(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;
    const int ss_dim = pKernel->m_ss_dim;

    pKernel->m_iterations++;
    pKernel->m_iteration_start = std::chrono::high_resolution_clock::now();
    pKernel->m_iter_csv_io_ms = 0.0;
    pKernel->m_iter_safety_phase_ms = 0.0;
    pKernel->m_iter_tt_build_ms = 0.0;
    pKernel->m_iter_tt_lookup_ms = 0.0;
    pKernel->m_iter_update_total_ms = 0.0;

    // TT-only mode: one column-wise binary search iteration
    if (pKernel->m_use_tt_only) {
        const unsigned int* nextStateTable = (const unsigned int*)pParallelProgram->m_dataPool[0].first;
        auto iter_start = std::chrono::high_resolution_clock::now();
        pKernel->ttOnlyOneIteration(nextStateTable);
        auto iter_end = std::chrono::high_resolution_clock::now();
        pKernel->m_iter_update_total_ms = std::chrono::duration<double, std::milli>(iter_end - iter_start).count();
        pKernel->writeThresholdCSV(pKernel->m_iterations);
        return 0;
    }

    pKernel->m_iter_basis_in = pKernel->m_safe_set_size;

    int* pBasisListSize = (int*)pParallelProgram->m_dataPool[4].first;
    if (pBasisListSize) {
        *pBasisListSize = pKernel->m_safe_set_size;
    }

    // Record basis coordinates to CSV if enabled (timed separately — this is I/O, not computation)
    // Only write basis CSV for the first benchmark run (subsequent runs produce identical results)
    pKernel->m_iter_csv_io_ms = 0.0;
    if (pKernel->m_record_basis_evolution && pKernel->m_basis_csv_file.is_open()
        && pKernel->m_benchmark_current_run == 0) {
        auto csv_start = std::chrono::high_resolution_clock::now();
        const int B = pKernel->m_safe_set_size;
        const int max_chars_per_line = 12 + ss_dim * 12;
        std::string buf;
        buf.reserve(B * max_chars_per_line);
        char line[256];
        for (int i = 0; i < B; ++i) {
            int pos = snprintf(line, sizeof(line), "%d", pKernel->m_iterations);
            for (int j = 0; j < ss_dim; ++j) {
                pos += snprintf(line + pos, sizeof(line) - pos, ",%d",
                                pKernel->m_safe_set_basis[i * ss_dim + j]);
            }
            line[pos++] = '\n';
            buf.append(line, pos);
        }
        pKernel->m_basis_csv_file.write(buf.data(), buf.size());
        auto csv_end = std::chrono::high_resolution_clock::now();
        pKernel->m_iter_csv_io_ms = std::chrono::duration<double, std::milli>(csv_end - csv_start).count();
    }

    // Update ND-Range (used when GPU safety check is active)
    if (!pKernel->m_use_threshold_table) {
        cl::NDRange ndRange(pKernel->m_safe_set_size, 1, 1);
        for (auto& job : pKernel->job_execCheckBasisSafety) {
            job->getTasks()[0]->setNdRangeGlobal(ndRange);
        }
    }

    // CPU threshold table safety check: build table + check in one pass
    if (pKernel->m_use_threshold_table) {
        auto tt_build_start = std::chrono::high_resolution_clock::now();
        pKernel->buildThresholdTable();
        auto tt_build_end = std::chrono::high_resolution_clock::now();
        const unsigned int* nextStateTable = (const unsigned int*)pParallelProgram->m_dataPool[0].first;
        int* pUnsafeFlags = (int*)pParallelProgram->m_dataPool[5].first;
        auto tt_lookup_start = std::chrono::high_resolution_clock::now();
        pKernel->cpuSafetyCheck(nextStateTable, pUnsafeFlags);
        auto tt_lookup_end = std::chrono::high_resolution_clock::now();
        pKernel->m_iter_tt_build_ms = std::chrono::duration<double, std::milli>(tt_build_end - tt_build_start).count();
        pKernel->m_iter_tt_lookup_ms = std::chrono::duration<double, std::milli>(tt_lookup_end - tt_lookup_start).count();
    }

    return 0;
}

size_t pfacesKernel_mono_synth::processSafeSetUpdate(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;
    
    // TT-only mode: check convergence, extract basis when done
    if (pKernel->m_use_tt_only) {
        auto iter_end = std::chrono::high_resolution_clock::now();
        double iter_total_ms = std::chrono::duration<double, std::milli>(iter_end - pKernel->m_iteration_start).count();

        if (pKernel->m_iteration_stats_csv_file.is_open()) {
            pKernel->m_iteration_stats_csv_file
                << (pKernel->m_benchmark_current_run + 1) << ","
                << pKernel->m_iterations << ","
                << pKernel->m_threshold_table_size << ","  // columns processed
                << 0 << "," << 0 << "," << 0 << ","       // unsafe/neighbor/added N/A
                << pKernel->m_threshold_table_size << ","
                << iter_total_ms << ","
                << 0 << "," << 0 << "," << 0 << "," << 0 << ","
                << pKernel->m_iter_update_total_ms << ","
                << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "\n";
            pKernel->m_iteration_stats_csv_file.flush();
        }

        if (!pKernel->m_tt_only_changed) {
            // Converged. RT/direct table queries do not need a basis.
            if (pKernel->m_extract_basis) {
                pKernel->extractBasisFromThresholdTable();
            } else {
                pKernel->m_safe_set_size = 0;
            }

            auto compute_end = std::chrono::high_resolution_clock::now();
            double time_ms = std::chrono::duration<double, std::milli>(compute_end - pKernel->m_compute_start).count();

            pKernel->m_benchmark_total_time_ms += time_ms;
            pKernel->m_benchmark_total_iterations += pKernel->m_iterations;
            pKernel->m_benchmark_current_run++;

            std::cout << "Run " << pKernel->m_benchmark_current_run << "/" << pKernel->m_benchmark_count
                      << ": " << pKernel->m_iterations << " iterations, "
                      << pKernel->m_safe_set_size
                      << (pKernel->m_extract_basis ? " basis (extracted from TT), " : " basis (skipped), ")
                      << (int)time_ms << " ms" << std::endl;
            return 0;  // Stop
        }
        return 1;  // Continue
    }

    int* pUnsafeFlags = (int*)pParallelProgram->m_dataPool[5].first;
    int added = pKernel->updateSafeSet(pUnsafeFlags);
    auto iter_end = std::chrono::high_resolution_clock::now();
    double iter_total_ms = std::chrono::duration<double, std::milli>(iter_end - pKernel->m_iteration_start).count();
    // safety_phase = total iteration time minus host update time minus CSV I/O time
    double safety_phase_ms = iter_total_ms - pKernel->m_iter_update_total_ms - pKernel->m_iter_csv_io_ms;
    if (safety_phase_ms < 0.0) safety_phase_ms = 0.0;
    pKernel->m_iter_safety_phase_ms = safety_phase_ms;

    if (pKernel->m_iteration_stats_csv_file.is_open()) {
        pKernel->m_iteration_stats_csv_file
            << (pKernel->m_benchmark_current_run + 1) << ","
            << pKernel->m_iterations << ","
            << pKernel->m_iter_basis_in << ","
            << pKernel->m_iter_unsafe_count << ","
            << pKernel->m_iter_neighbor_count << ","
            << pKernel->m_iter_added_count << ","
            << pKernel->m_iter_basis_out << ","
            << iter_total_ms << ","
            << pKernel->m_iter_safety_phase_ms << ","
            << pKernel->m_iter_csv_io_ms << ","
            << pKernel->m_iter_tt_build_ms << ","
            << pKernel->m_iter_tt_lookup_ms << ","
            << pKernel->m_iter_update_total_ms << ","
            << pKernel->m_iter_clear_ms << ","
            << pKernel->m_iter_neighbor_gen_ms << ","
            << pKernel->m_iter_compact_ms << ","
            << pKernel->m_iter_rebuild_ms << ","
            << pKernel->m_iter_add_neighbors_ms << "\n";
        pKernel->m_iteration_stats_csv_file.flush();
    }

    if (added == 0) {
        auto compute_end = std::chrono::high_resolution_clock::now();
        double time_ms = std::chrono::duration<double, std::milli>(compute_end - pKernel->m_compute_start).count();
        
        pKernel->m_benchmark_total_time_ms += time_ms;
        pKernel->m_benchmark_total_iterations += pKernel->m_iterations;
        pKernel->m_benchmark_current_run++;

        // Compute safe cell count from threshold table (sum of all entries)
        if (pKernel->m_use_threshold_table && !pKernel->m_use_tt_only) {
            pKernel->buildThresholdTable();
            int64_t safe_count = 0;
            for (int i = 0; i < pKernel->m_threshold_table_size; ++i)
                safe_count += pKernel->m_threshold_table[i];
            std::cout << "[MonoSynth] Safe cells: " << safe_count << "/" << pKernel->x_flat_width
                      << " (" << (100.0 * safe_count / pKernel->x_flat_width) << "%)" << std::endl;
        }

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
    auto update_start = std::chrono::high_resolution_clock::now();
    
    // Clear persistent buffers
    std::memset(m_unsafe_mask.data(), 0, m_safe_set_size * sizeof(unsigned char));
    std::memset(m_seen_neighbors.data(), 0, total_states * sizeof(unsigned char));
    auto clear_end = std::chrono::high_resolution_clock::now();

    // Pass 1: Find unsafe elements and generate unique neighbors
    // Store which dimension was decremented for each neighbor (for indexed redundancy check)
    int neighbor_count = 0;
    int unsafe_count = 0;
    for (int i = 0; i < m_safe_set_size; ++i) {
        if (!unsafe_flags[i]) continue;
        m_unsafe_mask[i] = 1;
        unsafe_count++;

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
    auto neighbor_end = std::chrono::high_resolution_clock::now();

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
    auto compact_end = std::chrono::high_resolution_clock::now();

    // Pass 2.5: Rebuild coordinate index for surviving elements
    rebuildCoordIndex();
    auto rebuild_end = std::chrono::high_resolution_clock::now();

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

    auto add_end = std::chrono::high_resolution_clock::now();
    m_iter_unsafe_count = unsafe_count;
    m_iter_neighbor_count = neighbor_count;
    m_iter_added_count = added;
    m_iter_basis_out = m_safe_set_size;
    m_iter_clear_ms = std::chrono::duration<double, std::milli>(clear_end - update_start).count();
    m_iter_neighbor_gen_ms = std::chrono::duration<double, std::milli>(neighbor_end - clear_end).count();
    m_iter_compact_ms = std::chrono::duration<double, std::milli>(compact_end - neighbor_end).count();
    m_iter_rebuild_ms = std::chrono::duration<double, std::milli>(rebuild_end - compact_end).count();
    m_iter_add_neighbors_ms = std::chrono::duration<double, std::milli>(add_end - rebuild_end).count();
    m_iter_update_total_ms = std::chrono::duration<double, std::milli>(add_end - update_start).count();

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

void pfacesKernel_mono_synth::buildThresholdTable() {
    std::memset(m_threshold_table.data(), 0, m_threshold_table_size * sizeof(int));
    const int n = m_ss_dim;

    // Scatter: for each basis element, record max d_star value at its key position
    for (int i = 0; i < m_safe_set_size; ++i) {
        int key_flat = 0;
        for (int d = 0; d < n; ++d) {
            if (d == m_threshold_d_star) continue;
            key_flat += (m_safe_set_basis[i * n + d] - 1) * m_threshold_orig_to_key_stride[d];
        }
        const int b_dstar = m_safe_set_basis[i * n + m_threshold_d_star];
        if (m_threshold_table[key_flat] < b_dstar)
            m_threshold_table[key_flat] = b_dstar;
    }

    // Prefix-max sweep: for each key dimension, propagate from high to low
    const int num_key_dims = (int)m_threshold_key_dims.size();
    for (int ki = 0; ki < num_key_dims; ++ki) {
        const int N_k = m_threshold_key_dims[ki];
        const int stride_k = m_threshold_orig_to_key_stride[m_threshold_key_dim_indices[ki]];
        const int block = stride_k * N_k;

        for (int c = N_k - 2; c >= 0; --c) {
            for (int outer = 0; outer < m_threshold_table_size; outer += block) {
                for (int inner = 0; inner < stride_k; ++inner) {
                    const int idx = outer + c * stride_k + inner;
                    const int hi = idx + stride_k;
                    if (m_threshold_table[hi] > m_threshold_table[idx])
                        m_threshold_table[idx] = m_threshold_table[hi];
                }
            }
        }
    }
}

void pfacesKernel_mono_synth::cpuSafetyCheck(const unsigned int* next_state_flat, int* unsafe_flags) {
    const int n = m_ss_dim;
    const int total = (int)x_flat_width;

    for (int i = 0; i < m_safe_set_size; ++i) {
        const int flat_idx = m_safe_set_flat_indices[i];

        if (flat_idx < 0 || flat_idx >= total) {
            unsafe_flags[i] = 1;
            continue;
        }

        unsigned int flat_succ = next_state_flat[flat_idx];

        if (flat_succ == 0xFFFFFFFFu) {
            unsafe_flags[i] = 1;
            continue;
        }

        // Unflatten flat_succ to get per-coordinate 1-based indices
        int next_state[16];
        {
            unsigned int tmp = flat_succ;
            for (int d = 0; d < n; ++d) {
                next_state[d] = (int)(tmp % X_widthPerDimension[d]) + 1;
                tmp /= X_widthPerDimension[d];
            }
        }

        // Compute key flat index from successor coordinates
        int key_flat = 0;
        bool oob = false;
        for (int d = 0; d < n; ++d) {
            if (d == m_threshold_d_star) continue;
            const int c = next_state[d] - 1;
            if (c < 0 || c >= (int)X_widthPerDimension[d]) { oob = true; break; }
            key_flat += c * m_threshold_orig_to_key_stride[d];
        }

        if (oob) {
            unsafe_flags[i] = 1;
            continue;
        }

        const int threshold = m_threshold_table[key_flat];
        unsafe_flags[i] = (threshold == 0 || next_state[m_threshold_d_star] > threshold) ? 1 : 0;
    }
}

void pfacesKernel_mono_synth::writeThresholdCSV(int iteration) {
    if (!m_threshold_csv_file.is_open()) return;
    if (m_benchmark_current_run != 0) return;  // only first benchmark run

    const int n = m_ss_dim;
    const int d_star = m_threshold_d_star;
    const int table_size = m_threshold_table_size;
    const int max_chars = 12 + table_size * (12 * (n + 2));
    std::string buf;
    buf.reserve(std::min(max_chars, 4 * 1024 * 1024));
    char line[256];

    for (int key_flat = 0; key_flat < table_size; ++key_flat) {
        int tau = m_threshold_table[key_flat];
        if (tau <= 0) continue;

        int pos = snprintf(line, sizeof(line), "%d,%d,%d", iteration, key_flat, tau);

        // Unflatten key_flat to original 1-based indices for non-d_star dims
        int temp = key_flat;
        for (int d = 0; d < n; ++d) {
            if (d == d_star) continue;
            int coord = (temp % (int)X_widthPerDimension[d]) + 1;
            temp /= (int)X_widthPerDimension[d];
            pos += snprintf(line + pos, sizeof(line) - pos, ",%d", coord);
        }
        // d_star coordinate = tau
        pos += snprintf(line + pos, sizeof(line) - pos, ",%d", tau);
        line[pos++] = '\n';
        buf.append(line, pos);
    }
    m_threshold_csv_file.write(buf.data(), buf.size());
    m_threshold_csv_file.flush();
}

void pfacesKernel_mono_synth::ttOnlyOneIteration(const unsigned int* next_state_flat) {
    const int n = m_ss_dim;
    const int d_star = m_threshold_d_star;
    const int table_size = m_threshold_table_size;

    // Full-space strides (dim 0 fastest)
    int full_stride[16];
    full_stride[0] = 1;
    for (int d = 1; d < n; ++d)
        full_stride[d] = full_stride[d - 1] * (int)X_widthPerDimension[d - 1];
    const int d_star_stride = full_stride[d_star];

    // Double-buffering: read from snapshot, write to m_threshold_table
    std::vector<int> tau_in(m_threshold_table);

    m_tt_only_changed = false;

    for (int key_flat = 0; key_flat < table_size; ++key_flat) {
        int tau_old = tau_in[key_flat];
        if (tau_old == 0) continue;

        // Unflatten key_flat to original 1-based indices (d_star = 1 placeholder)
        int orig_idx[16];
        int temp = key_flat;
        for (int d = 0; d < n; ++d) {
            if (d == d_star) { orig_idx[d] = 1; continue; }
            orig_idx[d] = (temp % (int)X_widthPerDimension[d]) + 1;
            temp /= (int)X_widthPerDimension[d];
        }

        // Base flat index with d_star coordinate = 1
        int base_flat = 0;
        for (int d = 0; d < n; ++d)
            base_flat += (orig_idx[d] - 1) * full_stride[d];

        // Binary search for max safe v in [1, tau_old]
        int f = 0, lo = 1, hi = tau_old;
        while (lo <= hi) {
            int mid = (lo + hi) / 2;
            int cell_flat = base_flat + (mid - 1) * d_star_stride;

            unsigned int flat_succ = next_state_flat[cell_flat];

            if (flat_succ == 0xFFFFFFFFu) { hi = mid - 1; continue; }

            // Unflatten flat_succ to get per-coordinate 1-based indices
            int next[16];
            {
                unsigned int tmp = flat_succ;
                for (int d = 0; d < n; ++d) {
                    next[d] = (int)(tmp % X_widthPerDimension[d]) + 1;
                    tmp /= X_widthPerDimension[d];
                }
            }

            // Compute successor key flat index
            int succ_key_flat = 0;
            bool oob = false;
            int succ_stride = 1;
            for (int d = 0; d < n; ++d) {
                if (d == d_star) continue;
                int c = next[d] - 1;
                if (c < 0 || c >= (int)X_widthPerDimension[d]) { oob = true; break; }
                succ_key_flat += c * succ_stride;
                succ_stride *= (int)X_widthPerDimension[d];
            }

            int next_dstar = next[d_star];
            if (oob || next_dstar <= 0 || next_dstar > (int)X_widthPerDimension[d_star]) {
                hi = mid - 1;
                continue;
            }

            if (next_dstar <= tau_in[succ_key_flat]) {
                f = mid;
                lo = mid + 1;
            } else {
                hi = mid - 1;
            }
        }

        if (f < tau_old) {
            m_threshold_table[key_flat] = f;
        }
    }

    // Optional prefix-max sweep: restore non-increasing property.
    // Under the monotone boundary-handling assumption (Assumption 1 in the
    // paper) the column update alone yields a lower-closed threshold, so
    // the sweep is a no-op and skipped by default. Kept here as a numerical
    // safeguard for users who relax Assumption 1.
    if (m_use_prefix_sweep) {
        const int num_key_dims = (int)m_threshold_key_dims.size();
        for (int ki = 0; ki < num_key_dims; ++ki) {
            const int N_k = m_threshold_key_dims[ki];
            const int stride_k = m_threshold_orig_to_key_stride[m_threshold_key_dim_indices[ki]];
            const int block = stride_k * N_k;
            for (int c = N_k - 2; c >= 0; --c) {
                for (int outer = 0; outer < table_size; outer += block) {
                    for (int inner = 0; inner < stride_k; ++inner) {
                        const int idx = outer + c * stride_k + inner;
                        const int hi = idx + stride_k;
                        if (m_threshold_table[hi] > m_threshold_table[idx])
                            m_threshold_table[idx] = m_threshold_table[hi];
                    }
                }
            }
        }
    }

    // Convergence check: compare result vs previous table
    for (int i = 0; i < table_size; ++i) {
        if (m_threshold_table[i] != tau_in[i]) {
            m_tt_only_changed = true;
            break;
        }
    }
}

void pfacesKernel_mono_synth::extractBasisFromThresholdTable() {
    const int n = m_ss_dim;
    const int d_star = m_threshold_d_star;
    const int table_size = m_threshold_table_size;
    const int num_key_dims = (int)m_threshold_key_dim_indices.size();

    m_safe_set_size = 0;

    for (int key_flat = 0; key_flat < table_size; ++key_flat) {
        int tau = m_threshold_table[key_flat];
        if (tau <= 0) continue;

        // Check antichain condition: for each key dim, either at boundary or threshold drops
        bool is_maximal = true;
        int temp = key_flat;
        for (int ki = 0; ki < num_key_dims; ++ki) {
            int N_d = m_threshold_key_dims[ki];
            int d = m_threshold_key_dim_indices[ki];
            int k_d = temp % N_d;  // 0-based position in this key dim
            temp /= N_d;

            if (k_d + 1 < N_d) {
                int neighbor_key = key_flat + m_threshold_orig_to_key_stride[d];
                if (m_threshold_table[neighbor_key] >= tau) {
                    is_maximal = false;
                    break;
                }
            }
        }

        if (is_maximal) {
            if (m_safe_set_size >= MAX_BASIS_ELEMENTS) {
                std::cerr << "[MonoSynth] Warning: Extracted basis exceeds MAX_BASIS_ELEMENTS" << std::endl;
                break;
            }

            // Unflatten key_flat to original 1-based multi-index, d_star = tau
            temp = key_flat;
            for (int d = 0; d < n; ++d) {
                if (d == d_star) {
                    m_safe_set_basis[m_safe_set_size * n + d] = tau;
                    continue;
                }
                m_safe_set_basis[m_safe_set_size * n + d] = (temp % (int)X_widthPerDimension[d]) + 1;
                temp /= (int)X_widthPerDimension[d];
            }
            m_safe_set_flat_indices[m_safe_set_size] = flattenIndex(&m_safe_set_basis[m_safe_set_size * n]);
            m_safe_set_size++;
        }
    }
}

void pfacesKernel_mono_synth::buildThresholdTableFromBitmap() {
    std::memset(m_threshold_table.data(), 0, m_threshold_table_size * sizeof(int));
    const int n = m_ss_dim;

    for (size_t flat = 0; flat < x_flat_width; ++flat) {
        if (!getPackedBit(m_bitmap_words, flat)) continue;

        size_t tmp = flat;
        int key_flat = 0;
        int dstar_height = 0;
        for (int d = 0; d < n; ++d) {
            const int coord0 = (int)(tmp % X_widthPerDimension[d]);
            tmp /= X_widthPerDimension[d];
            if (d == m_threshold_d_star) {
                dstar_height = coord0 + 1;
            } else {
                key_flat += coord0 * m_threshold_orig_to_key_stride[d];
            }
        }
        if (m_threshold_table[key_flat] < dstar_height)
            m_threshold_table[key_flat] = dstar_height;
    }
}

// Data pool indices for bitmap GFP buffers. These follow funcs 0..3:
// precompute (0,1), basis check (2..5), TT-only (6..9), bitmap (10..13).
#define BITMAP_GFP_POOL_IN      10
#define BITMAP_GFP_POOL_OUT     11
#define BITMAP_GFP_POOL_CHANGED 12
#define BITMAP_GFP_POOL_SWEEP   13

size_t pfacesKernel_mono_synth::initBitmapGFP(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;

    pKernel->m_ss_dim = pKernel->m_spCfg->getSsDim();
    pKernel->m_safe_set_size = 0;
    pKernel->m_iterations = 0;
    pKernel->m_bitmap_gfp_iteration = 0;
    pKernel->m_bitmap_safe_cells = 0;
    pKernel->m_compute_start = std::chrono::high_resolution_clock::now();

    pKernel->m_bitmap_words.assign(pKernel->m_bitmap_word_count, 0xFFFFFFFFu);
    const size_t tail_bits = pKernel->x_flat_width & 31u;
    if (tail_bits != 0 && !pKernel->m_bitmap_words.empty()) {
        pKernel->m_bitmap_words.back() = (1u << tail_bits) - 1u;
    }

    cl_uint* pIn = (cl_uint*)pParallelProgram->m_dataPool[BITMAP_GFP_POOL_IN].first;
    cl_uint* pOut = (cl_uint*)pParallelProgram->m_dataPool[BITMAP_GFP_POOL_OUT].first;
    int* pChanged = (int*)pParallelProgram->m_dataPool[BITMAP_GFP_POOL_CHANGED].first;
    std::memcpy(pIn, pKernel->m_bitmap_words.data(), pKernel->m_bitmap_word_count * sizeof(cl_uint));
    std::memset(pOut, 0, pKernel->m_bitmap_word_count * sizeof(cl_uint));
    *pChanged = 0;

    std::cout << "[MonoSynth] Bitmap GFP: initialized all " << pKernel->x_flat_width
              << " grid cells as safe" << std::endl;
    return 0;
}

size_t pfacesKernel_mono_synth::prepareBitmapGFPIteration(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;

    pKernel->m_bitmap_gfp_iteration++;
    pKernel->m_iteration_start = std::chrono::high_resolution_clock::now();
    pKernel->m_bitmap_gfp_sweep_dim_idx = 0;
    int* pChanged = (int*)pParallelProgram->m_dataPool[BITMAP_GFP_POOL_CHANGED].first;
    *pChanged = 0;
    return 0;
}

size_t pfacesKernel_mono_synth::setBitmapSweepParams(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;

    const int d = pKernel->m_bitmap_gfp_sweep_dim_idx;
    int stride = 1;
    for (int k = 0; k < d; ++k) stride *= (int)pKernel->X_widthPerDimension[k];

    int* pSweep = (int*)pParallelProgram->m_dataPool[BITMAP_GFP_POOL_SWEEP].first;
    pSweep[0] = stride;
    pSweep[1] = (int)pKernel->X_widthPerDimension[d];

    pKernel->m_bitmap_gfp_sweep_dim_idx++;
    return 0;
}

size_t pfacesKernel_mono_synth::processBitmapGFPUpdate(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;

    auto iter_end = std::chrono::high_resolution_clock::now();
    double iter_ms = std::chrono::duration<double, std::milli>(iter_end - pKernel->m_iteration_start).count();
    const int changed = *((int*)pParallelProgram->m_dataPool[BITMAP_GFP_POOL_CHANGED].first);

    if (pKernel->m_iteration_stats_csv_file.is_open()) {
        pKernel->m_iteration_stats_csv_file
            << (pKernel->m_benchmark_current_run + 1) << ","
            << pKernel->m_bitmap_gfp_iteration << ","
            << pKernel->x_flat_width << ","
            << 0 << "," << 0 << "," << 0 << ","
            << pKernel->x_flat_width << ","
            << iter_ms << ","
            << iter_ms << ","
            << 0 << "," << 0 << "," << 0 << "," << 0 << ","
            << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "\n";
        pKernel->m_iteration_stats_csv_file.flush();
    }

    if (changed == 0) {
        pKernel->m_iterations = pKernel->m_bitmap_gfp_iteration;
        auto compute_end = std::chrono::high_resolution_clock::now();
        pKernel->m_bitmap_gfp_last_ms = std::chrono::duration<double, std::milli>(compute_end - pKernel->m_compute_start).count();
        pKernel->m_benchmark_total_time_ms += pKernel->m_bitmap_gfp_last_ms;
        pKernel->m_benchmark_total_iterations += pKernel->m_bitmap_gfp_iteration;
        pKernel->m_benchmark_current_run++;
        return 0;
    }

    return 1;
}

size_t pfacesKernel_mono_synth::finalizeBitmapGFP(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;

    const cl_uint* pIn = (const cl_uint*)pParallelProgram->m_dataPool[BITMAP_GFP_POOL_IN].first;
    pKernel->m_bitmap_words.assign(pIn, pIn + pKernel->m_bitmap_word_count);

    pKernel->m_bitmap_safe_cells = 0;
    for (size_t i = 0; i < pKernel->m_bitmap_word_count; ++i) {
        pKernel->m_bitmap_safe_cells += __builtin_popcount(pKernel->m_bitmap_words[i]);
    }

    if (pKernel->m_record_basis_evolution) {
        pKernel->buildThresholdTableFromBitmap();
        if (pKernel->m_benchmark_current_run == 1) {
            std::ofstream out("bitmap_threshold_final.csv");
            out << "key_flat,tau\n";
            for (int i = 0; i < pKernel->m_threshold_table_size; ++i) {
                if (pKernel->m_threshold_table[i] > 0)
                    out << i << "," << pKernel->m_threshold_table[i] << "\n";
            }
        }
    }

    std::cout << "[MonoSynth] Safe cells: " << pKernel->m_bitmap_safe_cells << "/"
              << pKernel->x_flat_width << " ("
              << (100.0 * pKernel->m_bitmap_safe_cells / pKernel->x_flat_width)
              << "%)" << std::endl;
    std::cout << "Run " << pKernel->m_benchmark_current_run << "/"
              << pKernel->m_benchmark_count << ": "
              << pKernel->m_bitmap_gfp_iteration << " iterations (bitmap GFP), "
              << pKernel->m_bitmap_safe_cells << " safe cells, "
              << (int)pKernel->m_bitmap_gfp_last_ms << " ms" << std::endl;
    return 0;
}

/* ================================================================
 * GPU TT-only host functions
 *
 * Data pool indices (when TT-only GPU is active):
 *   Pool[6] = threshold_table_in  (func2.arg1)
 *   Pool[7] = threshold_table_out (func2.arg2 / func3.arg0 resident)
 *   Pool[8] = changed_flag        (func2.arg3)
 *   Pool[9] = sweep_params        (func3.arg1)
 * ================================================================ */
#define TT_GPU_POOL_TT_IN  6
#define TT_GPU_POOL_TT_OUT 7
#define TT_GPU_POOL_CHANGED 8
#define TT_GPU_POOL_SWEEP  9

/* initTTGPU: Fill threshold table with N_d*, set up basis pointers, init sweep_params */
size_t pfacesKernel_mono_synth::initTTGPU(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;

    pKernel->m_ss_dim = pKernel->m_spCfg->getSsDim();
    pKernel->m_safe_set_flat_indices = (int*)pParallelProgram->m_dataPool[2].first;
    pKernel->m_safe_set_basis = (int*)pParallelProgram->m_dataPool[3].first;
    pKernel->m_safe_set_size = 0;
    pKernel->m_tt_gpu_iteration = 0;
    pKernel->m_compute_start = std::chrono::high_resolution_clock::now();

    // Fill threshold table with N_d* (all columns start at maximum height)
    const int N_dstar = (int)pKernel->X_widthPerDimension[pKernel->m_threshold_d_star];
    std::fill(pKernel->m_threshold_table.begin(), pKernel->m_threshold_table.end(), N_dstar);

    // Pre-fill sweep_params buffer for each key dimension
    // The sweep_params buffer holds [stride_k0, N_k0, stride_k1, N_k1, ...] in the Pool[10]
    // But since each prefix_max launch writes the WHOLE sweep_params buffer, we need to
    // set the correct values in prepareTTGPUIteration before each sweep launch.
    // Actually, the write instruction transfers the entire Pool[10] buffer each time.
    // So we just set the right values before each dimension's write in the prep function.

    std::cout << "[MonoSynth] GPU TT-only: initialized " << pKernel->m_threshold_table_size
              << " columns to height " << N_dstar << std::endl;

    return 0;
}

/* prepareTTGPUIteration: Copy current threshold table to pool TT_in, zero changed_flag.
   Called once per iteration, before the GPU kernels execute. */
size_t pfacesKernel_mono_synth::prepareTTGPUIteration(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;

    pKernel->m_tt_gpu_iteration++;
    pKernel->m_iteration_start = std::chrono::high_resolution_clock::now();
    pKernel->m_tt_gpu_sweep_dim_idx = 0;  // reset sweep dimension counter

    const int table_size = pKernel->m_threshold_table_size;

    // Copy current threshold table → pool TT_in buffer
    int* pTTIn = (int*)pParallelProgram->m_dataPool[TT_GPU_POOL_TT_IN].first;
    std::memcpy(pTTIn, pKernel->m_threshold_table.data(), table_size * sizeof(int));

    // Zero changed_flag
    int* pChanged = (int*)pParallelProgram->m_dataPool[TT_GPU_POOL_CHANGED].first;
    *pChanged = 0;

    return 0;
}

/* setSweepParams: Set sweep_params buffer for the next key dimension.
   Called once per key dimension per iteration, before each prefix_max launch. */
size_t pfacesKernel_mono_synth::setSweepParams(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;

    const int ki = pKernel->m_tt_gpu_sweep_dim_idx;
    int* pSweep = (int*)pParallelProgram->m_dataPool[TT_GPU_POOL_SWEEP].first;
    pSweep[0] = pKernel->m_threshold_orig_to_key_stride[pKernel->m_threshold_key_dim_indices[ki]];
    pSweep[1] = pKernel->m_threshold_key_dims[ki];

    pKernel->m_tt_gpu_sweep_dim_idx++;
    return 0;
}

/* processTTGPUUpdate: Read TT_out from pool → m_threshold_table, check convergence.
   Returns 0 to stop (converged), 1 to continue. */
size_t pfacesKernel_mono_synth::processTTGPUUpdate(void* pPackedKernel, void* pPackedParallelProgram) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    pfacesParallelProgram* pParallelProgram = (pfacesParallelProgram*)pPackedParallelProgram;

    auto iter_end = std::chrono::high_resolution_clock::now();
    double iter_ms = std::chrono::duration<double, std::milli>(iter_end - pKernel->m_iteration_start).count();

    const int table_size = pKernel->m_threshold_table_size;

    // Read TT_out from pool → m_threshold_table
    const int* pTTOut = (const int*)pParallelProgram->m_dataPool[TT_GPU_POOL_TT_OUT].first;
    std::memcpy(pKernel->m_threshold_table.data(), pTTOut, table_size * sizeof(int));

    // Record threshold table evolution to CSV
    pKernel->writeThresholdCSV(pKernel->m_tt_gpu_iteration);

    // Check convergence: compare final swept result against pre-iteration table.
    // NOTE: We do NOT rely solely on changed_flag from column_update because
    // the prefix_max sweep may restore values that column_update decreased,
    // leading to changed_flag=1 but no actual change in the final table.
    bool changed = false;
    {
        const int* pTTIn = (const int*)pParallelProgram->m_dataPool[TT_GPU_POOL_TT_IN].first;
        for (int i = 0; i < table_size; ++i) {
            if (pKernel->m_threshold_table[i] != pTTIn[i]) {
                changed = true;
                break;
            }
        }
    }

    if (pKernel->m_iteration_stats_csv_file.is_open()) {
        pKernel->m_iteration_stats_csv_file
            << (pKernel->m_benchmark_current_run + 1) << ","
            << pKernel->m_tt_gpu_iteration << ","
            << table_size << ","
            << 0 << "," << 0 << "," << 0 << ","
            << table_size << ","
            << iter_ms << ","
            << 0 << "," << 0 << "," << 0 << "," << 0 << ","
            << iter_ms << ","
            << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "\n";
        pKernel->m_iteration_stats_csv_file.flush();
    }

    if (!changed) {
        // Converged. RT/direct table queries do not need a basis.
        pKernel->m_iterations = pKernel->m_tt_gpu_iteration;
        if (pKernel->m_extract_basis) {
            pKernel->extractBasisFromThresholdTable();
        } else {
            pKernel->m_safe_set_size = 0;
        }

        auto compute_end = std::chrono::high_resolution_clock::now();
        double time_ms = std::chrono::duration<double, std::milli>(compute_end - pKernel->m_compute_start).count();

        pKernel->m_benchmark_total_time_ms += time_ms;
        pKernel->m_benchmark_total_iterations += pKernel->m_tt_gpu_iteration;
        pKernel->m_benchmark_current_run++;

        // Report safe cells (replaces former GPU Bitmap output)
        int64_t safe_count = 0;
        for (int i = 0; i < pKernel->m_threshold_table_size; ++i) safe_count += pKernel->m_threshold_table[i];
        std::cout << "[MonoSynth] Safe cells: " << safe_count << "/" << pKernel->x_flat_width
                  << " (" << (100.0 * safe_count / pKernel->x_flat_width) << "%)" << std::endl;

        std::cout << "Run " << pKernel->m_benchmark_current_run << "/" << pKernel->m_benchmark_count
                  << ": " << pKernel->m_tt_gpu_iteration << " iterations (GPU TT-only), "
                  << pKernel->m_safe_set_size
                  << (pKernel->m_extract_basis ? " basis, " : " basis (skipped), ")
                  << (int)time_ms << " ms" << std::endl;
        return 0;  // Stop
    }

    return 1;  // Continue
}

/* checkSkipPrecompute: Returns 1 to skip precompute, 0 to execute it.
   Also records precompute start time for phase timing. */
size_t pfacesKernel_mono_synth::checkSkipPrecompute(void* pPackedKernel, void* /*pPackedParallelProgram*/) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    return pKernel->m_skip_precompute ? 1 : 0;
}

/* timerAfterPrecompute: Record precompute phase time */
size_t pfacesKernel_mono_synth::timerAfterPrecompute(void* pPackedKernel, void* /*pPackedParallelProgram*/) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    auto now = std::chrono::high_resolution_clock::now();
    pKernel->m_precompute_ms = std::chrono::duration<double, std::milli>(now - pKernel->m_phase_timer).count();
    std::cout << "[MonoSynth] Precompute phase: " << (int)pKernel->m_precompute_ms << " ms" << std::endl;
    return 0;
}

/* timerAfterGFP: Record GFP phase time (called after GFP convergence) */
size_t pfacesKernel_mono_synth::timerAfterGFP(void* pPackedKernel, void* /*pPackedParallelProgram*/) {
    pfacesKernel_mono_synth* pKernel = (pfacesKernel_mono_synth*)pPackedKernel;
    auto now = std::chrono::high_resolution_clock::now();
    pKernel->m_gfp_total_ms = std::chrono::duration<double, std::milli>(now - pKernel->m_compute_start).count();
    return 0;
}

/* Provide a minimal single-function execution program for the pFaces tuner. */
void pfacesKernel_mono_synth::configureTuneParallelProgram(
    pfacesParallelProgram& tuneParallelProgram,
    size_t targetFunctionIdx) {
	pfacesParallelAdvisor parallelAdvisor(
		tuneParallelProgram.getMachine(),
		tuneParallelProgram.getTargetDevicesIndicies());

	tuneParallelProgram.m_isFixedJobDistribution = true;
	tuneParallelProgram.m_fixedJobDistribution = {1.0};
	tuneParallelProgram.m_beVerboseLevel = 0;

	cl::NDRange ndrOffset{0, 0, 0};
	cl::NDRange tuneRange{1, 1, 1};
	std::vector<std::shared_ptr<pfacesDeviceExecuteJob>> execJobs;

	switch (targetFunctionIdx) {
		case KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_IDX:
			tuneRange = cl::NDRange{x_flat_width, 1, 1};
			execJobs = parallelAdvisor.distributeJob(
				*this,
				KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_IDX,
				tuneRange,
				ndrOffset,
				true,
				{1.0},
				true,
				false,
				false);
			break;
		case KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_IDX:
			tuneRange = cl::NDRange{(size_t)MAX_BASIS_ELEMENTS, 1, 1};
			execJobs = parallelAdvisor.distributeJob(
				*this,
				KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_IDX,
				tuneRange,
				ndrOffset,
				true,
				{1.0},
				true,
				false,
				false);
			break;
		default:
			throw std::runtime_error("[MonoSynth] Unknown kernel function index for tuning.");
	}

	std::vector<std::pair<char*, size_t>> dataPool;
	allocateMemory(
		dataPool,
		tuneParallelProgram.getMachine(),
		tuneParallelProgram.getTargetDevicesIndicies(),
		1,
		true);

	if (execJobs.empty()) {
		throw std::runtime_error("[MonoSynth] Tuning could not create an execute job.");
	}

	// Populate a representative, non-empty basis for the kernels that consume it.
	if (targetFunctionIdx != KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_IDX) {
		if (dataPool.size() > 2 && dataPool[2].first) {
			int* pBasisFlatIdx = (int*)dataPool[2].first;
			*pBasisFlatIdx = 0;
		}
		if (dataPool.size() > 4 && dataPool[4].first) {
			int* pBasisListSize = (int*)dataPool[4].first;
			*pBasisListSize = 1;
		}
		if (dataPool.size() > 3 && dataPool[3].first) {
			int* pBasisList = (int*)dataPool[3].first;
			for (int d = 0; d < m_spCfg->getSsDim(); ++d) {
				pBasisList[d] = 0;
			}
		}
	}

	if (dataPool.size() > 1 && dataPool[1].first) {
		float* params = (float*)dataPool[1].first;
		params[0] = m_runtime_param0;
	}

	std::vector<std::shared_ptr<pfacesInstruction>> instructionList;
	auto add_write_instr = [&instructionList](const std::shared_ptr<pfacesDeviceWriteJob>& job) {
		auto instr = std::make_shared<pfacesInstruction>();
		instr->setAsWriteDeviceBuffer(job);
		instructionList.push_back(instr);
	};
	auto add_exec_job = [&instructionList](const std::shared_ptr<pfacesDeviceExecuteJob>& job) {
		auto instr = std::make_shared<pfacesInstruction>();
		instr->setAsDeviceExecute(job);
		instructionList.push_back(instr);
	};

	const cl::Device& dataAccessDevice = tuneParallelProgram.getTargetDevices()[0];
	switch (targetFunctionIdx) {
		case KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_IDX: {
			auto writeRuntimeParams = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 0, 2, 1);
			add_write_instr(writeRuntimeParams);
			break;
		}
		case KERNEL_MONO_SYNTH_CHECK_BASIS_SAFETY_FUNC_IDX: {
			auto writeBasisFlatIdx = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 1, 5, 0);
			auto writeNextStateTable = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 1, 5, 1);
			auto writeBasisList = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 1, 5, 2);
			auto writeBasisListSize = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice, 1, 5, 3);
			add_write_instr(writeBasisFlatIdx);
			add_write_instr(writeNextStateTable);
			add_write_instr(writeBasisList);
			add_write_instr(writeBasisListSize);
			break;
		}
	}

	add_exec_job(execJobs.front());

	tuneParallelProgram.m_Universal_globalNDRange = tuneRange;
	tuneParallelProgram.m_Process_globalNDRange = tuneRange;
	tuneParallelProgram.m_Universal_offsetNDRange = ndrOffset;
	tuneParallelProgram.m_Process_offsetNDRange = ndrOffset;
	tuneParallelProgram.m_dataPool = std::move(dataPool);
	tuneParallelProgram.m_spInstructionList = std::move(instructionList);
}

} // namespace mono_synth

PFACES_REGISTER_LOADABLE_KERNEL(mono_synth::pfacesKernel_mono_synth)
