/*
* pfacesKernel_mono_synth.cpp
*
*  created on: 02.10.2025
*      author: M. Khaled
*/

#include <ctime>

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
		std::cout << "Loading transitions from file: " << file_path << std::endl;
		int cached_total_states;
		cache_in.read(reinterpret_cast<char*>(&cached_total_states), sizeof(int));
		
		if (cached_total_states == number_of_states) {
			cache_in.read(reinterpret_cast<char*>(pDataTransitionTable), 
							sizeof(cl_int) * number_of_elements);
			cache_in.close();
		}
		else {
			std::cerr << "Cached transitions do not match the current problem size. Recomputing transitions." << std::endl;
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

	// Loading the memory fingerprint of the abstract function
	auto precomputeTransitionsFunctionArgs = pfacesKernelFunctionArguments::loadFromFile(
		spLaunchState->getKernelPackPath() + "mono_synth.mem",						/* memory config file with the function memory fingerprint */
		KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NAME,  						/* name of the function to add */
		{KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNCARG_NEXT_STATE_TABLE_NAME},	/* list of the names of its args */
		false																		/* do not save memory render files */
	);
	precomputeTransitionsFunctionArgs.m_baseTypeSize = {sizeof(cl_uint)};
	precomputeTransitionsFunctionArgs.m_baseTypeMultiple = {x_flat_width * ssDim};
	pfacesKernelFunction precomputeTransitionsFunction(KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NAME, precomputeTransitionsFunctionArgs);

	// adding the function to the kernel
	addKernelFunction(precomputeTransitionsFunction);
	
	// updating the list of params
	auto params_and_vals = getParameterList();
	updateParameters(params_and_vals.first, params_and_vals.second);	
}

/* providing implementation of the driver of the kernel */
void pfacesKernel_mono_synth::configureParallelProgram(pfacesParallelProgram& parallelProgram) {

	// A parallel advisor used for task scheduling
	pfacesParallelAdvisor parallelAdvisor(parallelProgram.getMachine(), parallelProgram.getTargetDevicesIndicies());
	size_t beVerboseLevel = parallelProgram.m_beVerboseLevel;

	// X space cardinality
	size_t problem_x_width = x_flat_width;

	// Print Universal XU-space information
	if (beVerboseLevel >= 2) {
		std::cout << "Universal X-space has " << problem_x_width << " symbols: ";
		pfacesUtils::PrintVector(X_widthPerDimension, 'x');
	}

	cl::NDRange ndRangeRunPrecomputeTrans{problem_x_width, 1, 1};
	cl::NDRange ndRangeOffsetPrecomputeTrans{0, 0, 0};
	job_execPrecomputeTransition = parallelAdvisor.distributeJob(
		*this, KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_IDX, ndRangeRunPrecomputeTrans, ndRangeOffsetPrecomputeTrans,
		parallelProgram.m_isFixedJobDistribution, 
		parallelProgram.m_fixedJobDistribution, 
		///TODO: optimize this later
		true, false, false);

	// print the task-scheduling report
	if (beVerboseLevel >= 2){
		parallelAdvisor.printTaskSchedulingReport(
			parallelProgram.getMachine(),
			{ KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NAME },
			{ job_execPrecomputeTransition },
			problem_x_width
		);
	}

	// Allocating the memory used for abstraction/synthesis
	std::vector<std::pair<char*, size_t>> dataPool;
	///TODO: double check this
	const int numInstances = 1;
	pFacesMemoryAllocationReport memReport = allocateMemory(dataPool, parallelProgram.getMachine(), parallelProgram.getTargetDevicesIndicies(), numInstances, false);
	if (beVerboseLevel >= 2) {
		memReport.PrintReport();
	}	

	// First device in the list will be used for memory access
	const cl::Device&  dataAccessDevice = parallelProgram.getTargetDevices()[0];

	// Initialize jobs/instructions for data read/write of the transition table
	job_readNextStateTable = std::make_shared<pfacesDeviceReadJob>(dataAccessDevice);
	job_readNextStateTable->setKernelFunctionIdx(KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_IDX, KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NUM_ARGS);
	job_readNextStateTable->setKernelFunctionArgIdx(KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNCARG_NEXT_STATE_TABLE_IDX);

	job_writeNextStateTable = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice);
	job_writeNextStateTable->setKernelFunctionIdx(KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_IDX, KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNC_NUM_ARGS);
	job_writeNextStateTable->setKernelFunctionArgIdx(KERNEL_MONO_SYNTH_PRECOMPUTE_TRANSITIONS_FUNCARG_NEXT_STATE_TABLE_IDX);

	instr_readNextStateTable->setAsReadDeviceBuffer(job_readNextStateTable);
	instr_writeNextStateTable->setAsWriteDeviceBuffer(job_writeNextStateTable);

	// Configure other instructions
	instr_BlockingSyncPoint->setAsBlockingSyncPoint();

	// if using the direct access to host memory, we add this instruction
	if (parallelProgram.m_useHostMemory) {
		instructionList.push_back(instr_writeNextStateTable);
	}
	if (parallelProgram.countTargetDevices() > 1) {
		instructionList.push_back(instr_BlockingSyncPoint);
	}
	// Check if the transition table is already loaded
	if(std::ifstream(cache_file).good()) {
		// Read the transition table
		instructionList.push_back(instr_readNextStateTable);

		// Sync to make sure data is read
		instructionList.push_back(instr_BlockingSyncPoint);

		// Load the transition table
		instr_hostFuncLoadNextStateTable->setAsHostFunction(pfacesKernel_mono_synth::loadTransitionTable, "loadTransitionTable");
		instructionList.push_back(instr_hostFuncLoadNextStateTable);
	} else {

		// The first task: PrecomputeTransitions
		for (size_t i = 0; i < job_execPrecomputeTransition.size(); i++) {
			std::shared_ptr<pfacesInstruction> tmpExecuteInstr = std::make_shared<pfacesInstruction>();
			tmpExecuteInstr->setAsDeviceExecute(job_execPrecomputeTransition[i]);
			instructionList.push_back(tmpExecuteInstr);
		}

		// A Barrier to force all devices to finish.
		if (parallelProgram.countTargetDevices() > 1) {
			instructionList.push_back(instr_BlockingSyncPoint);
		}

		// Read the transition table
		instructionList.push_back(instr_readNextStateTable);

		// Sync to make sure data is read
		instructionList.push_back(instr_BlockingSyncPoint);

		// Call host function to save transitions
		instr_hostFuncSaveTransitions->setAsHostFunction(pfacesKernel_mono_synth::saveTransitionTable, "saveTransitionTable");
		instructionList.push_back(instr_hostFuncSaveTransitions);
	}

	// Last instruction
	instructionList.push_back(instr_BlockingSyncPoint);
	

	// setting the execute ranges
	parallelProgram.m_Universal_globalNDRange = ndRangeRunPrecomputeTrans;
	parallelProgram.m_Universal_offsetNDRange = ndRangeOffsetPrecomputeTrans;
	parallelProgram.m_Process_globalNDRange = ndRangeRunPrecomputeTrans;
	parallelProgram.m_Process_offsetNDRange = ndRangeOffsetPrecomputeTrans;

	parallelProgram.m_dataPool = dataPool;
	parallelProgram.m_spInstructionList = instructionList;

	///TODO: Move this as host function when you add the other kernel function
	std::vector<std::shared_ptr<void>> postExecuteParams;
}

/* not providing implementation of the virtual method: configureTuneParallelProgram*/
void pfacesKernel_mono_synth::configureTuneParallelProgram(pfacesParallelProgram&, size_t) {
}

} // namespace mono_synth

PFACES_REGISTER_LOADABLE_KERNEL(mono_synth::pfacesKernel_mono_synth)



