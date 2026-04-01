#pragma once

#include <sstream>

#include <pfaces-sdk.h>

class defaultConfiguration {
public:

	// Constructor and destructor
	defaultConfiguration();
	~defaultConfiguration();

	// Gets the default configuration values
	static const char* getDefaults();

	// Gets the configuration's schema
	static void getSchema(const char**& schema, int& schemaSize);
	static const char** getSchema();

private:
	// Member variables
	std::string m_defaults;
	const char* m_schema[814];
	static defaultConfiguration s_singleton;
};

class configReader {
	std::shared_ptr<pfacesConfigurationReader> m_spConfigObject;

	// general
	std::string	m_validatemsg;
	std::string m_config_file_dir;

	bool m_isSynthesisPackSizeExactorPerc;
	size_t m_syntheisPackSize;

	std::string m_project_name;

	std::string m_data;
	bool            m_save_transitions;
	bool            m_save_controller;
	bool            m_record_basis_evolution;
	bool            m_boundary_seeding;
	bool            m_use_threshold_table;
	bool            m_use_tt_only;
	bool            m_use_tt_only_gpu;
	bool            m_use_inline_dynamics;
	size_t          m_benchmark_count;

	bool			m_hasTarget;
	size_t			m_targetCount;
	std::string		m_targetData;
	bool			m_hasSafe;
	size_t			m_safeCount;
	std::string		m_safeData;
	bool			m_hasAvoid;
	size_t			m_avoidCount;
	std::string		m_avoidData;

	// state/input spaces
	float m_sampling_period;
	size_t m_ode_steps;
	std::string m_user_dynamics_file;
	std::string m_state_priorities;
	std::vector<int> vSsPriorities;

	// state space strings
	size_t m_statedim;
	std::string m_stateeta;
	std::string m_statelb;
	std::string m_stateub;
	std::string m_stateerr;

	// state space vectors
	std::vector<concrete_t>	vSsEta;
	std::vector<concrete_t>	vSsLb;
	std::vector<concrete_t>	vSsUb;
	std::vector<concrete_t>	vSsErr;
	
	// disturbances
	size_t		m_disturbdim;

	// input space strings
	size_t m_inputdim;
	std::string m_inputeta;
	std::string m_inputlb;
	std::string m_inputub;
	std::string m_inputerr;

	// input space vectors
	std::vector<concrete_t> vIsEta;
	std::vector<concrete_t> vIsLb;
	std::vector<concrete_t> vIsUb;
	std::vector<concrete_t> vIsErr;
	
	// array limits
	size_t m_max_basis_elements;

	// extra include file
	std::string m_extraIncludeFile;

public:
	configReader(const std::shared_ptr<pfacesConfigurationReader>& spConfigObject);

	//--------
	// Acccessors for configuration variables.
	//--------
	inline const char* getValidationMessage() const { return m_validatemsg.c_str(); }
	inline const char* getProjectName() const { return m_project_name.c_str(); }

	inline const char* getDataImplementationtype()	const { return m_data.c_str(); }

	inline float getSamplingPeriod() const { return m_sampling_period; }
	inline size_t getOdeSteps() const { return m_ode_steps; }
	inline const char* getUserDynamicsFile() const { return m_user_dynamics_file.c_str(); }
	inline std::vector<int> getSsPriorities() const { return vSsPriorities; }

	inline size_t getSsDim() const { return m_statedim; }
	inline std::vector<concrete_t> getSsEta() const { return vSsEta; }
	inline std::vector<concrete_t> getSsLb()  const { return vSsLb; }
	inline std::vector<concrete_t> getSsUb()  const { return vSsUb; }
	inline std::vector<concrete_t> getSsErr() const { return vSsErr; }

	inline size_t getIsDim() const {
		return m_inputdim;
	}
	inline std::vector<concrete_t> getIsEta() const { return vIsEta; }
	inline std::vector<concrete_t> getIsLb()  const { return vIsLb; }
	inline std::vector<concrete_t> getIsUb()  const { return vIsUb; }
	inline std::vector<concrete_t> getIsErr() const { return vIsErr; }
	
	inline size_t getDisturbDim() const { return m_disturbdim; }
	
	inline size_t getMaxBasisElements() const { return m_max_basis_elements; }

	/* for sparse-aware kernels */
	std::vector<std::vector<bool>> perComponentAffectingX;
	std::vector<std::vector<bool>> perComponentAffectingU;
	std::vector<size_t>	getAffectingXComponents(size_t affected_x_component) const;
	std::vector<size_t>	getAffectingUComponents(size_t affected_x_component) const;

	inline size_t estimateSymbolicDiameter() const {
		concrete_t sum = 0, min_eta = getSsEta()[0];
		for (size_t i = 0; i < getSsDim(); i++) {
			sum += (getSsLb()[i] - getSsUb()[i]) * (getSsLb()[i] - getSsUb()[i]);

			if (getSsEta()[i] < min_eta)
				min_eta = getSsEta()[i];
		}
		return std::lround(std::sqrt(sum) / min_eta);
	}

	inline size_t diameterToSynthesiPackSize(size_t diameter) const {
		if (m_isSynthesisPackSizeExactorPerc)
			return m_syntheisPackSize;
		else
			return (size_t)std::ceil(((double)m_syntheisPackSize) / 100.0 * (double)diameter);
	}

	std::string	getExtraIncludeFile()const;

	inline bool isSaveTransitions() const { return m_save_transitions; }
	inline bool isSaveController() const { return m_save_controller; }
	inline bool isRecordBasisEvolution() const { return m_record_basis_evolution; }
	inline bool isBoundarySeeding() const { return m_boundary_seeding; }
	inline bool isUseThresholdTable() const { return m_use_threshold_table; }
	inline bool isUseTTOnly() const { return m_use_tt_only; }
	inline bool isUseTTOnlyGPU() const { return m_use_tt_only_gpu; }
	inline bool isUseInlineDynamics() const { return m_use_inline_dynamics; }
	inline size_t getBenchmarkCount() const { return m_benchmark_count; }

	inline bool isHasTarget() const { return m_hasTarget; }
	inline bool isHasSafe() const { return m_hasSafe; }
	inline bool isHasAvoid() const { return m_hasAvoid; }

	inline size_t getTargetCount() const { return m_targetCount; }
	inline std::string getTargetData() const { return m_targetData; }
	inline size_t getSafeCount() const { return m_safeCount; }
	inline std::string getSafeData() const { return m_safeData; }
	inline size_t getAvoidCount() const { return m_avoidCount; }
	inline std::string getAvoidData() const { return m_avoidData; }

	void load_values();
	int validate_values();
};
