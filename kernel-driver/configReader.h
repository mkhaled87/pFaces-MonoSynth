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
	const char* m_schema[802];
	static defaultConfiguration s_singleton;
};

class configReader {
	std::shared_ptr<pfacesConfigurationReader> m_spConfigObject;

	// general
	std::string	m_validatemsg;

	bool m_isSynthesisPackSizeExactorPerc;
	size_t m_syntheisPackSize;

	std::string m_project_name;

	std::string m_data;
	bool            m_save_transitions;
	bool            m_save_controller;

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
	float 		m_samplingperiod;

	size_t 		m_statedim;
	std::string m_stateeta;
	std::string m_statelb;
	std::string m_stateub;
	std::string m_stateerr;

	std::vector<concrete_t>		vSsEta;
	std::vector<concrete_t>		vSsLb;
	std::vector<concrete_t>		vSsUb;
	std::vector<concrete_t>		vSsErr;


	size_t 			m_inputdim;
	std::string m_inputeta;
	std::string m_inputlb;
	std::string m_inputub;
	std::string m_inputerr;

	std::vector<concrete_t>		vIsEta;
	std::vector<concrete_t>		vIsLb;
	std::vector<concrete_t>		vIsUb;
	std::vector<concrete_t>		vIsErr;

	// post / growth
	bool m_postisode;
	bool m_postCodeonly;
	bool m_growthisode;
	bool m_growthCodeonly;
	size_t m_maxposts;

	std::string m_extraIncludeFile;

	std::vector<std::string> m_postDynamics_initCodes;
	std::vector<std::string> m_postDynamics_finishCodes;
	std::vector<std::string> m_growthDynamics_initCodes;
	std::vector<std::string> m_growthDynamics_finishCodes;

	std::vector<std::string> postDynamics_raw;
	std::vector<std::string> postDynamics;

	std::vector<std::string> growthDynamics_raw;
	std::vector<std::string> growthDynamics;

	std::vector<std::string> missingPostDynamics;
	std::vector<std::string> missingGrowthDynamics;
public:
	configReader(const std::shared_ptr<pfacesConfigurationReader>& spConfigObject);

	//--------
	// Acccessors for configuration variables.
	//--------
	inline const char* getValidationMessage() const { return m_validatemsg.c_str(); }
	inline const char* getProjectName() const { return m_project_name.c_str(); }

	inline const char* getDataImplementationtype()	const { return m_data.c_str(); }

	inline float getSamplingPeriod() const { return m_samplingperiod; }

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

	inline bool getUseOdePost() const { return m_postisode; }
	inline bool getCodeOnlyPost() const { return m_postCodeonly; }
	inline bool getUseOdeRadius() const { return m_growthisode; }
	inline bool getCodeOnlyRadius() const { return m_growthCodeonly; }

	std::string	getExtraIncludeFile()const;

	std::string	getPostDynamicsInitCodeOpenCL()const;
	std::string	getPostDynamicsFinishCodeOpenCL()const;
	std::string	getGrowthDynamicsInitCodeOpenCL()const;
	std::string	getGrowthDynamicsFinishCodeOpenCL()const;

	std::string	getPostDynamicsElement(size_t elementIndex, bool isRawVersion = false)const;
	std::string getPostDynamicsOpenCL()const;
	std::string getGrowthDynamicsOpenCL()const;

	size_t getMaxPosts() const;

	inline bool isSaveTransitions() const { return m_save_transitions; }
	inline bool isSaveController() const { return m_save_controller; }

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
