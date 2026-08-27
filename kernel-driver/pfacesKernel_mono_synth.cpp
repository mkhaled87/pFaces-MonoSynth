#include "pfacesKernel_mono_synth.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace mono_synth {
namespace {

constexpr std::size_t kMembershipWorkgroup = 64;

void addSync(std::vector<std::shared_ptr<pfacesInstruction>>& list) {
  auto instruction = std::make_shared<pfacesInstruction>();
  instruction->setAsBlockingSyncPoint();
  list.push_back(instruction);
}

void addHost(std::vector<std::shared_ptr<pfacesInstruction>>& list,
             size_t (*function)(void*, void*), const char* name) {
  auto instruction = std::make_shared<pfacesInstruction>();
  instruction->setAsHostFunction(function, name);
  list.push_back(instruction);
}

void addExecute(
    std::vector<std::shared_ptr<pfacesInstruction>>& list,
    const std::vector<std::shared_ptr<pfacesDeviceExecuteJob>>& jobs) {
  for (const auto& job : jobs) {
    auto instruction = std::make_shared<pfacesInstruction>();
    instruction->setAsDeviceExecute(job);
    list.push_back(instruction);
  }
}

void addWrite(std::vector<std::shared_ptr<pfacesInstruction>>& list,
              const std::shared_ptr<pfacesDeviceWriteJob>& job) {
  auto instruction = std::make_shared<pfacesInstruction>();
  instruction->setAsWriteDeviceBuffer(job);
  list.push_back(instruction);
}

void addRead(std::vector<std::shared_ptr<pfacesInstruction>>& list,
             const std::shared_ptr<pfacesDeviceReadJob>& job) {
  auto instruction = std::make_shared<pfacesInstruction>();
  instruction->setAsReadDeviceBuffer(job);
  list.push_back(instruction);
}

std::uint64_t fnvMix(std::uint64_t hash, std::uint64_t value) {
  for (unsigned byte = 0; byte < 8; ++byte) {
    hash ^= static_cast<unsigned char>(value >> (8 * byte));
    hash *= UINT64_C(1099511628211);
  }
  return hash;
}

struct CacheHeader {
  char magic[8];
  std::uint32_t version;
  std::uint32_t dimensions;
  std::uint64_t states;
  std::uint64_t fingerprint;
};

}  // namespace

pfacesKernel_mono_synth::pfacesKernel_mono_synth(
    const std::shared_ptr<pfacesKernelLaunchState>& launch_state,
    const std::shared_ptr<pfacesConfigurationReader>& configuration)
    : pfaces2DKernel(
          launch_state->getDefaultSourceFilePath(KERNEL_NAME_MONO_SYNTH)),
      config_(std::make_shared<configReader>(configuration)),
      method_(config_->getSynthesisMethod()),
      backend_(config_->getTransitionBackend()),
      state_dimension_(config_->getSsDim()),
      max_basis_elements_(config_->getMaxBasisElements()),
      current_basis_(max_basis_elements_),
      target_basis_(max_basis_elements_),
      work_basis_(max_basis_elements_),
      controlled_basis_(max_basis_elements_),
      frontier_basis_(max_basis_elements_) {
  if (state_dimension_ == 0) throw std::invalid_argument("state dimension is zero");
  if (backend_ == TransitionBackend::INLINE &&
      method_ != SynthesisMethod::THRESHOLD &&
      method_ != SynthesisMethod::BITMAP_REFERENCE) {
    throw std::invalid_argument(
        "inline transitions are implemented only for threshold and bitmap_reference");
  }

  std::vector<cl_ulong> pfaces_widths;
  const auto flat_width = pfacesFlatSpace::getFlatWidthFromConcreteSpace(
      config_->getSsDim(), config_->getSsEta(), config_->getSsLb(),
      config_->getSsUb(), config_->getSsErr(), pfaces_widths);
  grid_widths_cl_ = pfaces_widths;
  grid_widths_.reserve(pfaces_widths.size());
  for (cl_ulong width : pfaces_widths) {
    if (width > static_cast<cl_ulong>(std::numeric_limits<cl_int>::max())) {
      throw std::overflow_error(
          "grid width exceeds the signed 32-bit OpenCL coordinate range");
    }
    grid_widths_.push_back(width);
  }
  total_states_ = BasisStore::checked_grid_size(grid_widths_);
  if (flat_width.toUnsignedLong() != total_states_) {
    throw std::runtime_error("pFaces and checked grid cardinalities disagree");
  }
  if (total_states_ > std::numeric_limits<std::size_t>::max()) {
    throw std::overflow_error("grid does not fit host/OpenCL NDRange size_t");
  }

  initializeThresholdGeometry();
  extract_basis_ = config_->isExtractBasis();
  stats_.method = method_;

  cache_fingerprint_ = UINT64_C(1469598103934665603);
  cache_fingerprint_ = fnvMix(cache_fingerprint_, total_states_);
  cache_fingerprint_ = fnvMix(cache_fingerprint_, state_dimension_);
  cache_fingerprint_ = fnvMix(cache_fingerprint_, config_->getIsDim());
  cache_fingerprint_ = fnvMix(cache_fingerprint_, config_->getDisturbDim());
  for (GridIndex width : grid_widths_) {
    cache_fingerprint_ = fnvMix(cache_fingerprint_, width);
  }
  std::ostringstream fingerprint_text;
  fingerprint_text << std::setprecision(17)
                   << "stage_projected_rk4_v1:"
                   << config_->getSamplingPeriod() << ':'
                   << config_->getOdeSteps() << ':'
                   << config_->getTransitionSemantics() << ':';
  fingerprint_text << boundarySemanticsName(config_->getBoundarySemantics())
                   << ':';
  for (auto value : config_->getSsLb()) fingerprint_text << value << ',';
  for (auto value : config_->getSsUb()) fingerprint_text << value << ',';
  for (auto value : config_->getSsEta()) fingerprint_text << value << ',';
  for (int value : config_->getSsPriorities()) fingerprint_text << value << ',';
  const std::string dynamics_path = config_->getUserDynamicsFile();
  fingerprint_text << dynamics_path << ':';
  if (!dynamics_path.empty()) {
    fingerprint_text << pfacesFileIO::readTextFromFile(dynamics_path);
  }
  for (unsigned char ch : fingerprint_text.str()) {
    cache_fingerprint_ = fnvMix(cache_fingerprint_, ch);
  }
  cache_file_ = std::string(config_->getProjectName()) +
                ".transitions.u64.v2.bin";

  std::string pack_path = launch_state->getKernelPackPath();
  if (!pack_path.empty() && pack_path.back() != '/' && pack_path.back() != '\\') {
    pack_path += '/';
  }
  registerKernelFunctions(pack_path);
  const auto parameters = getParameterList();
  updateParameters(parameters.first, parameters.second);

  std::cout << "[MonoSynth] method=" << synthesisMethodName(method_)
            << ", transition_backend=" << transitionBackendName(backend_)
            << ", boundary_semantics="
            << boundarySemanticsName(config_->getBoundarySemantics())
            << ", states=" << total_states_ << ", d*=" << threshold_d_star_
            << ", threshold_columns=" << threshold_table_size_ << std::endl;
}

pfacesKernel_mono_synth::~pfacesKernel_mono_synth() = default;

void pfacesKernel_mono_synth::initializeThresholdGeometry() {
  const int requested = config_->getThresholdDStarOverride();
  if (requested >= 0) {
    threshold_d_star_ = static_cast<std::size_t>(requested);
  } else {
    threshold_d_star_ = 0;
    for (std::size_t d = 1; d < grid_widths_.size(); ++d) {
      if (grid_widths_[d] > grid_widths_[threshold_d_star_]) threshold_d_star_ = d;
    }
  }

  threshold_table_size_ = 1;
  threshold_key_strides_.assign(state_dimension_, 0);
  TableOffset stride = 1;
  for (std::size_t d = 0; d < state_dimension_; ++d) {
    if (d == threshold_d_star_) continue;
    threshold_key_dimensions_.push_back(d);
    threshold_key_strides_[d] = stride;
    stride = checkedMultiply(stride, grid_widths_[d], "threshold-table size");
  }
  threshold_table_size_ = stride;
  if (threshold_table_size_ > std::numeric_limits<std::size_t>::max()) {
    throw std::overflow_error("threshold table does not fit host size_t");
  }
  threshold_result_u32_.assign(static_cast<std::size_t>(threshold_table_size_), 0);
}

std::pair<std::vector<std::string>, std::vector<std::string>>
pfacesKernel_mono_synth::getParameterList() {
  std::vector<std::string> names;
  std::vector<std::string> values;
  const auto add = [&](const std::string& name, const std::string& value) {
    names.push_back(name);
    values.push_back(value);
  };

  std::string dynamics_code;
  const std::string dynamics_path = config_->getUserDynamicsFile();
  if (!dynamics_path.empty()) dynamics_code = pfacesFileIO::readTextFromFile(dynamics_path);
  add("@@USER_DYNAMICS_CODE@@", dynamics_code);
  add("@@STATE_DIM@@", std::to_string(state_dimension_));
  add("@@INPUT_DIM@@", std::to_string(config_->getIsDim()));
  add("@@DISTURB_DIM@@", std::to_string(config_->getDisturbDim()));
  add("@@TOTAL_STATES@@", std::to_string(total_states_));
  add("@@BITMAP_WORD_COUNT@@", std::to_string((total_states_ + 31) / 32));
  add("@@ODE_STEPS@@", std::to_string(config_->getOdeSteps()));
  add("@@SAMPLING_TIME@@", std::to_string(config_->getSamplingPeriod()) + "f");
  add("@@THRESHOLD_D_STAR@@", std::to_string(threshold_d_star_));
  add("@@THRESHOLD_TABLE_SIZE@@", std::to_string(threshold_table_size_));
  add("@@USE_INLINE_DYNAMICS@@", backend_ == TransitionBackend::INLINE ? "1" : "0");
  add("@@SATURATE_FAVORABLE_EXITS@@",
      config_->getBoundarySemantics() ==
              BoundarySemantics::FAVORABLE_SATURATING
          ? "1" : "0");

  const auto lower = config_->getSsLb();
  const auto upper = config_->getSsUb();
  const auto resolution = config_->getSsEta();
  const auto priorities = config_->getSsPriorities();
  std::ostringstream lower_text, upper_text, resolution_text, priorities_text,
      widths_text;
  lower_text << std::fixed << std::setprecision(9) << '{';
  upper_text << std::fixed << std::setprecision(9) << '{';
  resolution_text << std::fixed << std::setprecision(9) << '{';
  priorities_text << '{';
  widths_text << '{';
  for (std::size_t d = 0; d < state_dimension_; ++d) {
    if (d) {
      lower_text << ',';
      upper_text << ',';
      resolution_text << ',';
      priorities_text << ',';
      widths_text << ',';
    }
    lower_text << lower[d] << 'f';
    upper_text << upper[d] << 'f';
    resolution_text << resolution[d] << 'f';
    priorities_text << priorities[d];
    widths_text << grid_widths_[d];
  }
  lower_text << '}';
  upper_text << '}';
  resolution_text << '}';
  priorities_text << '}';
  widths_text << '}';
  add("@@X_MIN_ARRAY@@", lower_text.str());
  add("@@X_MAX_ARRAY@@", upper_text.str());
  add("@@X_RES_ARRAY@@", resolution_text.str());
  add("@@X_PRIORITY_ARRAY@@", priorities_text.str());
  add("@@GRID_SIZES_ARRAY@@", widths_text.str());
  return {names, values};
}

void pfacesKernel_mono_synth::registerKernelFunctions(const std::string& path) {
  const std::size_t transition_entries =
      backend_ == TransitionBackend::INLINE ? 1 : static_cast<std::size_t>(total_states_);
  const std::size_t basis_coordinates =
      checkedMultiply(max_basis_elements_, state_dimension_, "basis allocation");
  const std::size_t table_entries = static_cast<std::size_t>(threshold_table_size_);
  const std::size_t bitmap_words = static_cast<std::size_t>((total_states_ + 31) / 32);
  const bool uses_scan = method_ == SynthesisMethod::CDC ||
                         method_ == SynthesisMethod::AUTOMATICA_SCAN;
  const bool uses_automatica_threshold =
      method_ == SynthesisMethod::AUTOMATICA_THRESHOLD;
  const bool uses_threshold_gfp = method_ == SynthesisMethod::THRESHOLD;
  const std::size_t scan_capacity = uses_scan ? max_basis_elements_ : 1;
  const std::size_t scan_coordinates = uses_scan ? basis_coordinates : state_dimension_;
  const std::size_t threshold_query_capacity =
      uses_automatica_threshold ? max_basis_elements_ : 1;
  const std::size_t automatica_table_entries =
      uses_automatica_threshold ? table_entries : 1;
  const std::size_t conversion_coordinates =
      uses_automatica_threshold ? basis_coordinates : state_dimension_;
  const std::size_t gfp_table_entries = uses_threshold_gfp ? table_entries : 1;

  auto load = [&](const char* descriptor, const char* function,
                  const std::vector<std::string>& arguments,
                  const std::vector<std::size_t>& multiples) {
    auto memory = pfacesKernelFunctionArguments::loadFromFile(
        path + descriptor, function, arguments, true);
    memory.m_baseTypeMultiple = multiples;
    addKernelFunction(pfacesKernelFunction(function, memory));
  };

  load("precompute_transitions.mem", "precompute_transitions",
       {"next_state_table", "runtime_params"}, {transition_entries, 4});
  load("predecessor_scan.mem", "predecessor_scan",
       {"query_flat_indices", "query_count", "next_state_table", "target_basis",
        "target_basis_size", "controlled_flags"},
       {scan_capacity, 1, transition_entries, scan_coordinates, 1,
        scan_capacity});
  load("predecessor_threshold.mem", "predecessor_threshold",
       {"query_flat_indices", "query_count", "next_state_table",
        "target_threshold", "controlled_flags"},
       {threshold_query_capacity, 1, transition_entries,
        automatica_table_entries, threshold_query_capacity});
  load("threshold_clear.mem", "threshold_clear", {"threshold_table"},
       {automatica_table_entries});
  load("threshold_scatter.mem", "threshold_scatter",
       {"basis", "basis_size", "threshold_table"},
       {conversion_coordinates, 1, automatica_table_entries});
  load("threshold_prefix.mem", "threshold_prefix",
       {"threshold_table", "sweep_params"}, {automatica_table_entries, 2});
  load("threshold_gfp_step.mem", "threshold_gfp_step",
       {"next_state_table", "threshold_a", "threshold_b", "source_parity",
        "changed_flag", "runtime_params"},
       {transition_entries, gfp_table_entries, gfp_table_entries, 1, 1, 4});

  const std::size_t reference_bitmap_words =
      method_ == SynthesisMethod::BITMAP_REFERENCE ? bitmap_words : 1;
  load("bitmap_gfp_iterate.mem", "bitmap_gfp_iterate",
       {"next_state_table", "bitmap_in", "bitmap_out", "changed_flag",
        "runtime_params"},
       {transition_entries, reference_bitmap_words, reference_bitmap_words, 1, 4});
  load("bitmap_gfp_advance.mem", "bitmap_gfp_advance",
       {"bitmap_in", "bitmap_out", "changed_flag"},
       {reference_bitmap_words, reference_bitmap_words, 1});
  load("bitmap_gfp_prefix.mem", "bitmap_gfp_prefix",
       {"bitmap", "sweep_params"}, {reference_bitmap_words, 2});
}

void pfacesKernel_mono_synth::configureParallelProgram(
    pfacesParallelProgram& program) {
  if (program.getTargetDevices().size() != 1) {
    throw std::invalid_argument(
        "MonoSynth currently requires exactly one target OpenCL device");
  }
  pfacesParallelAdvisor advisor(program.getMachine(),
                                program.getTargetDevicesIndicies());
  instructions_.clear();
  membership_jobs_.clear();
  conversion_scatter_jobs_.clear();

  std::vector<std::pair<char*, std::size_t>> data_pool;
  const pFacesMemoryAllocationReport report = allocateMemory(
      data_pool, program.getMachine(), program.getTargetDevicesIndicies(), 1, true);
  if (program.m_beVerboseLevel >= 2) report.PrintReport();
  allocated_bytes_ = report.totalAllocatedMemory;
  program.m_dataPool = std::move(data_pool);

  const cl::NDRange offset{0, 0, 0};
  appendCommonPrecompute(program, advisor, offset);
  switch (method_) {
    case SynthesisMethod::CDC:
      appendCdcSchedule(program, advisor, offset);
      break;
    case SynthesisMethod::AUTOMATICA_SCAN:
      appendAutomaticaSchedule(program, advisor, offset, MembershipKind::SCAN);
      break;
    case SynthesisMethod::AUTOMATICA_THRESHOLD:
      appendAutomaticaSchedule(program, advisor, offset, MembershipKind::THRESHOLD);
      break;
    case SynthesisMethod::THRESHOLD:
      appendThresholdSchedule(program, advisor, offset);
      break;
    case SynthesisMethod::BITMAP_REFERENCE:
    case SynthesisMethod::THRESHOLD_CPU_REFERENCE:
      appendReferenceSchedule(program, advisor, offset);
      break;
  }
  appendCommonFinalize(program);

  program.m_Universal_globalNDRange =
      program.m_Process_globalNDRange = cl::NDRange{
          static_cast<std::size_t>(total_states_), 1, 1};
  program.m_Universal_offsetNDRange = program.m_Process_offsetNDRange = offset;
  program.m_spInstructionList = instructions_;
}

void pfacesKernel_mono_synth::appendCommonPrecompute(
    pfacesParallelProgram& program, pfacesParallelAdvisor& advisor,
    cl::NDRange offset) {
  const cl::Device& device = program.getTargetDevices()[0];
  auto write_runtime = std::make_shared<pfacesDeviceWriteJob>(
      device, functionIndex(KernelFunction::PRECOMPUTE_TRANSITIONS), 2, 1);
  float* parameters = reinterpret_cast<float*>(
      program.m_dataPool[POOL_RUNTIME_PARAMS].first);
  parameters[0] = runtime_param0_;
  parameters[1] = parameters[2] = parameters[3] = 0.0f;
  addWrite(instructions_, write_runtime);

  if (backend_ == TransitionBackend::INLINE) {
    m_precompute_ms = 0.0;
    return;
  }

  bool cache_valid = false;
  if (!skip_cache_) {
    std::ifstream stream(cache_file_, std::ios::binary);
    CacheHeader header{};
    stream.read(reinterpret_cast<char*>(&header), sizeof(header));
    cache_valid = stream.good() && std::memcmp(header.magic, "MONOTR64", 8) == 0 &&
                  header.version == 2 && header.dimensions == state_dimension_ &&
                  header.states == total_states_ &&
                  header.fingerprint == cache_fingerprint_;
  }

  auto write_transitions = std::make_shared<pfacesDeviceWriteJob>(
      device, functionIndex(KernelFunction::PRECOMPUTE_TRANSITIONS), 2, 0);
  auto read_transitions = std::make_shared<pfacesDeviceReadJob>(
      device, functionIndex(KernelFunction::PRECOMPUTE_TRANSITIONS), 2, 0);
  if (cache_valid) {
    addSync(instructions_);
    addHost(instructions_, loadTransitionTable, "loadTransitionTableU64");
    addWrite(instructions_, write_transitions);
    return;
  }

  cl::NDRange precompute_range{static_cast<std::size_t>(total_states_), 1, 1};
  const auto jobs = advisor.distributeJob(
      *this, functionIndex(KernelFunction::PRECOMPUTE_TRANSITIONS),
      precompute_range, offset,
      program.m_isFixedJobDistribution, program.m_fixedJobDistribution, true,
      false, false);
  addSync(instructions_);
  addHost(instructions_, hostStartPrecomputeTimer, "startPrecomputeTimer");
  addExecute(instructions_, jobs);
  addRead(instructions_, read_transitions);
  addSync(instructions_);
  addHost(instructions_, hostStopPrecomputeTimer, "stopPrecomputeTimer");
  if (config_->isSaveTransitions() && !skip_cache_) {
    addHost(instructions_, saveTransitionTable, "saveTransitionTableU64");
  }
}

void pfacesKernel_mono_synth::appendCdcSchedule(
    pfacesParallelProgram& program, pfacesParallelAdvisor& advisor,
    cl::NDRange offset) {
  const cl::Device& device = program.getTargetDevices()[0];
  cl::NDRange membership_range{max_basis_elements_ * kMembershipWorkgroup, 1, 1};
  membership_jobs_ = advisor.distributeJob(
      *this, functionIndex(KernelFunction::PREDECESSOR_SCAN),
      membership_range, offset,
      program.m_isFixedJobDistribution, program.m_fixedJobDistribution, true,
      false, false);
  for (const auto& job : membership_jobs_) {
    for (const auto& task : job->getTasks()) {
      task->setNdRangeLocal(cl::NDRange{kMembershipWorkgroup, 1, 1});
    }
  }
  const std::size_t scan = functionIndex(KernelFunction::PREDECESSOR_SCAN);
  auto write_query = std::make_shared<pfacesDeviceWriteJob>(device, scan, 6, 0);
  auto write_query_count = std::make_shared<pfacesDeviceWriteJob>(device, scan, 6, 1);
  auto write_target = std::make_shared<pfacesDeviceWriteJob>(device, scan, 6, 3);
  auto write_target_size = std::make_shared<pfacesDeviceWriteJob>(device, scan, 6, 4);
  auto read_flags = std::make_shared<pfacesDeviceReadJob>(device, scan, 6, 5);

  addSync(instructions_);
  addHost(instructions_, hostInitCdc, "initLiteralCdc");
  const std::size_t loop = instructions_.size();
  addHost(instructions_, hostPrepareCdc, "prepareLiteralCdcEpoch");
  addWrite(instructions_, write_query);
  addWrite(instructions_, write_query_count);
  addWrite(instructions_, write_target);
  addWrite(instructions_, write_target_size);
  addExecute(instructions_, membership_jobs_);
  addRead(instructions_, read_flags);
  addSync(instructions_);
  addHost(instructions_, hostProcessCdc, "consumeFirstUnsafeCdcGenerator");
  auto jump = std::make_shared<pfacesInstruction>();
  jump->setAsJumpNe(loop);
  instructions_.push_back(jump);
}

void pfacesKernel_mono_synth::appendAutomaticaSchedule(
    pfacesParallelProgram& program, pfacesParallelAdvisor& advisor,
    cl::NDRange offset, MembershipKind membership) {
  automatica_membership_ = membership;
  const cl::Device& device = program.getTargetDevices()[0];
  const std::size_t function = membership == MembershipKind::SCAN
      ? functionIndex(KernelFunction::PREDECESSOR_SCAN)
      : functionIndex(KernelFunction::PREDECESSOR_THRESHOLD);
  const std::size_t argument_count = membership == MembershipKind::SCAN ? 6 : 5;
  const std::size_t lanes = membership == MembershipKind::SCAN ? kMembershipWorkgroup : 1;
  cl::NDRange membership_range{max_basis_elements_ * lanes, 1, 1};
  membership_jobs_ = advisor.distributeJob(
      *this, function, membership_range, offset,
      program.m_isFixedJobDistribution, program.m_fixedJobDistribution, true,
      false, false);
  if (membership == MembershipKind::SCAN) {
    for (const auto& job : membership_jobs_) {
      for (const auto& task : job->getTasks()) {
        task->setNdRangeLocal(cl::NDRange{kMembershipWorkgroup, 1, 1});
      }
    }
  }
  auto write_query = std::make_shared<pfacesDeviceWriteJob>(device, function,
                                                            argument_count, 0);
  auto write_query_count = std::make_shared<pfacesDeviceWriteJob>(
      device, function, argument_count, 1);
  auto read_flags = std::make_shared<pfacesDeviceReadJob>(
      device, function, argument_count, argument_count - 1);
  std::shared_ptr<pfacesDeviceWriteJob> write_target;
  std::shared_ptr<pfacesDeviceWriteJob> write_target_size;
  if (membership == MembershipKind::SCAN) {
    const std::size_t scan = functionIndex(KernelFunction::PREDECESSOR_SCAN);
    write_target = std::make_shared<pfacesDeviceWriteJob>(device, scan, 6, 3);
    write_target_size = std::make_shared<pfacesDeviceWriteJob>(device, scan, 6, 4);
  }

  addSync(instructions_);
  addHost(instructions_, hostInitAutomatica, "initAutomaticaGfp");
  const std::size_t outer_loop = instructions_.size();
  addHost(instructions_, hostPrepareAutomaticaOuter, "prepareAutomaticaOuter");

  if (membership == MembershipKind::SCAN) {
    addWrite(instructions_, write_target);
    addWrite(instructions_, write_target_size);
  }

  if (membership == MembershipKind::THRESHOLD) {
    cl::NDRange clear_range{static_cast<std::size_t>(threshold_table_size_), 1, 1};
    auto clear_jobs = advisor.distributeJob(
        *this, functionIndex(KernelFunction::THRESHOLD_CLEAR), clear_range,
        offset, program.m_isFixedJobDistribution, program.m_fixedJobDistribution,
        true, false, false);
    cl::NDRange scatter_range{max_basis_elements_, 1, 1};
    conversion_scatter_jobs_ = advisor.distributeJob(
        *this, functionIndex(KernelFunction::THRESHOLD_SCATTER), scatter_range, offset,
        program.m_isFixedJobDistribution, program.m_fixedJobDistribution, true,
        false, false);
    auto write_basis = std::make_shared<pfacesDeviceWriteJob>(
        device, functionIndex(KernelFunction::THRESHOLD_SCATTER), 3, 0);
    auto write_basis_size = std::make_shared<pfacesDeviceWriteJob>(
        device, functionIndex(KernelFunction::THRESHOLD_SCATTER), 3, 1);
    auto write_prefix = std::make_shared<pfacesDeviceWriteJob>(
        device, functionIndex(KernelFunction::THRESHOLD_PREFIX), 2, 1);
    addWrite(instructions_, write_basis);
    addWrite(instructions_, write_basis_size);
    addExecute(instructions_, clear_jobs);
    addExecute(instructions_, conversion_scatter_jobs_);
    addSync(instructions_);
    conversion_sweep_index_ = 0;
    for (std::size_t key_dimension : threshold_key_dimensions_) {
      const std::size_t fibers = static_cast<std::size_t>(
          threshold_table_size_ / grid_widths_[key_dimension]);
      cl::NDRange prefix_range{fibers, 1, 1};
      const auto prefix_jobs = advisor.distributeJob(
          *this, functionIndex(KernelFunction::THRESHOLD_PREFIX), prefix_range, offset,
          program.m_isFixedJobDistribution, program.m_fixedJobDistribution, true,
          false, false);
      addHost(instructions_, hostSetPrefixParams, "setThresholdPrefixParams");
      addWrite(instructions_, write_prefix);
      addExecute(instructions_, prefix_jobs);
      addSync(instructions_);
    }
    addHost(instructions_, hostFinishRepresentation,
            "finishBasisToThresholdTiming");
  }

  addSync(instructions_);
  const std::size_t inner_loop = instructions_.size();
  addHost(instructions_, hostPrepareAutomaticaBatch, "prepareAutomaticaFrontier");
  addWrite(instructions_, write_query);
  addWrite(instructions_, write_query_count);
  addExecute(instructions_, membership_jobs_);
  addRead(instructions_, read_flags);
  addSync(instructions_);
  addHost(instructions_, hostProcessAutomaticaBatch, "processAutomaticaFrontier");
  auto inner_jump = std::make_shared<pfacesInstruction>();
  inner_jump->setAsJumpNe(inner_loop);
  instructions_.push_back(inner_jump);

  addHost(instructions_, hostFinishAutomaticaOuter, "finishAutomaticaOuter");
  auto outer_jump = std::make_shared<pfacesInstruction>();
  outer_jump->setAsJumpNe(outer_loop);
  instructions_.push_back(outer_jump);
}

void pfacesKernel_mono_synth::appendThresholdSchedule(
    pfacesParallelProgram& program, pfacesParallelAdvisor& advisor,
    cl::NDRange offset) {
  const cl::Device& device = program.getTargetDevices()[0];
  cl::NDRange threshold_range{static_cast<std::size_t>(threshold_table_size_), 1, 1};
  const auto jobs = advisor.distributeJob(
      *this, functionIndex(KernelFunction::THRESHOLD_GFP_STEP),
      threshold_range, offset,
      program.m_isFixedJobDistribution, program.m_fixedJobDistribution, true,
      false, false);
  const std::size_t gfp = functionIndex(KernelFunction::THRESHOLD_GFP_STEP);
  auto write_a = std::make_shared<pfacesDeviceWriteJob>(device, gfp, 6, 1);
  auto write_b = std::make_shared<pfacesDeviceWriteJob>(device, gfp, 6, 2);
  auto write_parity = std::make_shared<pfacesDeviceWriteJob>(device, gfp, 6, 3);
  auto write_changed = std::make_shared<pfacesDeviceWriteJob>(device, gfp, 6, 4);
  auto read_a = std::make_shared<pfacesDeviceReadJob>(device, gfp, 6, 1);
  auto read_b = std::make_shared<pfacesDeviceReadJob>(device, gfp, 6, 2);
  auto read_changed = std::make_shared<pfacesDeviceReadJob>(device, gfp, 6, 4);

  addSync(instructions_);
  addHost(instructions_, hostInitThreshold, "initResidentThresholdGfp");
  addWrite(instructions_, write_a);
  addWrite(instructions_, write_b);
  addWrite(instructions_, write_parity);
  addWrite(instructions_, write_changed);
  addSync(instructions_);
  const std::size_t loop = instructions_.size();
  addHost(instructions_, hostPrepareThresholdRound, "prepareThresholdRound");
  addWrite(instructions_, write_parity);
  addWrite(instructions_, write_changed);
  addExecute(instructions_, jobs);
  addRead(instructions_, read_changed);
  addSync(instructions_);
  addHost(instructions_, hostProcessThresholdRound, "processThresholdRound");
  auto jump = std::make_shared<pfacesInstruction>();
  jump->setAsJumpNe(loop);
  instructions_.push_back(jump);
  addHost(instructions_, hostFinalThresholdIsB, "selectFinalThresholdBuffer");
  auto jump_to_b = std::make_shared<pfacesInstruction>();
  instructions_.push_back(jump_to_b);
  addRead(instructions_, read_a);
  auto jump_after_read = std::make_shared<pfacesInstruction>();
  instructions_.push_back(jump_after_read);
  const std::size_t read_b_index = instructions_.size();
  addRead(instructions_, read_b);
  const std::size_t after_read_index = instructions_.size();
  jump_to_b->setAsJumpNe(read_b_index);
  jump_after_read->setAsJumpUnconditional(after_read_index);
  addSync(instructions_);
  addHost(instructions_, hostFinalizeThreshold, "finalizeResidentThresholdGfp");
}

void pfacesKernel_mono_synth::appendReferenceSchedule(
    pfacesParallelProgram& program, pfacesParallelAdvisor& advisor,
    cl::NDRange offset) {
  if (method_ == SynthesisMethod::BITMAP_REFERENCE) {
    const cl::Device& device = program.getTargetDevices()[0];
    cl::NDRange iterate_range{static_cast<std::size_t>(total_states_), 1, 1};
    cl::NDRange word_range{static_cast<std::size_t>((total_states_ + 31) / 32), 1, 1};
    auto iterate_jobs = advisor.distributeJob(
        *this, functionIndex(KernelFunction::BITMAP_GFP_ITERATE), iterate_range,
        offset, program.m_isFixedJobDistribution,
        program.m_fixedJobDistribution, true, false, false);
    auto advance_jobs = advisor.distributeJob(
        *this, functionIndex(KernelFunction::BITMAP_GFP_ADVANCE), word_range,
        offset, program.m_isFixedJobDistribution,
        program.m_fixedJobDistribution, true, false, false);
    const std::size_t iterate = functionIndex(KernelFunction::BITMAP_GFP_ITERATE);
    const std::size_t prefix = functionIndex(KernelFunction::BITMAP_GFP_PREFIX);
    auto write_in = std::make_shared<pfacesDeviceWriteJob>(device, iterate, 5, 1);
    auto read_in = std::make_shared<pfacesDeviceReadJob>(device, iterate, 5, 1);
    auto write_out = std::make_shared<pfacesDeviceWriteJob>(device, iterate, 5, 2);
    auto write_changed = std::make_shared<pfacesDeviceWriteJob>(device, iterate, 5, 3);
    auto read_changed = std::make_shared<pfacesDeviceReadJob>(device, iterate, 5, 3);
    auto write_prefix = std::make_shared<pfacesDeviceWriteJob>(device, prefix, 2, 1);

    addSync(instructions_);
    addHost(instructions_, hostInitBitmapReference, "initBitmapReference");
    addWrite(instructions_, write_in);
    addWrite(instructions_, write_out);
    addWrite(instructions_, write_changed);
    addSync(instructions_);
    const std::size_t loop = instructions_.size();
    addHost(instructions_, hostPrepareBitmapReference, "prepareBitmapReference");
    addWrite(instructions_, write_out);
    addWrite(instructions_, write_changed);
    addExecute(instructions_, iterate_jobs);
    addSync(instructions_);
    for (std::size_t d = 0; d < state_dimension_; ++d) {
      const std::size_t fibers =
          static_cast<std::size_t>(total_states_ / grid_widths_[d]);
      cl::NDRange prefix_range{fibers, 1, 1};
      auto prefix_jobs = advisor.distributeJob(
          *this, prefix, prefix_range, offset, program.m_isFixedJobDistribution,
          program.m_fixedJobDistribution, true, false, false);
      addHost(instructions_, hostSetBitmapPrefixParams,
              "setBitmapPrefixParams");
      addWrite(instructions_, write_prefix);
      addExecute(instructions_, prefix_jobs);
      addSync(instructions_);
    }
    addExecute(instructions_, advance_jobs);
    addRead(instructions_, read_changed);
    addSync(instructions_);
    addHost(instructions_, hostProcessBitmapReference,
            "processBitmapReference");
    auto jump = std::make_shared<pfacesInstruction>();
    jump->setAsJumpNe(loop);
    instructions_.push_back(jump);
    addRead(instructions_, read_in);
    addSync(instructions_);
    addHost(instructions_, hostFinalizeBitmapReference,
            "finalizeBitmapReference");
    return;
  }
  addSync(instructions_);
  addHost(instructions_, hostRunCpuReference, "runReferenceGfp");
}

void pfacesKernel_mono_synth::appendCommonFinalize(pfacesParallelProgram&) {
  addSync(instructions_);
  addHost(instructions_, hostFinalizeSolver, "finalizeSolverStats");
  addSync(instructions_);
}

void pfacesKernel_mono_synth::resetStatsAndTimer() {
  stats_ = SolverStats{};
  stats_.method = method_;
  stats_.allocated_bytes = allocated_bytes_;
  m_iterations = 0;
  solver_timer_stopped_ = false;
  solver_timer_ = std::chrono::high_resolution_clock::now();
}

void pfacesKernel_mono_synth::stopSolverTimer() {
  if (solver_timer_stopped_) return;
  stats_.solver_ms = std::chrono::duration<double, std::milli>(
      std::chrono::high_resolution_clock::now() - solver_timer_).count();
  solver_timer_stopped_ = true;
}

void pfacesKernel_mono_synth::uploadBasis(pfacesParallelProgram& program,
                                          const BasisStore& query,
                                          const BasisStore& target,
                                          MembershipKind membership,
                                          bool upload_target) {
  const auto& query_flat = query.flat_indices();
  const auto& target_points = target.coordinates();
  if (query_flat.size() > max_basis_elements_ ||
      target_points.size() > max_basis_elements_) {
    throw std::length_error("basis exceeds configured max_basis_elements");
  }
  const std::size_t query_pool = membership == MembershipKind::SCAN
                                     ? POOL_SCAN_QUERY
                                     : POOL_THRESHOLD_QUERY;
  const std::size_t count_pool = membership == MembershipKind::SCAN
                                     ? POOL_SCAN_QUERY_COUNT
                                     : POOL_THRESHOLD_QUERY_COUNT;
  std::memcpy(program.m_dataPool[query_pool].first, query_flat.data(),
              query_flat.size() * sizeof(GridIndex));
  *reinterpret_cast<GridIndex*>(program.m_dataPool[count_pool].first) =
      query_flat.size();
  if (membership == MembershipKind::SCAN && upload_target) {
    auto* coordinates = reinterpret_cast<Height*>(
        program.m_dataPool[POOL_SCAN_TARGET_BASIS].first);
    std::size_t position = 0;
    for (const auto& point : target_points) {
      for (Height coordinate : point) coordinates[position++] = coordinate;
    }
    *reinterpret_cast<GridIndex*>(
        program.m_dataPool[POOL_SCAN_TARGET_SIZE].first) = target_points.size();
  }
  const std::size_t lanes = membership == MembershipKind::SCAN
                                ? kMembershipWorkgroup
                                : 1;
  const cl::NDRange range{std::max<std::size_t>(1, query.size() * lanes), 1, 1};
  for (const auto& job : membership_jobs_) {
    for (const auto& task : job->getTasks()) task->setNdRangeGlobal(range);
  }
}

size_t pfacesKernel_mono_synth::hostInitCdc(void* kernel, void*) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  self->resetStatsAndTimer();
  self->current_basis_.initialize_box(self->grid_widths_);
  return 0;
}

size_t pfacesKernel_mono_synth::hostPrepareCdc(void* kernel, void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  self->uploadBasis(*parallel, self->current_basis_, self->current_basis_,
                    MembershipKind::SCAN, true);
  ++self->stats_.cdc_epochs;
  ++self->m_iterations;
  self->stats_.membership_queries += self->current_basis_.size();
  self->phase_timer_ = std::chrono::high_resolution_clock::now();
  return 0;
}

size_t pfacesKernel_mono_synth::hostProcessCdc(void* kernel, void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  const auto* flags = reinterpret_cast<const std::uint32_t*>(
      parallel->m_dataPool[POOL_SCAN_FLAGS].first);
  self->stats_.membership_ms += std::chrono::duration<double, std::milli>(
      std::chrono::high_resolution_clock::now() - self->phase_timer_).count();
  std::size_t unsafe = self->current_basis_.size();
  for (std::size_t i = 0; i < self->current_basis_.size(); ++i) {
    if (flags[i] == 0) {
      unsafe = i;
      break;
    }
  }
  if (unsafe == self->current_basis_.size()) {
    self->stopSolverTimer();
    self->setCanonicalResult(self->current_basis_);
    return 0;
  }
  const auto start = std::chrono::high_resolution_clock::now();
  self->current_basis_.erase_one_and_expand(unsafe);
  const auto finish = std::chrono::high_resolution_clock::now();
  self->stats_.basis_update_ms +=
      std::chrono::duration<double, std::milli>(finish - start).count();
  ++self->stats_.cdc_mutations;
  if (self->current_basis_.empty()) {
    self->stopSolverTimer();
    self->setCanonicalResult(self->current_basis_);
    return 0;
  }
  return 1;
}

size_t pfacesKernel_mono_synth::hostInitAutomatica(void* kernel, void*) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  self->resetStatsAndTimer();
  self->current_basis_.initialize_box(self->grid_widths_);
  return 0;
}

size_t pfacesKernel_mono_synth::hostPrepareAutomaticaOuter(void* kernel,
                                                           void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  self->target_basis_ = self->current_basis_;
  self->work_basis_ = self->current_basis_;
  self->controlled_basis_.assign_antichain(self->grid_widths_, {});
  self->frontier_basis_ = self->work_basis_;
  self->conversion_sweep_index_ = 0;
  ++self->stats_.gfp_rounds;
  ++self->m_iterations;

  if (self->automatica_membership_ == MembershipKind::SCAN) {
    const auto& target = self->target_basis_.coordinates();
    auto* coordinates = reinterpret_cast<Height*>(
        parallel->m_dataPool[POOL_SCAN_TARGET_BASIS].first);
    std::size_t position = 0;
    for (const auto& point : target) {
      for (Height coordinate : point) coordinates[position++] = coordinate;
    }
    *reinterpret_cast<GridIndex*>(
        parallel->m_dataPool[POOL_SCAN_TARGET_SIZE].first) = target.size();
  } else {
    auto* coordinates = reinterpret_cast<Height*>(
        parallel->m_dataPool[POOL_CONVERSION_BASIS].first);
    std::size_t position = 0;
    for (const auto& point : self->target_basis_.coordinates()) {
      for (Height coordinate : point) coordinates[position++] = coordinate;
    }
    *reinterpret_cast<GridIndex*>(
        parallel->m_dataPool[POOL_CONVERSION_BASIS_SIZE].first) =
        self->target_basis_.size();
    const cl::NDRange scatter_range{
        std::max<std::size_t>(1, self->target_basis_.size()), 1, 1};
    for (const auto& job : self->conversion_scatter_jobs_) {
      for (const auto& task : job->getTasks()) {
        task->setNdRangeGlobal(scatter_range);
      }
    }
    self->phase_timer_ = std::chrono::high_resolution_clock::now();
  }
  return 0;
}

size_t pfacesKernel_mono_synth::hostPrepareAutomaticaBatch(void* kernel,
                                                           void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  self->uploadBasis(*parallel, self->frontier_basis_, self->target_basis_,
                    self->automatica_membership_, false);
  ++self->stats_.frontier_batches;
  self->stats_.membership_queries += self->frontier_basis_.size();
  self->phase_timer_ = std::chrono::high_resolution_clock::now();
  return 0;
}

size_t pfacesKernel_mono_synth::hostProcessAutomaticaBatch(void* kernel,
                                                           void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  const std::size_t flags_pool =
      self->automatica_membership_ == MembershipKind::SCAN
          ? POOL_SCAN_FLAGS
          : POOL_THRESHOLD_FLAGS;
  const auto* flags = reinterpret_cast<const std::uint32_t*>(
      parallel->m_dataPool[flags_pool].first);
  self->stats_.membership_ms += std::chrono::duration<double, std::milli>(
      std::chrono::high_resolution_clock::now() - self->phase_timer_).count();
  const auto frontier = self->frontier_basis_.coordinates();
  std::vector<BasisStore::Point> retained = self->controlled_basis_.coordinates();
  std::vector<BasisStore::Point> unsafe_points;
  for (std::size_t i = 0; i < frontier.size(); ++i) {
    if (flags[i]) retained.push_back(frontier[i]);
    else unsafe_points.push_back(frontier[i]);
  }
  // Both sets are disjoint subsets of the current canonical work basis:
  // controlled generators are never removed, and the frontier explicitly
  // excludes them. Their union is therefore already an antichain.
  self->controlled_basis_.assign_antichain(self->grid_widths_, retained);

  const auto update_start = std::chrono::high_resolution_clock::now();
  if (!unsafe_points.empty()) {
    std::vector<std::size_t> indices;
    const auto& work = self->work_basis_.coordinates();
    for (const auto& point : unsafe_points) {
      const auto found = std::lower_bound(work.begin(), work.end(), point);
      if (found == work.end() || *found != point) {
        throw std::logic_error("Automatica frontier is not a subset of Bas(work)");
      }
      indices.push_back(static_cast<std::size_t>(found - work.begin()));
    }
    self->work_basis_.erase_batch_and_expand(indices);
  }

  std::vector<BasisStore::Point> next_frontier;
  for (const auto& point : self->work_basis_.coordinates()) {
    if (!std::binary_search(self->controlled_basis_.coordinates().begin(),
                            self->controlled_basis_.coordinates().end(), point)) {
      next_frontier.push_back(point);
    }
  }
  // Filtering a canonical work basis preserves incomparability.
  self->frontier_basis_.assign_antichain(self->grid_widths_, next_frontier);
  const auto update_finish = std::chrono::high_resolution_clock::now();
  self->stats_.basis_update_ms +=
      std::chrono::duration<double, std::milli>(update_finish - update_start).count();
  return self->frontier_basis_.empty() ? 0 : 1;
}

size_t pfacesKernel_mono_synth::hostFinishAutomaticaOuter(void* kernel, void*) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  const bool changed = self->controlled_basis_ != self->current_basis_;
  self->current_basis_ = self->controlled_basis_;
  if (!changed || self->current_basis_.empty()) {
    self->stopSolverTimer();
    self->setCanonicalResult(self->current_basis_);
    return 0;
  }
  return 1;
}

size_t pfacesKernel_mono_synth::hostSetPrefixParams(void* kernel,
                                                    void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  if (self->conversion_sweep_index_ >= self->threshold_key_dimensions_.size()) {
    throw std::logic_error("threshold prefix sweep index overflow");
  }
  const std::size_t dimension =
      self->threshold_key_dimensions_[self->conversion_sweep_index_++];
  auto* parameters = reinterpret_cast<TableOffset*>(
      parallel->m_dataPool[POOL_PREFIX_PARAMS].first);
  parameters[0] = self->threshold_key_strides_[dimension];
  parameters[1] = self->grid_widths_[dimension];
  return 0;
}

size_t pfacesKernel_mono_synth::hostFinishRepresentation(void* kernel, void*) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  self->stats_.representation_ms += std::chrono::duration<double, std::milli>(
      std::chrono::high_resolution_clock::now() - self->phase_timer_).count();
  return 0;
}

size_t pfacesKernel_mono_synth::hostInitThreshold(void* kernel, void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  self->resetStatsAndTimer();
  const Height full_height = static_cast<Height>(
      self->grid_widths_[self->threshold_d_star_]);
  auto* a = reinterpret_cast<Height*>(parallel->m_dataPool[POOL_GFP_A].first);
  auto* b = reinterpret_cast<Height*>(parallel->m_dataPool[POOL_GFP_B].first);
  std::fill(a, a + self->threshold_table_size_, full_height);
  std::fill(b, b + self->threshold_table_size_, full_height);
  self->gfp_parity_ = 0;
  *reinterpret_cast<std::uint32_t*>(
      parallel->m_dataPool[POOL_GFP_PARITY].first) = 0;
  *reinterpret_cast<std::uint32_t*>(
      parallel->m_dataPool[POOL_GFP_CHANGED].first) = 0;
  return 0;
}

size_t pfacesKernel_mono_synth::hostPrepareThresholdRound(void* kernel,
                                                          void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  *reinterpret_cast<std::uint32_t*>(
      parallel->m_dataPool[POOL_GFP_PARITY].first) = self->gfp_parity_;
  *reinterpret_cast<std::uint32_t*>(
      parallel->m_dataPool[POOL_GFP_CHANGED].first) = 0;
  ++self->stats_.gfp_rounds;
  ++self->m_iterations;
  self->phase_timer_ = std::chrono::high_resolution_clock::now();
  return 0;
}

size_t pfacesKernel_mono_synth::hostProcessThresholdRound(void* kernel,
                                                          void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  const std::uint32_t changed = *reinterpret_cast<const std::uint32_t*>(
      parallel->m_dataPool[POOL_GFP_CHANGED].first);
  self->stats_.membership_ms += std::chrono::duration<double, std::milli>(
      std::chrono::high_resolution_clock::now() - self->phase_timer_).count();
  if (changed) {
    self->gfp_parity_ ^= 1u;
    return 1;
  }
  self->stopSolverTimer();
  return 0;
}

size_t pfacesKernel_mono_synth::hostFinalThresholdIsB(void* kernel, void*) {
  const auto* self = static_cast<const pfacesKernel_mono_synth*>(kernel);
  return self->gfp_parity_ != 0 ? 1 : 0;
}

size_t pfacesKernel_mono_synth::hostFinalizeThreshold(void* kernel,
                                                      void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  const std::size_t pool = self->gfp_parity_ ? POOL_GFP_B : POOL_GFP_A;
  const auto* source = reinterpret_cast<const Height*>(
      parallel->m_dataPool[pool].first);
  self->setCanonicalResult(std::vector<Height>(
      source, source + static_cast<std::size_t>(self->threshold_table_size_)));
  return 0;
}

std::vector<Height> pfacesKernel_mono_synth::cpuThresholdStep(
    const std::vector<Height>& source, const GridIndex* transitions,
    std::uint64_t* probes) const {
  std::vector<Height> destination(source.size(), 0);
  std::vector<GridIndex> full_strides(state_dimension_, 1);
  for (std::size_t d = 1; d < state_dimension_; ++d) {
    full_strides[d] = checkedMultiply(full_strides[d - 1], grid_widths_[d - 1]);
  }
  for (TableOffset key = 0; key < threshold_table_size_; ++key) {
    std::vector<Height> point(state_dimension_, 1);
    TableOffset rest = key;
    GridIndex base = 0;
    for (std::size_t d = 0; d < state_dimension_; ++d) {
      if (d == threshold_d_star_) continue;
      point[d] = static_cast<Height>(rest % grid_widths_[d] + 1);
      rest /= grid_widths_[d];
      base += static_cast<GridIndex>(point[d] - 1) * full_strides[d];
    }
    Height lower = 1;
    Height upper = source[static_cast<std::size_t>(key)];
    Height best = 0;
    while (lower <= upper && upper != 0) {
      const Height middle = lower + (upper - lower) / 2;
      ++*probes;
      const GridIndex successor = transitions[
          base + static_cast<GridIndex>(middle - 1) *
                     full_strides[threshold_d_star_]];
      bool is_controlled = successor != std::numeric_limits<GridIndex>::max();
      TableOffset successor_key = 0;
      TableOffset stride = 1;
      Height successor_height = 0;
      GridIndex value = successor;
      if (is_controlled) {
        for (std::size_t d = 0; d < state_dimension_; ++d) {
          const Height coordinate = static_cast<Height>(value % grid_widths_[d] + 1);
          value /= grid_widths_[d];
          if (d == threshold_d_star_) successor_height = coordinate;
          else {
            successor_key += static_cast<TableOffset>(coordinate - 1) * stride;
            stride = checkedMultiply(stride, grid_widths_[d]);
          }
        }
        is_controlled = successor_height <= source[successor_key];
      }
      if (is_controlled) {
        best = middle;
        lower = middle + 1;
      } else {
        upper = middle - 1;
      }
    }
    destination[static_cast<std::size_t>(key)] = best;
  }
  return destination;
}

size_t pfacesKernel_mono_synth::hostInitBitmapReference(void* kernel,
                                                        void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  self->resetStatsAndTimer();
  const std::size_t words = static_cast<std::size_t>((self->total_states_ + 31) / 32);
  auto* input = reinterpret_cast<cl_uint*>(
      parallel->m_dataPool[POOL_BITMAP_IN].first);
  auto* output = reinterpret_cast<cl_uint*>(
      parallel->m_dataPool[POOL_BITMAP_OUT].first);
  std::fill(input, input + words, std::numeric_limits<cl_uint>::max());
  if ((self->total_states_ & 31u) != 0) {
    input[words - 1] = (cl_uint(1) << (self->total_states_ & 31u)) - 1u;
  }
  std::fill(output, output + words, 0u);
  *reinterpret_cast<cl_int*>(
      parallel->m_dataPool[POOL_BITMAP_CHANGED].first) = 0;
  self->bitmap_sweep_index_ = 0;
  return 0;
}

size_t pfacesKernel_mono_synth::hostPrepareBitmapReference(void* kernel,
                                                           void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  const std::size_t words = static_cast<std::size_t>((self->total_states_ + 31) / 32);
  auto* output = reinterpret_cast<cl_uint*>(
      parallel->m_dataPool[POOL_BITMAP_OUT].first);
  std::fill(output, output + words, 0u);
  *reinterpret_cast<cl_int*>(
      parallel->m_dataPool[POOL_BITMAP_CHANGED].first) = 0;
  self->bitmap_sweep_index_ = 0;
  ++self->stats_.gfp_rounds;
  ++self->m_iterations;
  self->phase_timer_ = std::chrono::high_resolution_clock::now();
  return 0;
}

size_t pfacesKernel_mono_synth::hostSetBitmapPrefixParams(void* kernel,
                                                          void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  if (self->bitmap_sweep_index_ >= self->state_dimension_) {
    throw std::logic_error("bitmap prefix sweep index overflow");
  }
  const std::size_t dimension = self->bitmap_sweep_index_++;
  GridIndex stride = 1;
  for (std::size_t d = 0; d < dimension; ++d) {
    stride = checkedMultiply(stride, self->grid_widths_[d]);
  }
  if (stride > std::numeric_limits<cl_int>::max() ||
      self->grid_widths_[dimension] >
          static_cast<GridIndex>(std::numeric_limits<cl_int>::max())) {
    throw std::overflow_error("bitmap reference prefix parameters exceed int32");
  }
  auto* parameters = reinterpret_cast<cl_int*>(
      parallel->m_dataPool[POOL_BITMAP_PREFIX_PARAMS].first);
  parameters[0] = static_cast<cl_int>(stride);
  parameters[1] = static_cast<cl_int>(self->grid_widths_[dimension]);
  return 0;
}

size_t pfacesKernel_mono_synth::hostProcessBitmapReference(void* kernel,
                                                           void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  self->stats_.membership_ms += std::chrono::duration<double, std::milli>(
      std::chrono::high_resolution_clock::now() - self->phase_timer_).count();
  const bool changed = *reinterpret_cast<const cl_int*>(
      parallel->m_dataPool[POOL_BITMAP_CHANGED].first) != 0;
  if (!changed) self->stopSolverTimer();
  return changed ? 1 : 0;
}

size_t pfacesKernel_mono_synth::hostFinalizeBitmapReference(void* kernel,
                                                            void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  const std::size_t words = static_cast<std::size_t>((self->total_states_ + 31) / 32);
  const auto* input = reinterpret_cast<const cl_uint*>(
      parallel->m_dataPool[POOL_BITMAP_IN].first);
  self->bitmap_words_.assign(input, input + words);
  std::vector<Height> threshold(
      static_cast<std::size_t>(self->threshold_table_size_), 0);
  for (GridIndex flat = 0; flat < self->total_states_; ++flat) {
    if ((input[flat >> 5] & (cl_uint(1) << (flat & 31u))) == 0) continue;
    GridIndex rest = flat;
    TableOffset key = 0;
    TableOffset stride = 1;
    Height height = 0;
    for (std::size_t d = 0; d < self->state_dimension_; ++d) {
      const Height coordinate =
          static_cast<Height>(rest % self->grid_widths_[d] + 1);
      rest /= self->grid_widths_[d];
      if (d == self->threshold_d_star_) height = coordinate;
      else {
        key += static_cast<TableOffset>(coordinate - 1) * stride;
        stride = checkedMultiply(stride, self->grid_widths_[d]);
      }
    }
    threshold[key] = std::max(threshold[key], height);
  }
  self->setCanonicalResult(threshold);
  return 0;
}

size_t pfacesKernel_mono_synth::hostRunCpuReference(void* kernel,
                                                    void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  if (self->backend_ != TransitionBackend::PRECOMPUTED) {
    throw std::runtime_error("CPU reference modes require precomputed transitions");
  }
  self->resetStatsAndTimer();
  std::vector<Height> current(
      static_cast<std::size_t>(self->threshold_table_size_),
      static_cast<Height>(self->grid_widths_[self->threshold_d_star_]));
  const auto* transitions = reinterpret_cast<const GridIndex*>(
      parallel->m_dataPool[POOL_NEXT_STATE].first);
  while (true) {
    std::vector<Height> next = self->cpuThresholdStep(
        current, transitions, &self->stats_.binary_search_probes);
    ++self->stats_.gfp_rounds;
    ++self->m_iterations;
    if (next == current) break;
    current.swap(next);
  }
  self->stopSolverTimer();
  self->setCanonicalResult(current);
  return 0;
}

void pfacesKernel_mono_synth::setCanonicalResult(const BasisStore& basis) {
  threshold_result_u32_ = basis.to_threshold(threshold_d_star_, grid_widths_);
  basis_coordinates_.clear();
  if (extract_basis_) {
    for (const auto& point : basis.coordinates()) {
      for (Height coordinate : point) basis_coordinates_.push_back(coordinate);
    }
  }
}

void pfacesKernel_mono_synth::setCanonicalResult(
    const std::vector<Height>& threshold) {
  if (threshold.size() != threshold_table_size_) {
    throw std::invalid_argument("canonical threshold has the wrong size");
  }
  threshold_result_u32_ = threshold;
  if (extract_basis_) extractBasisFromThreshold();
  else basis_coordinates_.clear();
}

void pfacesKernel_mono_synth::extractBasisFromThreshold() {
  std::vector<BasisStore::Point> points;
  for (TableOffset key = 0; key < threshold_table_size_; ++key) {
    const Height height = threshold_result_u32_[static_cast<std::size_t>(key)];
    if (!height) continue;
    bool maximal = true;
    for (std::size_t d : threshold_key_dimensions_) {
      const TableOffset stride = threshold_key_strides_[d];
      const GridIndex coordinate = (key / stride) % grid_widths_[d];
      if (coordinate + 1 < grid_widths_[d] &&
          threshold_result_u32_[static_cast<std::size_t>(key + stride)] >= height) {
        maximal = false;
        break;
      }
    }
    if (!maximal) continue;
    BasisStore::Point point(state_dimension_, 1);
    TableOffset rest = key;
    for (std::size_t d = 0; d < state_dimension_; ++d) {
      if (d == threshold_d_star_) point[d] = height;
      else {
        point[d] = static_cast<Height>(rest % grid_widths_[d] + 1);
        rest /= grid_widths_[d];
      }
    }
    points.push_back(point);
  }
  BasisStore basis(grid_widths_, points, max_basis_elements_);
  basis_coordinates_.clear();
  for (const auto& point : basis.coordinates()) {
    for (Height coordinate : point) basis_coordinates_.push_back(coordinate);
  }
}

size_t pfacesKernel_mono_synth::hostFinalizeSolver(void* kernel, void*) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  self->stopSolverTimer();
  self->stats_.transition_ms = self->m_precompute_ms;
  self->m_gfp_total_ms = self->stats_.solver_ms;
  if (self->config_->isSaveController()) {
    const std::string output = std::string(self->config_->getProjectName()) +
                               ".threshold.u32.bin";
    std::ofstream stream(output, std::ios::binary | std::ios::trunc);
    if (!stream) throw std::runtime_error("cannot create canonical threshold output");
    stream.write(reinterpret_cast<const char*>(self->threshold_result_u32_.data()),
                 static_cast<std::streamsize>(self->threshold_result_u32_.size() *
                                              sizeof(Height)));
    if (!stream) throw std::runtime_error("cannot write canonical threshold output");
  }
  std::cout << "[MonoSynth] " << synthesisMethodName(self->method_)
            << " converged: safe_cells=" << self->getSafeCellCount()
            << ", rounds=" << self->stats_.gfp_rounds
            << ", cdc_epochs=" << self->stats_.cdc_epochs
            << ", solver_ms=" << self->stats_.solver_ms << std::endl;
  std::cout << "MONOSYNTH_STATS"
            << " method=" << synthesisMethodName(self->method_)
            << " safe_cells=" << self->getSafeCellCount()
            << " cdc_epochs=" << self->stats_.cdc_epochs
            << " cdc_mutations=" << self->stats_.cdc_mutations
            << " gfp_rounds=" << self->stats_.gfp_rounds
            << " frontier_batches=" << self->stats_.frontier_batches
            << " membership_queries=" << self->stats_.membership_queries
            << " binary_search_probes=" << self->stats_.binary_search_probes
            << " allocated_bytes=" << self->stats_.allocated_bytes
            << " transition_ms=" << self->stats_.transition_ms
            << " membership_ms=" << self->stats_.membership_ms
            << " representation_ms=" << self->stats_.representation_ms
            << " basis_update_ms=" << self->stats_.basis_update_ms
            << " solver_ms=" << self->stats_.solver_ms << std::endl;
  return 0;
}

std::int64_t pfacesKernel_mono_synth::getSafeCellCount() const {
  std::uint64_t count = 0;
  for (Height height : threshold_result_u32_) {
    if (count > std::numeric_limits<std::uint64_t>::max() - height) {
      throw std::overflow_error("safe-cell count overflows uint64_t");
    }
    count += height;
  }
  if (count > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
    throw std::overflow_error("safe-cell count does not fit int64_t");
  }
  return static_cast<std::int64_t>(count);
}

size_t pfacesKernel_mono_synth::hostStartPrecomputeTimer(void* kernel, void*) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  self->phase_timer_ = std::chrono::high_resolution_clock::now();
  return 0;
}

size_t pfacesKernel_mono_synth::hostStopPrecomputeTimer(void* kernel, void*) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  self->m_precompute_ms = std::chrono::duration<double, std::milli>(
      std::chrono::high_resolution_clock::now() - self->phase_timer_).count();
  return 0;
}

size_t pfacesKernel_mono_synth::saveTransitionTable(void* kernel,
                                                    void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  CacheHeader header{{'M','O','N','O','T','R','6','4'}, 2,
                     static_cast<std::uint32_t>(self->state_dimension_),
                     self->total_states_, self->cache_fingerprint_};
  std::ofstream stream(self->cache_file_, std::ios::binary | std::ios::trunc);
  if (!stream) throw std::runtime_error("cannot create transition cache");
  stream.write(reinterpret_cast<const char*>(&header), sizeof(header));
  stream.write(parallel->m_dataPool[POOL_NEXT_STATE].first,
               static_cast<std::streamsize>(self->total_states_ * sizeof(GridIndex)));
  if (!stream) throw std::runtime_error("cannot write complete transition cache");
  return 0;
}

size_t pfacesKernel_mono_synth::loadTransitionTable(void* kernel,
                                                    void* program) {
  auto* self = static_cast<pfacesKernel_mono_synth*>(kernel);
  auto* parallel = static_cast<pfacesParallelProgram*>(program);
  std::ifstream stream(self->cache_file_, std::ios::binary);
  CacheHeader header{};
  stream.read(reinterpret_cast<char*>(&header), sizeof(header));
  const bool valid = stream.good() && std::memcmp(header.magic, "MONOTR64", 8) == 0 &&
                     header.version == 2 && header.dimensions == self->state_dimension_ &&
                     header.states == self->total_states_ &&
                     header.fingerprint == self->cache_fingerprint_;
  if (!valid) throw std::runtime_error("transition cache header mismatch");
  stream.read(parallel->m_dataPool[POOL_NEXT_STATE].first,
              static_cast<std::streamsize>(self->total_states_ * sizeof(GridIndex)));
  if (!stream) throw std::runtime_error("transition cache is truncated");
  return 0;
}

void pfacesKernel_mono_synth::configureTuneParallelProgram(
    pfacesParallelProgram& program, std::size_t target_function) {
  if (target_function > functionIndex(KernelFunction::THRESHOLD_GFP_STEP)) {
    throw std::invalid_argument("reference kernels are not tuning targets");
  }
  pfacesParallelAdvisor advisor(program.getMachine(),
                                program.getTargetDevicesIndicies());
  program.m_isFixedJobDistribution = true;
  program.m_fixedJobDistribution = {1.0};
  cl::NDRange offset{0, 0, 0};
  std::size_t work = 1;
  if (target_function == 0) work = static_cast<std::size_t>(total_states_);
  else if (target_function == 1) work = max_basis_elements_ * kMembershipWorkgroup;
  else if (target_function == 2 || target_function == 4) work = max_basis_elements_;
  else if (target_function == 3 || target_function == 6)
    work = static_cast<std::size_t>(threshold_table_size_);
  cl::NDRange range{work, 1, 1};
  const auto jobs = advisor.distributeJob(*this, target_function, range, offset,
                                          true, {1.0}, true, false, false);
  std::vector<std::pair<char*, std::size_t>> pool;
  allocateMemory(pool, program.getMachine(), program.getTargetDevicesIndicies(),
                 1, true);
  std::vector<std::shared_ptr<pfacesInstruction>> list;
  addExecute(list, jobs);
  program.m_dataPool = std::move(pool);
  program.m_spInstructionList = std::move(list);
  program.m_Universal_globalNDRange = program.m_Process_globalNDRange = range;
  program.m_Universal_offsetNDRange = program.m_Process_offsetNDRange = offset;
}

}  // namespace mono_synth

PFACES_REGISTER_LOADABLE_KERNEL(mono_synth::pfacesKernel_mono_synth)
