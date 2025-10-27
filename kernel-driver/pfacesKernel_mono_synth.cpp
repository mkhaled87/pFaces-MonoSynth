/*
* pfacesKernel_mono_synth.cpp
*
*  created on: 02.10.2025
*      author: M. Khaled
*/

#include <ctime>
#include <cmath>
#include <algorithm>
#include <vector>
#include <cstring>
#include <thread>
#include <mutex>
#include <atomic>
#include "pfacesKernel_mono_synth.h"

// Constants from safe_set.cpp
const double DT_2D = 0.1;
const double H_MIN_2D = 5.0, H_MAX_2D = 80.0;
const double V_MIN_2D = 0.0, V_MAX_2D = 20.0;
const double H_RES_2D = 0.2, V_RES_2D = 0.1;
const double U_MIN_2D = -3.0, U_MAX_2D = 3.0;
const double U_RES_2D = 0.1;
const double DT_3D = 0.4;
const double H_MIN_3D = 0.0, H_MAX_3D = 80.0;
const double V_MIN_3D = 0.0, V_MAX_3D = 20.0;
const double H_RES_3D = 0.8, V_RES_3D = 0.4;
const double T_RES_3D = 150.0;
const double T_MAX = 1200.0, T_BRAKE_MIN = -1800.0, T_BRAKE_MAX = -2400.0;
const double M_MIN = 2000.0, M_MAX = 2500.0;
const double R_W_MIN = 0.30, R_W_MAX = 0.35;
const double ALPHA_MIN = 300.0, ALPHA_MAX = 350.0;
const double BETA_MIN = 0.10, BETA_MAX = 0.25;
const double GAMMA_MIN = 0.30, GAMMA_MAX = 0.65;
const int MAX_STATE_DIM = 3;
const int MAX_INPUT_DIM = 1;
const int MAX_DISTURBANCE_DIM = 1;
const int MAX_BASIS_ELEMENTS = 2000;
const int RK4_STEPS_OPTIMIZED = 200;

namespace mono_synth {

// Safe set computation classes (from safe_set.cpp)
class MonotoneDynamics {
private:
    double dt;
    bool is_3d;
public:
    MonotoneDynamics(int /*state_dim*/, int /*input_dim*/, int /*disturbance_dim*/, double timestep, bool use_3d)
        : dt(timestep), is_3d(use_3d) {}
    void nextState(const double* x, const double* u, const double* w, double* x_plus) const {
        if (is_3d) {
            solve3DDynamics(x, u, w, x_plus);
        } else {
            // 2D dynamics: h(t+dt) = h(t) + v_L*dt - v(t)*dt - 0.5*a*dt^2, v(t+dt) = v(t) + a*dt
            x_plus[0] = x[0] + w[0]*dt - x[1]*dt - 0.5*u[0]*dt*dt;
            x_plus[1] = x[1] + u[0]*dt;
        }
    }
private:
    void solve3DDynamics(const double* x, const double* u, const double* w, double* x_plus) const {
        // === 1) Choose worst-case parameters for ego vehicle ===
        double R_w_ego = (u[0] > 0) ? R_W_MIN : R_W_MAX;
        double M_ego   = ((u[0] / R_w_ego - ALPHA_MIN) > 0) ? M_MIN : M_MAX;
        double a  = (1.0 / M_ego) * (u[0] / R_w_ego - ALPHA_MIN);
        double b  = -(1.0 / M_MAX) * BETA_MIN;
        double c  = -(1.0 / M_MAX) * GAMMA_MIN;

        // === 2) Choose worst-case parameters for lead vehicle ===
        double R_w_L = (w[0] > 0) ? R_W_MAX : R_W_MIN;
        double M_L   = ((w[0] / R_w_L - ALPHA_MAX) > 0) ? M_MAX : M_MIN;
        double a_L = (1.0 / M_L) * (w[0] / R_w_L - ALPHA_MAX);
        double b_L = -(1.0 / M_MIN) * BETA_MAX;
        double c_L = -(1.0 / M_MIN) * GAMMA_MAX;

        // === 3) Initial states ===
        double s  = x[0];   // relative distance
        double vE = x[1];   // ego velocity
        double vL = x[2];   // lead velocity

        // === 4) Integrate combined ODE using RK4 ===
        const int num_steps = RK4_STEPS_OPTIMIZED;  // same as before
        const double h = dt / num_steps;

        for (int i = 0; i < num_steps; ++i) {
            double k1_s, k1_vE, k1_vL;
            double k2_s, k2_vE, k2_vL;
            double k3_s, k3_vE, k3_vL;
            double k4_s, k4_vE, k4_vL;

            f_combined(s, vE, vL, a, b, c, a_L, b_L, c_L, k1_s, k1_vE, k1_vL);
            f_combined(s + 0.5*h*k1_s, vE + 0.5*h*k1_vE, vL + 0.5*h*k1_vL,
                       a, b, c, a_L, b_L, c_L, k2_s, k2_vE, k2_vL);
            f_combined(s + 0.5*h*k2_s, vE + 0.5*h*k2_vE, vL + 0.5*h*k2_vL,
                       a, b, c, a_L, b_L, c_L, k3_s, k3_vE, k3_vL);
            f_combined(s + h*k3_s, vE + h*k3_vE, vL + h*k3_vL,
                       a, b, c, a_L, b_L, c_L, k4_s, k4_vE, k4_vL);

            s  += h/6.0 * (k1_s  + 2*k2_s  + 2*k3_s  + k4_s);
            vE += h/6.0 * (k1_vE + 2*k2_vE + 2*k3_vE + k4_vE);
            vL += h/6.0 * (k1_vL + 2*k2_vL + 2*k3_vL + k4_vL);

            // Clamp velocities during integration
            if (vE <= 0.0) vE = 0.0;
            if (vL >= V_MAX_3D) vL = V_MAX_3D;
            if (vL <= 0.0) vL = 0.0;
        }

        // === 5) Clip final results to limits and assign outputs ===
        x_plus[0] = s;
        x_plus[1] = std::max(V_MIN_3D, std::min(vE, V_MAX_3D));
        x_plus[2] = std::max(V_MIN_3D, std::min(vL, V_MAX_3D));
    }

    void f_combined(double /*s*/, double vE, double vL,
                    double a, double b, double c,
                    double aL, double bL, double cL,
                    double& ds, double& dvE, double& dvL) const {
        // Relative distance derivative
        ds  = vL - vE;

        // Ego vehicle acceleration
        dvE = a + b * vE + c * vE * vE;
        if (vE <= 0.0 && dvE < 0.0) dvE = 0.0;

        // Lead vehicle acceleration
        dvL = aL + bL * vL + cL * vL * vL;
        if (vL >= V_MAX_3D && dvL > 0.0) dvL = 0.0;
        if (vL <= 0.0 && dvL < 0.0) dvL = 0.0;
    }
};

class MonotoneAbstraction {
private:
    MonotoneDynamics dyn;
    
    // Fixed-size arrays instead of STL containers
    double x_range_min[MAX_STATE_DIM], x_range_max[MAX_STATE_DIM];
    double u_range_min[MAX_INPUT_DIM], u_range_max[MAX_INPUT_DIM];
    double w_range_min[MAX_DISTURBANCE_DIM], w_range_max[MAX_DISTURBANCE_DIM];
    double x_res[MAX_STATE_DIM], u_res[MAX_INPUT_DIM], w_res[MAX_DISTURBANCE_DIM];
    int x_numCells[MAX_STATE_DIM], u_numCells[MAX_INPUT_DIM], w_numCells[MAX_DISTURBANCE_DIM];
    int x_priority[MAX_STATE_DIM], u_priority[MAX_INPUT_DIM], w_priority[MAX_INPUT_DIM];
    
    // Safe set basis as flattened 2D array: [basis_index][dimension]
    int safe_set_basis[MAX_BASIS_ELEMENTS * MAX_STATE_DIM];
    int safe_set_size;  // Current number of basis elements
    
    // Transition cache as flattened arrays
    int transition_cache_keys[MAX_BASIS_ELEMENTS];
    int transition_cache_values[MAX_BASIS_ELEMENTS * MAX_STATE_DIM];
    int transition_cache_size;
    
    bool is_3d;
    mutable std::mutex cache_mutex;
    
public:
    MonotoneAbstraction(const MonotoneDynamics& dynamics, bool use_3d) 
        : dyn(dynamics), safe_set_size(0), transition_cache_size(0), is_3d(use_3d) {
        memset(safe_set_basis, 0, sizeof(safe_set_basis));
        memset(transition_cache_keys, 0, sizeof(transition_cache_keys));
        memset(transition_cache_values, 0, sizeof(transition_cache_values));
    }
    
    void setupRanges() {
        if (is_3d) {
            x_range_min[0] = H_MIN_3D; x_range_max[0] = H_MAX_3D;
            x_range_min[1] = V_MIN_3D; x_range_max[1] = V_MAX_3D;
            x_range_min[2] = V_MIN_3D; x_range_max[2] = V_MAX_3D;
            u_range_min[0] = T_BRAKE_MIN; u_range_max[0] = T_MAX;
            w_range_min[0] = T_BRAKE_MAX; w_range_max[0] = T_MAX;
            x_res[0] = H_RES_3D; x_res[1] = V_RES_3D; x_res[2] = V_RES_3D;
            u_res[0] = T_RES_3D;
            w_res[0] = T_RES_3D;
            x_priority[0] = 0; x_priority[1] = 1; x_priority[2] = 0;
            u_priority[0] = 1;
            w_priority[0] = 0;
        } else {
            x_range_min[0] = H_MIN_2D; x_range_max[0] = H_MAX_2D;
            x_range_min[1] = V_MIN_2D; x_range_max[1] = V_MAX_2D;
            u_range_min[0] = U_MIN_2D; u_range_max[0] = U_MAX_2D;
            w_range_min[0] = V_MIN_2D; w_range_max[0] = V_MAX_2D;
            x_res[0] = H_RES_2D; x_res[1] = V_RES_2D;
            u_res[0] = U_RES_2D;
            w_res[0] = V_RES_2D;
            x_priority[0] = 0; x_priority[1] = 1;
            u_priority[0] = 1;
            w_priority[0] = 0;
        }
        
        int state_dim = is_3d ? 3 : 2;
        for (int i = 0; i < state_dim; ++i) {
            x_numCells[i] = std::ceil((x_range_max[i] - x_range_min[i]) / x_res[i]) + 1;
        }
        for (int i = 0; i < MAX_INPUT_DIM; ++i) {
            u_numCells[i] = std::ceil((u_range_max[i] - u_range_min[i]) / u_res[i]) + 1;
        }
        for (int i = 0; i < MAX_DISTURBANCE_DIM; ++i) {
            w_numCells[i] = std::ceil((w_range_max[i] - w_range_min[i]) / w_res[i]) + 1;
        }
    }
    
    void initializeSafeSet() {
        safe_set_size = 0;
        if (is_3d) {
			 // 3D without precomputation: Initialize with entire state space
			 safe_set_basis[0] = x_numCells[0];
			 safe_set_basis[1] = x_numCells[1];
			 safe_set_basis[2] = x_numCells[2];
			 safe_set_size = 1;
			 std::cout << "3D without precomputation completed. Basis size: " << safe_set_size << std::endl;
        } else {
            // 2D: Initialize with entire state space
            int state_dim = 2;
            for (int i = 0; i < state_dim; ++i) {
                safe_set_basis[safe_set_size * MAX_STATE_DIM + i] = x_numCells[i];
            }
            safe_set_size = 1;
        }
    }
    
    void getPriorityStateAtIdx(const int* idx, double* val) const {
        int state_dim = is_3d ? 3 : 2;
        for (int i = 0; i < state_dim; ++i) {
            if (x_priority[i] == 1) {
                val[i] = x_range_min[i] + (idx[i] - 1) * x_res[i];
            } else {
                val[i] = x_range_max[i] - (idx[i] - 1) * x_res[i];
            }
        }
    }
    
    bool getStateIdx(const double* val, int* idx) const {
        int state_dim = is_3d ? 3 : 2;
        const double threshold = 1e-5;
        
        for (int i = 0; i < state_dim; ++i) {
            if (val[i] < x_range_min[i] - threshold || val[i] > x_range_max[i] + threshold) {
                idx[0] = -1;
                return false;
            }
            
            if (x_priority[i] == 1) {
                idx[i] = std::ceil((val[i] - x_range_min[i]) / x_res[i] - threshold) + 1;
            } else {
                idx[i] = std::ceil((x_range_max[i] - val[i]) / x_res[i] - threshold) + 1;
            }
        }
        return true;
    }
    
    bool xInSafeSet(const int* x_idx) const {
        if (x_idx[0] == -1) return false;
        
        int state_dim = is_3d ? 3 : 2;
        for (int i = safe_set_size - 1; i >= 0; --i) {
            bool dominated = true;
            for (int j = 0; j < state_dim; ++j) {
                if (x_idx[j] > safe_set_basis[i * MAX_STATE_DIM + j]) {
                    dominated = false;
                    break;
                }
            }
            if (dominated) return true;
        }
        return false;
    }
    
    void getTransitionState(const int* x_idx, const int* u_idx, const int* w_idx, int* x_plus_idx) {
        int flat_idx = flattenIndex(x_idx);
        
        {
            std::lock_guard<std::mutex> lock(cache_mutex);
            for (int i = 0; i < transition_cache_size; ++i) {
                if (transition_cache_keys[i] == flat_idx) {
                    int state_dim = is_3d ? 3 : 2;
                    for (int j = 0; j < state_dim; ++j) {
                        x_plus_idx[j] = transition_cache_values[i * MAX_STATE_DIM + j];
                    }
                    return;
                }
            }
        }
        
        int state_dim = is_3d ? 3 : 2;
        double x_val[MAX_STATE_DIM], u_val[MAX_INPUT_DIM], w_val[MAX_DISTURBANCE_DIM];
        double x_plus_val[MAX_STATE_DIM];
        
        getPriorityStateAtIdx(x_idx, x_val);
        getPriorityInputAtIdx(u_idx, u_val);
        getPriorityDisturbanceAtIdx(w_idx, w_val);
        
        dyn.nextState(x_val, u_val, w_val, x_plus_val);
        getStateIdx(x_plus_val, x_plus_idx);
        
        {
            std::lock_guard<std::mutex> lock(cache_mutex);
            if (transition_cache_size < MAX_BASIS_ELEMENTS) {
                transition_cache_keys[transition_cache_size] = flat_idx;
                for (int j = 0; j < state_dim; ++j) {
                    transition_cache_values[transition_cache_size * MAX_STATE_DIM + j] = x_plus_idx[j];
                }
                transition_cache_size++;
            }
        }
    }
    
    void removeBasisIdx(const int* basis_idx) {
        int state_dim = is_3d ? 3 : 2;
        
        for (int i = 0; i < safe_set_size; ++i) {
            bool match = true;
            for (int j = 0; j < state_dim; ++j) {
                if (safe_set_basis[i * MAX_STATE_DIM + j] != basis_idx[j]) {
                    match = false;
                    break;
                }
            }
            if (match) {
                if (i < safe_set_size - 1) {
                    memmove(&safe_set_basis[i * MAX_STATE_DIM], 
                           &safe_set_basis[(i + 1) * MAX_STATE_DIM],
                           (safe_set_size - i - 1) * MAX_STATE_DIM * sizeof(int));
                }
                safe_set_size--;
                
                for (int j = 0; j < state_dim; ++j) {
                    if (basis_idx[j] == 1) continue;
                    int candidate[MAX_STATE_DIM];
                    for (int k = 0; k < state_dim; ++k) {
                        candidate[k] = basis_idx[k];
                    }
                    candidate[j] -= 1;
                    
                    if (!xInSafeSet(candidate)) {
                        addBasisIdx(candidate);
                    }
                }
                break;
            }
        }
    }
    
    void addBasisIdx(const int* x_idx) {
        if (safe_set_size >= MAX_BASIS_ELEMENTS) return;
        if (xInSafeSet(x_idx)) return;
        
        int state_dim = is_3d ? 3 : 2;
        for (int i = 0; i < state_dim; ++i) {
            safe_set_basis[safe_set_size * MAX_STATE_DIM + i] = x_idx[i];
        }
        safe_set_size++;
    }
    
    void computeSafeSet() {
        std::cout << "Starting invariant set computation..." << std::endl;
        
        int iter = 0;
        while (true) {
            int unsafe_basis_indices[MAX_BASIS_ELEMENTS * MAX_STATE_DIM];
            int unsafe_count = 0;
            iter++;
            
            if (safe_set_size == 0) break;
            
            for (int i = 0; i < safe_set_size; ++i) {
                int x_idx[MAX_STATE_DIM];
                for (int j = 0; j < (is_3d ? 3 : 2); ++j) {
                    x_idx[j] = safe_set_basis[i * MAX_STATE_DIM + j];
                }
                
                double x_val[MAX_STATE_DIM];
                getPriorityStateAtIdx(x_idx, x_val);
                
                bool is_unsafe = false;
                
                if (is_3d) {
                    int u_idx[1] = {1};
                    int w_idx[1] = {w_numCells[0]};
                    
                    int x_plus_idx[MAX_STATE_DIM];
                    getTransitionState(x_idx, u_idx, w_idx, x_plus_idx);
                    if (!xInSafeSet(x_plus_idx)) {
                        is_unsafe = true;
                    }
                } else {
                    double velocity = x_val[1];
                    double u_val = std::max(U_MIN_2D, -10.0 * velocity);
                    double u_val_vec[1] = {u_val};
                    int u_idx[1];
                    getInputIdx(u_val_vec, u_idx);
                    
                    double w_val = V_MIN_2D;
                    double w_val_vec[1] = {w_val};
                    int w_idx[1];
                    getDisturbanceIdx(w_val_vec, w_idx);
                    
                    int x_plus_idx[MAX_STATE_DIM];
                    getTransitionState(x_idx, u_idx, w_idx, x_plus_idx);
                    
                    if (!xInSafeSet(x_plus_idx)) {
                        is_unsafe = true;
                    }
                }
                
                if (is_unsafe) {
                    for (int j = 0; j < (is_3d ? 3 : 2); ++j) {
                        unsafe_basis_indices[unsafe_count * MAX_STATE_DIM + j] = x_idx[j];
                    }
                    unsafe_count++;
                }
            }
            
            if (unsafe_count == 0) {
                std::cout << "Converged! All basis elements are safe." << std::endl;
                break;
            } else {
                for (int i = 0; i < unsafe_count; ++i) {
                    int basis_idx[MAX_STATE_DIM];
                    for (int j = 0; j < (is_3d ? 3 : 2); ++j) {
                        basis_idx[j] = unsafe_basis_indices[i * MAX_STATE_DIM + j];
                    }
                    removeBasisIdx(basis_idx);
                }
            }
        }
        
        std::cout << "Safe set computation completed in " << iter << " iterations." << std::endl;
        std::cout << "Final safe set has " << safe_set_size << " basis elements." << std::endl;
    }
    
private:
    int flattenIndex(const int* idx) const {
        int result = 0, multiplier = 1;
        int state_dim = is_3d ? 3 : 2;
        for (int i = 0; i < state_dim; ++i) {
            result += (idx[i] - 1) * multiplier;
            multiplier *= x_numCells[i];
        }
        return result;
    }
    
    void getPriorityInputAtIdx(const int* idx, double* val) const {
        for (int i = 0; i < MAX_INPUT_DIM; ++i) {
            if (u_priority[i] == 1) {
                val[i] = u_range_min[i] + (idx[i] - 1) * u_res[i];
            } else {
                val[i] = u_range_max[i] - (idx[i] - 1) * u_res[i];
            }
        }
    }
    
    void getPriorityDisturbanceAtIdx(const int* idx, double* val) const {
        for (int i = 0; i < MAX_DISTURBANCE_DIM; ++i) {
            if (w_priority[i] == 1) {
                val[i] = w_range_min[i] + (idx[i] - 1) * w_res[i];
            } else {
                val[i] = w_range_max[i] - (idx[i] - 1) * w_res[i];
            }
        }
    }
    
    bool getInputIdx(const double* val, int* idx) const {
        const double threshold = 1e-5;
        
        for (int i = 0; i < MAX_INPUT_DIM; ++i) {
            if (val[i] < u_range_min[i] - threshold || val[i] > u_range_max[i] + threshold) {
                idx[0] = -1;
                return false;
            }
            
            if (u_priority[i] == 1) {
                idx[i] = std::ceil((val[i] - u_range_min[i]) / u_res[i] - threshold) + 1;
            } else {
                idx[i] = std::ceil((u_range_max[i] - val[i]) / u_res[i] - threshold) + 1;
            }
        }
        return true;
    }
    
    bool getDisturbanceIdx(const double* val, int* idx) const {
        const double threshold = 1e-5;
        
        for (int i = 0; i < MAX_DISTURBANCE_DIM; ++i) {
            if (val[i] < w_range_min[i] - threshold || val[i] > w_range_max[i] + threshold) {
                idx[0] = -1;
                return false;
            }
            
            if (w_priority[i] == 1) {
                idx[i] = std::ceil((val[i] - w_range_min[i]) / w_res[i] - threshold) + 1;
            } else {
                idx[i] = std::ceil((w_range_max[i] - val[i]) / w_res[i] - threshold) + 1;
            }
        }
        return true;
    }
};

void runSafeSetComputation(const pfacesKernel_mono_synth* pKernel, size_t beVerboseLevel) {
    if (beVerboseLevel >= 1) {
        std::cout << "Running safe_set computation (monotone abstraction)..." << std::endl;
    }
    
    double tau = pKernel->m_spCfg->getSamplingPeriod();
    size_t ssDim = pKernel->m_spCfg->getSsDim();
    bool is_3d = (ssDim == 3);
    
    // Create dynamics
    MonotoneDynamics dyn(ssDim, 1, 1, tau, is_3d);
    
    // Create abstraction
    MonotoneAbstraction abstraction(dyn, is_3d);
    
    // Setup ranges
    abstraction.setupRanges();
    
    // Initialize safe set
    abstraction.initializeSafeSet();
    
    // Compute safe set
    abstraction.computeSafeSet();
    
    if (beVerboseLevel >= 1) {
        std::cout << "Safe_set computation complete." << std::endl;
    }
}


	/*
	
	What needs to be done (if complex, start serial then parallelize):

	1- A kernel function to partition the state space U into subsets U_i i \in 1..N
		=> this can to be done in parallel as the input space might be large, but usually in the dimension of 1-3
		=> there are many known parallel partitioning algorithms in the literature
		=> partitioning should be based on the preorder provided for the input space
		=> input: U (lb, ub, eta) and the preorder relation 
			=> Q: How the preorder is given in the Matlab code?
		=> output: list of size |U| with each element being the index of the subset it belongs to (from 1..N)
		=> this means this parallel algorithm will be parallel on the output (i.e., no of threads will be |U|), and each thread will search for the subset it belongs to.


	2- A kernel function for computing the InvariantSet in parallel (Algorithm 1)
		=> Q: how the invariant set is represented in memory for faster/parallel access(R/W)/processing?
				=> Let's fix the size of any set Z_out to be |X_safe|
				=> each element as flag to indicate if the state x \in Z_out belongs to the set or not
		=> We know Basis \in X_safe (same apply to all Basis sets like B^{us} and B^{pr}) => Could be also represented with a list of size |X_safe| with flags
		=> LEt's also for now assume that all sets are hyper-rectangles, and based on this, we assume also that the Basis point is just the upper-right corner 
			of the hyper-rectangle and we later change this if this is wrong
		=> Line 4. is a set comparison => can be done with global atomic where each thread tries to write to it only if it finds out that its state has no match (=> minimal write, but the atomic memory flag needs to be reset first)
			=> use also privatization to reduce the number of global atomic writes 
		=> seems this algorithm could be parallel on Z_ex as it used to get the basis B used in the main for loop
			=> instead of launching everytime |X_safe| threads, we could launch |Z_ex| threads but two problems here:
				-> mapping of threads ids to Z_ex will be complex (Z_ex is not contiguous subset of X_safe)
		=> If we know |U| will be big, we could also launch |X_safe|*|U| threads as we will need to iterate per (x,u) in line 7
		=> I think it is better to merge lines 5 and 7 into parallel threads over (x,u) and make sure the initializations in lines 6,8 happens before
		=> line 9 is where the abstraction will be done on the fly and the check of inclusion is easily done in parallel
		=> Line 12 is simple flag raising in U_int => can be done with privatization
		=> Line 13 will separate parallelization over U_int (for simplicity over U as U_int is not fixed in size) = > separate sub kernel and split the kernel to pieces?
		=> Instead of physical swapping in lines 15 and 16, use a dual-buffer approach

	
	3- A small kernel function to initialize the controller set C(x) (line 1 in Algorithm 2)
		=> May be a list of size |X_safe| with each element as list of size |U| of flags to indicate the applicability of control action u in state x?
		=> size could be reduced by using a bit-map for flags in the |U|
		=> if so, this big memory could be also used in the above algorithms?

	4- A kernel function to extract the controller inputs from the calculated invariant set (line 7+9 in Algorithm 2)

	5- The main logic of the Algorithm 2 "Max Safety Controller" will be done in the main by calling the above functions in order either:
		- for N times (requires knowing N from step 1 => breaking the parallel execution once), or
		- with additional small kernel that checks we converged (may be start with global var N and reduce it?) => also breaks the parallel execution every loop, or
		- some other idea?


	*/





/* Host-side synthesis function: This is where the monotone synthesis algorithm runs */
size_t pfacesKernel_mono_synth::runMonotoneSynthesis(void* pPackedKernel, void* pPackedParallelProgram) {
	
	// Unpacking the pointers
	const pfacesKernel_mono_synth* pKernel = (const pfacesKernel_mono_synth*)pPackedKernel;
	const pfacesParallelProgram* pParallelProgram = (const pfacesParallelProgram*)pPackedParallelProgram;
	
	// Get verbosity level for logging
	size_t beVerboseLevel = pParallelProgram->m_beVerboseLevel;
	
	// Log start of synthesis
	if (beVerboseLevel >= 1) {
		COUT_DATE_AND_TIME("[Monotone Synthesis]")
		std::cout << "Starting monotone synthesis algorithm..." << std::endl;
	}
	
	// Extract configuration data
	size_t xWidth = pParallelProgram->m_Universal_globalNDRange[0];
	size_t uWidth = pParallelProgram->m_Universal_globalNDRange[1];
	size_t ssDim = pKernel->m_spCfg->getSsDim();
	size_t isDim = pKernel->m_spCfg->getIsDim();
	
	// Access the abstraction data (computed in parallel by the OpenCL kernel)
	// This is stored in m_dataPool[0] as the XU_bag data
	char* pAbstractionData = pParallelProgram->m_dataPool[0].first;
	size_t bagSize = RW_bag::getSizeBytes({(double)ssDim, (double)isDim, (double)pKernel->m_spCfg->getMaxPosts()}, 
											pKernel->m_kernelScope == KERNEL_SCOPE_GMEM || pKernel->m_kernelScope == KERNEL_SCOPE_CMEM);
	
	// Run safe_set computation
	runSafeSetComputation(pKernel, beVerboseLevel);
	
	return 0;
}

/* a call-back function to save the controller/abstraction after the kernel finishes */
size_t pfacesKernel_mono_synth::saveData(const pfaces2DKernel& thisKernel,  const pfacesParallelProgram& thisParallelProgram, std::vector<std::shared_ptr<void>>& postExecuteParamsList) {
	
	// unboxing the passed param
	bool spIsMemoryEfficientVersion = *std::static_pointer_cast<bool>(postExecuteParamsList[0]);

	// retrieving the required values
	char*					pData					= thisParallelProgram.m_dataPool[0].first;
	std::string				outPath					= pfacesFileIO::getFileDirectoryPath(thisParallelProgram.m_spCfgReader->getConfigFilePath()) + std::string(((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getProjectName());
	size_t					beVerboseLevel			= thisParallelProgram.m_beVerboseLevel;
	std::string				strDataImplementation	= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getDataImplementationtype();
	bool					isSaveTransitions		= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->isSaveTransitions();
	bool					isSaveController		= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->isSaveController();
	size_t					xWidth					= thisParallelProgram.m_Universal_globalNDRange[0];
	size_t					uWidth					= thisParallelProgram.m_Universal_globalNDRange[1];
	size_t					ssDim					= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getSsDim();
	size_t					isDim					= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getIsDim();
	std::vector<concrete_t> ssEta					= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getSsEta();
	std::vector<concrete_t> ssErr					= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getSsErr();
	std::vector<concrete_t> isEta					= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getIsEta();
	std::vector<concrete_t> isErr					= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getIsErr();
	std::vector<concrete_t> ssLb					= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getSsLb();
	std::vector<concrete_t> ssUb					= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getSsUb();
	std::vector<concrete_t> isLb					= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getIsLb();
	std::vector<concrete_t> isUb					= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getIsUb();
	concrete_t				tau						= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getSamplingPeriod();
	std::string				targets					= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getTargetData();
	std::string				avoids					= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getAvoidData();
	std::string				safes					= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getSafeData();
	size_t					maxPostsCount			= ((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getMaxPosts();
	size_t					bagSize					= RW_bag::getSizeBytes({(double)ssDim,(double)isDim, (double)maxPostsCount}, spIsMemoryEfficientVersion);
	std::string				usedKernelName			= thisKernel.getKernelName();

	// nothing required ?
	if (!isSaveTransitions && !isSaveController) {
		return 0;
	}

	// collecting X and U width per dimension
	std::vector<symbolic_t> X_widthPerDimension;
	std::vector<symbolic_t> U_widthPerDimension;
	pfacesFlatSpace::getFlatWidthFromConcreteSpace(ssDim, ssEta, ssLb, ssUb, ssErr, X_widthPerDimension);
	pfacesFlatSpace::getFlatWidthFromConcreteSpace(isDim, isEta, isLb, isUb, isErr, U_widthPerDimension);

	// building the controller meta-data header
	StringDataDictionary metadata;
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_SAMPLING_PERIOD, std::to_string(tau)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_SS_DIMENSION, std::to_string(ssDim)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_IS_DIMENSION, std::to_string(isDim)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_SS_ETA, pfacesUtils::vector2string<concrete_t>(ssEta)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_SS_ERR, pfacesUtils::vector2string<concrete_t>(ssErr)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_IS_ETA, pfacesUtils::vector2string<concrete_t>(isEta)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_IS_ERR, pfacesUtils::vector2string<concrete_t>(isErr)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_SS_LB, pfacesUtils::vector2string<concrete_t>(ssLb)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_SS_UB, pfacesUtils::vector2string<concrete_t>(ssUb)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_IS_LB, pfacesUtils::vector2string<concrete_t>(isLb)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_IS_UB, pfacesUtils::vector2string<concrete_t>(isUb)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_SS_STEPS, pfacesUtils::vector2string<symbolic_t>(X_widthPerDimension)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_IS_STEPS, pfacesUtils::vector2string<symbolic_t>(U_widthPerDimension)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_POST_USE_ODE, (((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getUseOdePost())?"1":"0"));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_RADIUS_USE_ODE, (((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getUseOdeRadius()) ? "1" : "0"));
	for (size_t i = 0; i < ssDim; i++)
	{
		metadata.push_back(
			std::make_pair(
				pfacesUtils::strReplaceAll(
					std::string(OUT_FILE_PARAM_SS_STATE_VAR_DYNAMICS), 
					std::string(OUT_FILE_PARAM_SS_STATE_VAR_NUMBER_TOKEN)
					, std::to_string(i+1)),
				((pfacesKernel_mono_synth*)(&thisKernel))->m_spCfg->getPostDynamicsElement(i, true)
			)
		);
	}
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_X_WIDTH, std::to_string(xWidth)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_U_WIDTH, std::to_string(uWidth)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_XU_BAG_SIZE, std::to_string(bagSize)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_CONCRETE_NAME, std::string(concrete_t_cl_string)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_CONCRETE_SIZE, std::to_string(sizeof(concrete_t))));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_SYMBOLIC_NAME, std::string(symbolic_t_cl_string)));
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_SYMBOLIC_SIZE, std::to_string(sizeof(symbolic_t))));
	if(!targets.empty()) {
		metadata.push_back(std::make_pair(OUT_FILE_PARAM_TARGET_SETS, targets));
	}
	if (!avoids.empty()) {
		metadata.push_back(std::make_pair(OUT_FILE_PARAM_AVOID_SETS, avoids));
	}
	if (!safes.empty()) {
		metadata.push_back(std::make_pair(OUT_FILE_PARAM_SAFE_SETS, safes));
	}
	metadata.push_back(std::make_pair(OUT_FILE_PARAM_USED_KERNEL_NAME, usedKernelName));

	// Informing the user we are saving
	if (beVerboseLevel >= 2) {
		COUT_DATE_AND_TIME(PFACES_NAME)
		std::cout << "Saving ";
		if (isSaveTransitions) std::cout << "Abstraction";
		if (isSaveTransitions && isSaveController) std::cout << "/";
		if (isSaveController) std::cout << "Controller";
		std::cout << " data [" << strDataImplementation << "]: ";
		std::cout << std::flush;
	}
	
	// Saving the controller based on the required data type
	DataFileType file_type = pfacesDataFile::getTypeFromString(strDataImplementation);

	if (file_type == DataFileType::DATA_FILE_INVALID) {
		throw std::runtime_error("Invalid data model for saving abstraction/controller.");
	}

	pfacesRawData rawData(pData, xWidth*uWidth*bagSize);
	std::string filePath = outPath + pfacesDataFile::getTypeDefaultExtension(file_type);
	bool writeFileStatus = true;
	if (file_type == DataFileType::DATA_FILE_RAW) {
		metadata.push_back(std::make_pair(OUT_FILE_PARAM_CONTENT, std::string(OUT_FILE_CONTENT_ABSTRACTION_AND_CONTROLLER)));
		writeFileStatus = pfacesDataFile::writeData(filePath, file_type, rawData, metadata, beVerboseLevel >= 2);
	} else {

		size_t bagSizeInBits = bagSize * 8;
		size_t controllerBitOffset;
		
		if (spIsMemoryEfficientVersion) {
			controllerBitOffset = RW_BAG_CONTROLLER_BIT_INDEX_IN_FLAGS;
		} else {
			controllerBitOffset = 2 * 8 * ssDim * sizeof(concrete_t) + RW_BAG_CONTROLLER_BIT_INDEX_IN_FLAGS;
		}

		if (isSaveController) {
			StringDataDictionary controllerMetadata = metadata;
			controllerMetadata.push_back(std::make_pair(OUT_FILE_PARAM_CONTENT, std::string(OUT_FILE_CONTENT_CONTROLLER)));
			writeFileStatus = pfacesDataFile::writeData(filePath, file_type,
				rawData, controllerMetadata, beVerboseLevel >= 2, bagSizeInBits, controllerBitOffset);
		}

		if (isSaveTransitions) {
			throw std::runtime_error("Saving transitions is not yet implemented !");
		}
	}


	if (!writeFileStatus && beVerboseLevel >= 2) {
		std::cout << "failed!";
	}

	if (beVerboseLevel >= 2) {
		std::cout << std::endl;
	}

	return 0;
}


std::pair<std::vector<std::string>, std::vector<std::string>> pfacesKernel_mono_synth::getParameterList() {
	
	std::vector<std::string> params;
	std::vector<std::string> paramvals;

	///TODO: update this from parallel advisor if possible
	const bool enforceOpenCLUseDouble = false;

	/* max posts */
	params.push_back(param_max_posts);
	paramvals.push_back(std::to_string(m_spCfg->getMaxPosts()));

	/* use ode post */
	std::string def_ode_post = "";
	if (m_spCfg->getUseOdePost()) {
		def_ode_post = "#define USE_ODE_SOLVER_POST";
	}
	params.push_back(param_use_ode_post);
	paramvals.push_back(def_ode_post);

	/* use ode radius*/
	std::string def_use_radius = "";
	if (m_spCfg->getUseOdeRadius()) {
		def_use_radius = "#define USE_ODE_SOLVER_RADIUS";
	}
	params.push_back(param_use_ode_radius);
	paramvals.push_back(def_use_radius);

	/* use code only for posts */
	std::string def_codeonly_post = "";
	if (m_spCfg->getCodeOnlyPost()) {
		def_codeonly_post = "#define USE_CODE_ONLY_POST";
	}
	params.push_back(param_code_only_post);
	paramvals.push_back(def_codeonly_post);

	/* use code only for radius */
	std::string def_codeonly_radius = "";
	if (m_spCfg->getCodeOnlyRadius()) {
		def_codeonly_radius = "#define USE_CODE_ONLY_RADIUS";
	}
	params.push_back(param_code_only_radius);
	paramvals.push_back(def_codeonly_radius);

	/* concrete data type*/
	params.push_back(param_concrete_type);
	paramvals.push_back(concrete_t_cl_string);

	/* symbolic data type */
	params.push_back(param_symbolic_type);
	paramvals.push_back(symbolic_t_cl_string);

	/* flat data type */
	params.push_back(param_flat_type);
	paramvals.push_back(flat_t_cl_string);

	/* bigint_size*/
	params.push_back(param_bigint_size);
	paramvals.push_back(std::to_string(bigint_size));

	/* enforce use double for OpenCL 1.2 */
	std::string def_use_double = "";
	if (enforceOpenCLUseDouble) {
		def_use_double = "#pragma OPENCL EXTENSION cl_khr_fp64 : enable";
	}
	params.push_back(param_def_use_double);
	paramvals.push_back(def_use_double);

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

	/* X params : Err */
	std::stringstream ss_err_ss;
	pfacesUtils::PrintVector(m_spCfg->getSsErr(), ',', false, ss_err_ss);
	std::string ss_err_str = ss_err_ss.str();
	params.push_back(param_ss_err);
	paramvals.push_back(ss_err_str);

	/* U params : Eta */
	std::stringstream is_eta_ss;
	pfacesUtils::PrintVector(m_spCfg->getIsEta(), ',', false, is_eta_ss);
	std::string is_eta_str = is_eta_ss.str();
	params.push_back(param_is_eta);
	paramvals.push_back(is_eta_str);

	/* U params : LB */
	std::stringstream is_lb_ss;
	pfacesUtils::PrintVector(m_spCfg->getIsLb(), ',', false, is_lb_ss);
	std::string is_lb_str = is_lb_ss.str();
	params.push_back(param_is_lb);
	paramvals.push_back(is_lb_str);

	/* U params : UB */
	std::stringstream is_ub_ss;
	pfacesUtils::PrintVector(m_spCfg->getIsUb(), ',', false, is_ub_ss);
	std::string is_ub_str = is_ub_ss.str();
	params.push_back(param_is_ub);
	paramvals.push_back(is_ub_str);

	/* U params : Err */
	std::stringstream is_err_ss;
	pfacesUtils::PrintVector(m_spCfg->getIsErr(), ',', false, is_err_ss);
	std::string is_err_str = is_err_ss.str();
	params.push_back(param_is_err);
	paramvals.push_back(is_err_str);

	/* Sampling time (tau) */
	params.push_back(param_tau);
	paramvals.push_back(std::to_string(m_spCfg->getSamplingPeriod()));

	/* Extra include file */
	params.push_back(param_extra_include);
	std::string extra_include_file = m_spCfg->getExtraIncludeFile();
	if (extra_include_file.empty() || extra_include_file == "") {
		paramvals.push_back("");
	} else {
		std::string extra_include = std::string("#include \"") + extra_include_file + std::string("\"");
		paramvals.push_back(extra_include);
	}
		
	/* Post dynamics : initial code */
	params.push_back(param_postdynamics_initcode);
	paramvals.push_back(m_spCfg->getPostDynamicsInitCodeOpenCL());

	/* Post dynamics : main */
	params.push_back(param_postdynamics);
	paramvals.push_back(m_spCfg->getPostDynamicsOpenCL());

	/* Post dynamics : finish code */
	params.push_back(param_postdynamics_finishcode);
	paramvals.push_back(m_spCfg->getPostDynamicsFinishCodeOpenCL());

	/* radius dynamics : initial code */
	params.push_back(param_radiusdynamics_initcode);
	paramvals.push_back(m_spCfg->getGrowthDynamicsInitCodeOpenCL());

	/* radius dynamics : main */
	params.push_back(param_radiusdynamics);
	paramvals.push_back(m_spCfg->getGrowthDynamicsOpenCL());

	/* radius dynamics : finish code */
	params.push_back(param_radiusdynamics_finishcode);
	paramvals.push_back(m_spCfg->getGrowthDynamicsFinishCodeOpenCL());

	/* Specification: target set */
	if (m_spCfg->isHasTarget()) {
		params.push_back(param_has_target);
		paramvals.push_back("#define HAS_TARGET");

		params.push_back(param_target_count);
		paramvals.push_back(std::to_string(m_spCfg->getTargetCount()));

		params.push_back(param_target_data);
		paramvals.push_back(m_spCfg->getTargetData());
	} else {
		params.push_back(param_has_target);
		paramvals.push_back("");

		params.push_back(param_target_count);
		paramvals.push_back(std::to_string(0));

		params.push_back(param_target_data);
		paramvals.push_back("");
	}

	/* Specification: safe set */
	if (m_spCfg->isHasSafe()) {
		params.push_back(param_has_safe);
		paramvals.push_back("#define HAS_SAFE");

		params.push_back(param_safe_count);
		paramvals.push_back(std::to_string(m_spCfg->getSafeCount()));

		params.push_back(param_safe_data);
		paramvals.push_back(m_spCfg->getSafeData());
	} else {
		params.push_back(param_has_safe);
		paramvals.push_back("");

		params.push_back(param_safe_count);
		paramvals.push_back(std::to_string(0));

		params.push_back(param_safe_data);
		paramvals.push_back("");
	}

	/* Specification: obstacles set */
	if (m_spCfg->isHasAvoid()) {
		params.push_back(param_has_avoid);
		paramvals.push_back("#define HAS_AVOID");

		params.push_back(param_avoid_count);
		paramvals.push_back(std::to_string(m_spCfg->getAvoidCount()));

		params.push_back(param_avoid_data);
		paramvals.push_back(m_spCfg->getAvoidData());
	} else {
		params.push_back(param_has_avoid);
		paramvals.push_back("");

		params.push_back(param_avoid_count);
		paramvals.push_back(std::to_string(0));

		params.push_back(param_avoid_data);
		paramvals.push_back("");
	}

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
	setDimensions(ssDim, isDim);

	// Convert concrete space into flat space for X
	x_flat_width = pfacesFlatSpace::getFlatWidthFromConcreteSpace(
		m_spCfg->getSsDim(), m_spCfg->getSsEta(), 
		m_spCfg->getSsLb(), m_spCfg->getSsUb(), 
		m_spCfg->getSsErr(), X_widthPerDimension);

	// Convert concrete space into flat space for U
	u_flat_width = pfacesFlatSpace::getFlatWidthFromConcreteSpace(
		m_spCfg->getIsDim(), m_spCfg->getIsEta(), 
		m_spCfg->getIsLb(), m_spCfg->getIsUb(), 
		m_spCfg->getIsErr(), U_widthPerDimension);

	// Estimate the size of the bigint needed
	bigint_size = pfacesBigInt::getBlkCount(pfacesBigInt::Max(x_flat_width, u_flat_width));
	bigint_size_bytes = bigint_size*flat_t_cl_numbytes;;


	// Checking if we are using memory efficient version
	bool isMemoryEfficientVersion = false;
	if (m_kernelScope == KERNEL_SCOPE_GMEM || m_kernelScope == KERNEL_SCOPE_CMEM)
		isMemoryEfficientVersion = true;

	// Loading the memory fingerprint of the abstract function
	auto abstractFunctionArgs = pfacesKernelFunctionArguments::loadFromFile(
		spLaunchState->getKernelPackPath() + "mono_synth.mem",	/* memory config file with the function memory fingerprint */
		KERNEL_MONO_SYNTH_ASTRACT_FUNC_NAME,  					/* name of the function to add */
		{ "XU_bag_process", "XU_bag_local", "RO_bag" },			/* list of the names of its args */
		false													/* do not save memory render files */
	);
	abstractFunctionArgs.m_baseTypeSize = {
		RW_bag::getSizeBytes({(double)ssDim,(double)isDim,(double)m_spCfg->getMaxPosts()}, isMemoryEfficientVersion),
		RW_bag::getSizeBytes({(double)ssDim,(double)isDim,(double)m_spCfg->getMaxPosts()}, isMemoryEfficientVersion),
		RO_bag::getSizeBytes({(double)bigint_size_bytes})
	};
	abstractFunctionArgs.m_baseTypeMultiple = {1, 1, 1};
	pfacesKernelFunction abstractFunction(KERNEL_MONO_SYNTH_ASTRACT_FUNC_NAME, abstractFunctionArgs);

	// adding the function to the kernel
	addKernelFunction(abstractFunction);
	
	// updating the list of params
	auto params_and_vals = getParameterList();
	updateParameters(params_and_vals.first, params_and_vals.second);


	
}

/* providing implementation of the driver of the kernel */
void pfacesKernel_mono_synth::configureParallelProgram(pfacesParallelProgram& parallelProgram) {

	// A parallel advisor used for task scheduling
	pfacesParallelAdvisor parallelAdvisor(parallelProgram.getMachine(), parallelProgram.getTargetDevicesIndicies());
	size_t beVerboseLevel = parallelProgram.m_beVerboseLevel;

	///TODO: handle bigger flat space = handle the bigger problem op big-job splitting
	if (x_flat_width.getBlkCount() > 1 || u_flat_width.getBlkCount() > 1) {
		throw std::runtime_error("Flat space size is bigger than what is supported with beta version !");
	}

	// X and U space cardinality
	size_t problem_x_width = *((flat_t_base_t*)x_flat_width.getBlkArray());
	size_t problem_u_width = *((flat_t_base_t*)u_flat_width.getBlkArray());

	// Print Universal XU-space information
	if (beVerboseLevel >= 2) {
		std::cout << "Universal X-space has " << problem_x_width << " symbols: ";
		pfacesUtils::PrintVector(X_widthPerDimension, 'x');

		std::cout << "Universal U-space has " << problem_u_width << " symbols: ";
		pfacesUtils::PrintVector(U_widthPerDimension, 'x');
	}
	
	// Distributing the jobs for the three main functions: abstract, 
	// synthesize and check, all for any kernel scope
	cl::NDRange ndUniversalRangeXU(problem_x_width, problem_u_width);
	cl::NDRange ndUniversalOffsetXU = cl::NullRange; /* we are currently doing it as one chunk */
	auto procRangeAndOffset = parallelAdvisor.getProcessNDRangeAndOffset(ndUniversalRangeXU);
	cl::NDRange& ndProcessRangeXU = procRangeAndOffset.first;
	cl::NDRange& ndProcessOffsetXU = procRangeAndOffset.second;
	bool enforceStrictJobRange = false;

	if (beVerboseLevel >= 2) {
		std::cout << "Scheduling abstraction/synthesis jobs on the targeted devices ... " << std::endl;
	}
	if(m_kernelScope == KERNEL_SCOPE_GPU || m_kernelScope == KERNEL_SCOPE_GMEM) {

		if (beVerboseLevel >= 2) {
			pfacesTerminal::showInfoMessage("We activate NON-STRICT mode for GPU kernels to mach the GPU PEs.");
		}

		perDevAbstractionJobs = parallelAdvisor.distributeJob(
			*this, KERNEL_MONO_SYNTH_ASTRACT_FUNC_IDX, ndProcessRangeXU, ndProcessOffsetXU,
			parallelProgram.m_isFixedJobDistribution, 
			parallelProgram.m_fixedJobDistribution, true, false, false);
	} else {		

		perDevAbstractionJobs = parallelAdvisor.distributeJob(
			*this, KERNEL_MONO_SYNTH_ASTRACT_FUNC_IDX, ndProcessRangeXU, ndProcessOffsetXU,
			parallelProgram.m_isFixedJobDistribution, 
			parallelProgram.m_fixedJobDistribution, 
			(!parallelProgram.m_enableNonStrictMode) | (enforceStrictJobRange), false, false);
	}

	// check if the task scheduler changed the sizes case we are using a non-strict
	// scheduling and it founded a better scheduling for us
	if (ndProcessRangeXU[0] != problem_x_width || ndProcessRangeXU[1] != problem_u_width) {

		std::stringstream ss_msg;
		ss_msg << "The space-size is modified so that devices will have no unused "
			   << "cores or to match the needs of local memory of the GPU-based kernel!" << std::endl
			   << "New X-space size: " << ndProcessRangeXU[0] << " symbols" << std::endl
			   << "New U-space size: " << ndProcessRangeXU[1] << " symbols";
		pfacesTerminal::showWarnMessage(ss_msg.str());

		x_flat_width = problem_x_width;
		u_flat_width = problem_u_width;
	}

	// it is important to use (parallelAdvisor.getNumberOfInstances2D) and not to calculate
	// it yourself this function takes care if we are using MPI by selecting the number of
	// instances needed for the process
	size_t bagSize = RW_bag::getSizeBytes({
		(double)m_spCfg->getSsDim(),
		(double)m_spCfg->getIsDim()
	}, m_kernelScope == KERNEL_SCOPE_GMEM || m_kernelScope == KERNEL_SCOPE_CMEM);
	size_t numInstances = pfacesUtils::oclGetRangeVolume(ndProcessRangeXU);
	if (beVerboseLevel >= 2) {
		std::cout << "Total number of (PEs|Mem-bags) to be allocated for the job: " << numInstances << std::endl;
		std::cout << "Memory bag size: " << bagSize << " bytes." << std::endl;
	}

	// print the task-scheduling report
	if (beVerboseLevel >= 2){
		parallelAdvisor.printTaskSchedulingReport(
			parallelProgram.getMachine(),
			{ KERNEL_MONO_SYNTH_ASTRACT_FUNC_NAME},
			{ perDevAbstractionJobs },
			ndUniversalRangeXU[0]
		);
	}

	// Allocating the memory used for abstraction/synthesis
	std::vector<std::pair<char*, size_t>> abstractionDataPool;
	pFacesMemoryAllocationReport memReport;
	
	memReport = allocateMemory(abstractionDataPool, 
		parallelProgram.getMachine(), parallelProgram.getTargetDevicesIndicies(), 
		numInstances, false);

	if (beVerboseLevel >= 2) {
		memReport.PrintReport();
	}	

	// preparing sub-buffer info for the all functions - at arg index 1 ! 
	// remember arg-0 and arg-2 are not buffered
	if (parallelProgram.countTargetDevices() > 1) {
		perDevAbstractionJob_XUBAG_LOCAL_SubBuffers = getSubBuffers(perDevAbstractionJobs, KERNEL_MONO_SYNTH_ASTRACT_FUNC_IDX, KERNEL_MONO_SYNTH_ASTRACT_FUNCARG_XUBAG_LOCAL, memReport.bufferFinalSize[0], problem_u_width);

		// printing the sub-buffering report
		if (beVerboseLevel >= 2) {
			parallelAdvisor.printSubBufferingReport(
				{ KERNEL_MONO_SYNTH_ASTRACT_FUNC_NAME },
				{ KERNEL_MONO_SYNTH_ASTRACT_FUNC_IDX },
				{ KERNEL_MONO_SYNTH_ASTRACT_FUNCARG_XUBAG_LOCAL },
				{ KERNEL_MONO_SYNTH_ASTRACT_FUNC_NUM_ARGS },
				{ perDevAbstractionJob_XUBAG_LOCAL_SubBuffers }
			);
		}
	}


	// Setting the job-chunk base
	flat_t grid_base_x = 0;
	flat_t grid_base_u = 0;
	if (beVerboseLevel >= 2) {
		std::cout << "Running a 2D job in one chunk with the base: (" << grid_base_x << ", " << grid_base_u << ")" << std::endl;
	}

	// First device in the list will be used for memory access
	const cl::Device&  dataAccessDevice = parallelProgram.getTargetDevices()[0];

	// initializing/configuring the Data-oriented jobs
	readAllDataJob = std::make_shared<pfacesDeviceReadJob>(dataAccessDevice);
	readAllDataJob->setKernelFunctionIdx(KERNEL_MONO_SYNTH_ASTRACT_FUNC_IDX, KERNEL_MONO_SYNTH_ASTRACT_FUNC_NUM_ARGS);

	writeAllDataJob = std::make_shared<pfacesDeviceWriteJob>(dataAccessDevice);
	writeAllDataJob->setKernelFunctionIdx(KERNEL_MONO_SYNTH_ASTRACT_FUNC_IDX, KERNEL_MONO_SYNTH_ASTRACT_FUNC_NUM_ARGS);

	
	// initialize/configuring the instructions
	instr_BlockingSyncPoint->setAsBlockingSyncPoint();
	instr_LogOn->setAsLogOn();
	instr_LogOff->setAsLogOff();
	instr_readAllData->setAsReadAllDeviceData(readAllDataJob);
	instr_writeAllData->setAsWriteAllDeviceData(writeAllDataJob);

	// if not using the direct access to host memory, we add this instruction
	// to write the data from the host memory to the device memory and followed by
	// a barrier to sync among all device threads
	if (!parallelProgram.m_useHostMemory) {
		instructionList.push_back(instr_writeAllData);

		if (parallelProgram.countTargetDevices() > 1) {
			instructionList.push_back(instr_BlockingSyncPoint);
		}
	}

	// Turn Logs on ?
	if (parallelProgram.m_oclDebug) {
		instructionList.push_back(instr_LogOn);
	}

	// The first main task: ABSTRACTION
	for (size_t i = 0; i < perDevAbstractionJobs.size(); i++) {
		std::shared_ptr<pfacesInstruction> tmpExecuteInstr = std::make_shared<pfacesInstruction>();

		if (perDevAbstractionJob_XUBAG_LOCAL_SubBuffers.size() != 0) {
			perDevAbstractionJobs[i]->setSubBufferBase(
				KERNEL_MONO_SYNTH_ASTRACT_FUNCARG_XUBAG_LOCAL, 
				perDevAbstractionJob_XUBAG_LOCAL_SubBuffers[i].first);
			
			perDevAbstractionJobs[i]->setSubBufferSize(
				KERNEL_MONO_SYNTH_ASTRACT_FUNCARG_XUBAG_LOCAL, 
				perDevAbstractionJob_XUBAG_LOCAL_SubBuffers[i].second);
		}

		tmpExecuteInstr->setAsDeviceExecute(perDevAbstractionJobs[i]);
		instructionList.push_back(tmpExecuteInstr);
	}

	// A Barrier to force all devices to finish.
	if (parallelProgram.countTargetDevices() > 1) {
		instructionList.push_back(instr_BlockingSyncPoint);
	}

	// Turn Log off if it was turned on !
	if (parallelProgram.m_oclDebug) {
		instructionList.push_back(instr_LogOff);
	}

	// Notify the user that abstraction is complete
	if (beVerboseLevel >= 2) {
		instr_MsgAbsComplete->setAsMessage("Abstraction task is complete.");
		instructionList.push_back(instr_MsgAbsComplete);
	}

	// Read results from device to host memory
	instructionList.push_back(instr_readAllData);
	instructionList.push_back(instr_BlockingSyncPoint);

	// ============================================================================
	// HOST-SIDE SYNTHESIS: After parallel abstraction is complete, run monotone synthesis
	// ============================================================================
	// This is where we run the serial synthesis algorithm on the host CPU.
	// The abstraction data computed by the OpenCL kernel is now available in host memory.
	//
	// How it works:
	// 1. The OpenCL kernel computed Post(x,u) for all (x,u) pairs in parallel
	// 2. Data was read back from device to host (above)
	// 3. Now we run the synthesis algorithm serially on the host to:
	//    - Partition the input space based on monotonicity
	//    - Compute invariant sets for each partition
	//    - Extract the maximal safe controller
	//
	instr_runSynthesis->setAsHostFunction(runMonotoneSynthesis, "runMonotoneSynthesis");
	instructionList.push_back(instr_runSynthesis);

	// Add another sync point after synthesis
	instructionList.push_back(instr_BlockingSyncPoint);

	/* writing to RO_bag :: the bases */
	///TODO: Make this as as WriteFromHost Job
	size_t roBagSize = RO_bag::getSizeBytes({ (double)bigint_size_bytes });
	std::vector<char> twoBaseArray(roBagSize);
	RO_bag::fillData(twoBaseArray, grid_base_x, grid_base_u, bigint_size_bytes);
	memoryWriteArgument(abstractionDataPool, 
		KERNEL_MONO_SYNTH_ASTRACT_FUNC_IDX, KERNEL_MONO_SYNTH_ASTRACT_FUNCARG_ROBAG, 
		twoBaseArray.data(), roBagSize, 0);

	// setting the execute ranges
	parallelProgram.m_Universal_globalNDRange = ndUniversalRangeXU;
	parallelProgram.m_Universal_offsetNDRange = ndUniversalOffsetXU;
	parallelProgram.m_Process_globalNDRange = ndProcessRangeXU;
	parallelProgram.m_Process_offsetNDRange = ndProcessOffsetXU;

	parallelProgram.m_dataPool = abstractionDataPool;
	parallelProgram.m_spInstructionList = instructionList;

	// the sizes of flat-type
	parallelProgram.m_bigint_size = bigint_size;
	parallelProgram.m_bigint_size_bytes = bigint_size_bytes;

	// setting a postExecute-function for collecting and saving the controller
	postExecuteParams.clear();

	// is memory efficient version ?
	if (m_kernelScope == KERNEL_SCOPE_GMEM || m_kernelScope == KERNEL_SCOPE_CMEM){
		std::shared_ptr<bool> spIsMemEfficient = std::make_shared<bool>(true);
		std::shared_ptr<void> casted = std::static_pointer_cast<void>(spIsMemEfficient);
		postExecuteParams.push_back(casted);
	} else {
		std::shared_ptr<bool> spIsMemEfficient = std::make_shared<bool>(false);
		std::shared_ptr<void> casted = std::static_pointer_cast<void>(spIsMemEfficient);
		postExecuteParams.push_back(casted);
	}

	registerPostExecuteFunction(pfacesKernel_mono_synth::saveData, "Saving results", postExecuteParams);

}

/* not providing implementation of the virtual method: configureTuneParallelProgram*/
void pfacesKernel_mono_synth::configureTuneParallelProgram(pfacesParallelProgram&, size_t) {
}

} // namespace mono_synth


PFACES_REGISTER_LOADABLE_KERNEL(mono_synth::pfacesKernel_mono_synth)



