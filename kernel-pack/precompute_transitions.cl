/*
* precompute_transitions.cl
*
*  date    : 20.01.2026
*  about   : Generic N-D worst-case transition kernel for monotone synthesis.
* ***********************************************************************
*/

/**
 * Convention: 1-based internal monotone coordinates with dim 0 fastest.
 *   For priority 0 (max-good): idx = 1 maps to x_max, idx = N maps to x_min.
 *   For priority 1 (min-good): idx = 1 maps to x_min, idx = N maps to x_max.
 *   flat = (idx[0]-1) + (idx[1]-1)*N0 + ...  (column-major, dim 0 fastest)
 *
 * This matches the pFaces basis iteration, which always expands toward
 * smaller internal indices.
 */

// ============================================================================
// USER DYNAMICS (included from external file)
// ============================================================================

@@USER_DYNAMICS_CODE@@

// ============================================================================
// GENERIC ODE SOLVER
// ============================================================================

inline void rk4_step(const float* x, const float* u, const float* w,
                     float dt, float* x_plus, const float* rt_params) {
    const float h = dt / @@ODE_STEPS@@;
    float x_curr[@@STATE_DIM@@];
    float k1[@@STATE_DIM@@], k2[@@STATE_DIM@@], k3[@@STATE_DIM@@], k4[@@STATE_DIM@@];
    float x_temp[@@STATE_DIM@@];
    
    for (int i = 0; i < @@STATE_DIM@@; ++i) x_curr[i] = x[i];
    
    for (int step = 0; step < @@ODE_STEPS@@; ++step) {
        ode_rhs(x_curr, u, w, k1, rt_params);
        
        for (int i = 0; i < @@STATE_DIM@@; ++i) 
            x_temp[i] = x_curr[i] + h * 0.5f * k1[i];
        ode_rhs(x_temp, u, w, k2, rt_params);
        
        for (int i = 0; i < @@STATE_DIM@@; ++i)
            x_temp[i] = x_curr[i] + h * 0.5f * k2[i];
        ode_rhs(x_temp, u, w, k3, rt_params);
        
        for (int i = 0; i < @@STATE_DIM@@; ++i)
            x_temp[i] = x_curr[i] + h * k3[i];
        ode_rhs(x_temp, u, w, k4, rt_params);
        
        for (int i = 0; i < @@STATE_DIM@@; ++i)
            x_curr[i] += (h / 6.0f) * (k1[i] + 2.0f*k2[i] + 2.0f*k3[i] + k4[i]);
    }
    
    for (int i = 0; i < @@STATE_DIM@@; ++i) {
        x_plus[i] = x_curr[i];
    }
    apply_state_constraints(x_plus);
}

// ============================================================================
// INDEX MAPPING: 1-based internal monotone coordinates, dim 0 fastest
// ============================================================================

inline void unflatten_index(int flat_idx, const unsigned int* dims, int* idx) {
    for (int i = 0; i < @@STATE_DIM@@; ++i) {
        idx[i] = (flat_idx % dims[i]) + 1;
        flat_idx /= dims[i];
    }
}

inline void idx_to_state(const int* idx,
                         const float* x_max,
                         const float* x_min,
                         const int* x_priority,
                         const float* x_res,
                         float* x) {
    for (int i = 0; i < @@STATE_DIM@@; ++i) {
        if (x_priority[i] == 0) {
            x[i] = x_max[i] - (idx[i] - 1) * x_res[i];
        } else {
            x[i] = x_min[i] + (idx[i] - 1) * x_res[i];
        }
    }
}

inline bool state_to_idx(const float* x,
                         const float* x_min,
                         const float* x_max,
                         const float* x_res,
                         const int* x_priority,
                         const unsigned int* x_numCells,
                         int* idx) {
    const float tol = 1e-2f;
    
    for (int i = 0; i < @@STATE_DIM@@; ++i) {
        if (x[i] < x_min[i] - tol || x[i] > x_max[i] + tol) {
            for (int j = 0; j < @@STATE_DIM@@; ++j) idx[j] = -1;
            return false;
        }
        
        float v = fmax(x_min[i], fmin(x[i], x_max[i]));
        float q;
        if (x_priority[i] == 0) {
            q = (x_max[i] - v) / x_res[i];
        } else {
            q = (v - x_min[i]) / x_res[i];
        }
        idx[i] = min((int)ceil(q - 1e-5f) + 1, (int)x_numCells[i]);
    }
    return true;
}

// ============================================================================
// MAIN KERNEL
// ============================================================================

__kernel void precompute_transitions(
    __global unsigned int* next_state_table,
    __global const float* runtime_params
) {
    int gid = get_global_id(0);
    
    float rt_params[4];
    rt_params[0] = runtime_params[0];
    rt_params[1] = runtime_params[1];
    rt_params[2] = runtime_params[2];
    rt_params[3] = runtime_params[3];
    
    const float x_min[@@STATE_DIM@@] = @@X_MIN_ARRAY@@;
    const float x_max[@@STATE_DIM@@] = @@X_MAX_ARRAY@@;
    const float x_res[@@STATE_DIM@@] = @@X_RES_ARRAY@@;
    const int x_priority[@@STATE_DIM@@] = @@X_PRIORITY_ARRAY@@;
    
    unsigned int x_numCells[@@STATE_DIM@@];
    for (int i = 0; i < @@STATE_DIM@@; ++i) {
        x_numCells[i] = (unsigned int)ceil((x_max[i] - x_min[i]) / x_res[i]) + 1;
    }
    
    int x_idx[@@STATE_DIM@@];
    unflatten_index(gid, x_numCells, x_idx);
    
    float x[@@STATE_DIM@@];
    idx_to_state(x_idx, x_max, x_min, x_priority, x_res, x);
    
    float u[@@INPUT_DIM@@], w[@@DISTURB_DIM@@];
    get_worst_case_inputs(x, u, w);
    
    float x_plus[@@STATE_DIM@@];
    rk4_step(x, u, w, @@SAMPLING_TIME@@, x_plus, rt_params);
    
    int x_plus_idx[@@STATE_DIM@@];
    for (int i = 0; i < @@STATE_DIM@@; ++i) x_plus_idx[i] = -1;
    state_to_idx(x_plus, x_min, x_max, x_res, x_priority, x_numCells, x_plus_idx);
    
    int base = gid * @@STATE_DIM@@;
    for (int i = 0; i < @@STATE_DIM@@; ++i) {
        next_state_table[base + i] = (unsigned int)x_plus_idx[i];
    }
}
