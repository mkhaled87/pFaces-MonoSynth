/**
 * OpenCL Kernel for Pre-computing Worst-Case State Transitions
 * Each work item computes the next state for one grid cell in parallel
 */

// Vehicle dynamics parameters (3D)
///TODO: put these in the config files
#define T_MAX 1200.0
#define T_BRAKE_MIN -1800.0
#define T_BRAKE_MAX -2400.0
#define M_MIN 2000.0
#define M_MAX 2500.0
#define R_W_MIN 0.30
#define R_W_MAX 0.35
#define ALPHA_MIN 300.0
#define ALPHA_MAX 350.0
#define BETA_MIN 0.10
#define BETA_MAX 0.25
#define GAMMA_MIN 0.30
#define GAMMA_MAX 0.65
#define H_MIN_3D 0.0
#define H_MAX_3D 80.0
#define V_MIN_3D 0.0
#define V_MAX_3D 20.0
#define H_RES_3D 0.8
#define V_RES_3D 0.4
#define X_PRIORITY_0 0
#define X_PRIORITY_1 1
#define X_PRIORITY_2 0

#define SS_MIN {H_MIN_3D, V_MIN_3D, V_MIN_3D}
#define SS_MAX {H_MAX_3D, V_MAX_3D, V_MAX_3D}
#define SS_RES {H_RES_3D, V_RES_3D, V_RES_3D}
#define SS_PRIORITY {X_PRIORITY_0, X_PRIORITY_1, X_PRIORITY_2}


// ODE solver parameters
#define NUM_STEPS 1000
#define SAMPLING_TIME 0.4
#define SS_DIM 3


///TODO: get this from the user
/**
 * ODE right-hand side: computes derivatives for position and velocity
 */
void f(double vel, double a, double b, double c, bool is_lead, double* dpos, double* dvel) {
    double dvdt = a + b * vel + c * vel * vel;
    if (vel <= 0.0 && dvdt < 0.0) dvdt = 0.0;
    if (is_lead && vel >= V_MAX_3D && dvdt > 0.0) dvdt = 0.0;
    *dpos = vel;
    *dvel = dvdt;
}

/**
 * RK4 ODE solver for vehicle dynamics
 */
void solveODE(double v0, double a, double b, double c, bool is_lead, double dt, 
              double* pos, double* vel) {
    const double step_size = dt / NUM_STEPS;
    *pos = 0.0;
    *vel = v0;
    
    for (int i = 0; i < NUM_STEPS; ++i) {
        double k1_pos, k1_vel, k2_pos, k2_vel, k3_pos, k3_vel, k4_pos, k4_vel;
        
        f(*vel, a, b, c, is_lead, &k1_pos, &k1_vel);
        f(*vel + step_size/2 * k1_vel, a, b, c, is_lead, &k2_pos, &k2_vel);
        f(*vel + step_size/2 * k2_vel, a, b, c, is_lead, &k3_pos, &k3_vel);
        f(*vel + step_size * k3_vel, a, b, c, is_lead, &k4_pos, &k4_vel);
        
        *pos += step_size/6 * (k1_pos + 2*k2_pos + 2*k3_pos + k4_pos);
        *vel += step_size/6 * (k1_vel + 2*k2_vel + 2*k3_vel + k4_vel);
    }
}


///TODO: maybe also from the user?
/**
 * Solve 3D vehicle dynamics with worst-case parameters
 */
void solve3DDynamicsWorstCase(double* x, double dt, double* x_plus) {

    double u_worst = T_BRAKE_MIN;
    double w_worst = T_BRAKE_MAX;

    // Choose worst-case parameters for ego vehicle (braking)
    double R_w_ego = (u_worst > 0) ? R_W_MIN : R_W_MAX;
    double M_ego = ((u_worst / R_w_ego - ALPHA_MIN) > 0) ? M_MIN : M_MAX;
    double a = (1.0 / M_ego) * (u_worst / R_w_ego - ALPHA_MIN);
    double b = -(1.0 / M_MAX) * BETA_MIN;
    double c = -(1.0 / M_MAX) * GAMMA_MIN;
    
    // Choose worst-case parameters for lead vehicle
    double R_w_L = (w_worst > 0) ? R_W_MAX : R_W_MIN;
    double M_L = ((w_worst / R_w_L - ALPHA_MAX) > 0) ? M_MAX : M_MIN;
    double a_L = (1.0 / M_L) * (w_worst / R_w_L - ALPHA_MAX);
    double b_L = -(1.0 / M_MIN) * BETA_MAX;
    double c_L = -(1.0 / M_MIN) * GAMMA_MAX;
    
    // Solve vehicle dynamics using RK4
    double pos_plus, vel_plus, posL_plus, velL_plus;
    solveODE(x[1], a, b, c, false, dt, &pos_plus, &vel_plus);
    solveODE(x[2], a_L, b_L, c_L, true, dt, &posL_plus, &velL_plus);
    
    x_plus[0] = x[0] + posL_plus - pos_plus;
    x_plus[1] = fmax((double)V_MIN_3D, fmin((double)vel_plus, (double)V_MAX_3D));
    x_plus[2] = fmax((double)V_MIN_3D, fmin((double)velL_plus, (double)V_MAX_3D));
}


///TODO: generalize this to n-dim
/**
 * Convert flattened index to multi-dimensional indices
 */
void unflattenIndex(int flat_idx, int state_dim, const unsigned int* x_numCells, int* idx) {
    if (state_dim == 2) {
        idx[1] = (flat_idx % x_numCells[1]) + 1;
        idx[0] = (flat_idx / x_numCells[1]) + 1;
    } else {  // 3D
        idx[2] = (flat_idx % x_numCells[2]) + 1;
        int temp = flat_idx / x_numCells[2];
        idx[1] = (temp % x_numCells[1]) + 1;
        idx[0] = (temp / x_numCells[1]) + 1;
    }
}

///TODO: generalize this to n-dim
/**
 * Convert indices to physical state values using priority directions
 */
void getPriorityStateAtIdx(int* idx, int state_dim,
                           const double* x_range_min,
                           const double* x_range_max,
                           const double* x_res,
                           const int* x_priority,
                           double* val) {

    for (int i = 0; i < state_dim; ++i) {
        if (x_priority[i] == 1) {
            val[i] = x_range_min[i] + (idx[i] - 1) * x_res[i];
        } else {
            val[i] = x_range_max[i] - (idx[i] - 1) * x_res[i];
        }
    }
}

///TODO: generalize this to n-dim
/**
 * Convert physical state values to indices
 */
void getStateIdx(double* val, int state_dim,
                 const double* x_range_min,
                 const double* x_range_max,
                 const double* x_res,
                 const int* x_priority,
                 const unsigned int* x_numCells,
                 unsigned int* idx) {

    for (int i = 0; i < state_dim; ++i) {
        double val_clamped = fmax((double)x_range_min[i], fmin((double)val[i], (double)x_range_max[i]));
        
        if (x_priority[i] == 1) {
            idx[i] = (int)floor((val_clamped - x_range_min[i]) / x_res[i]) + 1;
        } else {
            idx[i] = (int)floor((x_range_max[i] - val_clamped) / x_res[i]) + 1;
        }
        // Clamp to valid range
        if (idx[i] < 1) idx[i] = 1;
        if (idx[i] > x_numCells[i]) idx[i] = x_numCells[i];
    }
}

/**
 * Main kernel: Compute worst-case next state for each grid cell
 * 
 * @param next_state_table: Output buffer (total_states * 3 integers)
 */
__kernel void precompute_transitions(
    __global unsigned int* next_state_table                      // Size: |X| 
) {
    int flat_idx = get_global_id(0);

    const double x_range_min[SS_DIM] = SS_MIN;
    const double x_range_max[SS_DIM] = SS_MAX;
    const double x_res[SS_DIM] = SS_RES;
    const int x_priority[SS_DIM] = SS_PRIORITY;    
    
    unsigned int x_numCells[SS_DIM];
    for (int i = 0; i < SS_DIM; ++i) {
        x_numCells[i] = ceil((x_range_max[i] - x_range_min[i]) / x_res[i]) + 1;
    }
        
    // Un-flatten index to get multi-dimensional indices
    int x_idx[SS_DIM];
    unflattenIndex(flat_idx, SS_DIM, x_numCells, x_idx);
    
    // Convert indices to physical state values
    double x_val[SS_DIM];
    getPriorityStateAtIdx(x_idx, SS_DIM, x_range_min, x_range_max, x_res, x_priority, x_val);
    
    // Compute worst-case next state
    double x_plus_val[SS_DIM];

    // 3D worst-case: minimum input, maximum disturbance
    solve3DDynamicsWorstCase(x_val, SAMPLING_TIME, x_plus_val);

    // Convert next state values to indices
    unsigned int x_plus_idx[SS_DIM];
    getStateIdx(x_plus_val, SS_DIM, x_range_min, x_range_max, x_res, x_priority, x_numCells, x_plus_idx);
    
    // Store result in output buffer
    for (int j = 0; j < SS_DIM; ++j) {
        next_state_table[flat_idx * SS_DIM + j] = x_plus_idx[j];
    }
}