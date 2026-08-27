/* Exact construction of the threshold table representing down(B). */

__kernel void threshold_clear(__global uint* threshold_table) {
    const ulong key = (ulong)get_global_id(0);
    if (key < (ulong)@@THRESHOLD_TABLE_SIZE@@) threshold_table[key] = 0;
}

__kernel void threshold_scatter(
    __global const uint* basis,
    __global const ulong* basis_size,
    __global uint* threshold_table
) {
    const ulong index = (ulong)get_global_id(0);
    if (index >= basis_size[0]) return;
    const int widths[@@STATE_DIM@@] = @@GRID_SIZES_ARRAY@@;
    ulong key = 0;
    ulong stride = 1;
    for (int d = 0; d < @@STATE_DIM@@; ++d) {
        if (d == @@THRESHOLD_D_STAR@@) continue;
        key += (ulong)(basis[index * (ulong)@@STATE_DIM@@ + (ulong)d] - 1u) * stride;
        stride *= (ulong)widths[d];
    }
    atomic_max((volatile __global uint*)&threshold_table[key],
               basis[index * (ulong)@@STATE_DIM@@ + @@THRESHOLD_D_STAR@@]);
}

__kernel void threshold_prefix(
    __global uint* threshold_table,
    __global const ulong* sweep_params
) {
    const ulong fiber = (ulong)get_global_id(0);
    const ulong stride = sweep_params[0];
    const ulong width = sweep_params[1];
    const ulong table_size = (ulong)@@THRESHOLD_TABLE_SIZE@@;
    const ulong fibers = table_size / width;
    if (fiber >= fibers) return;

    const ulong outer = fiber / stride;
    const ulong inner = fiber % stride;
    const ulong base = outer * stride * width + inner;
    for (ulong coordinate = width - 1; coordinate > 0; --coordinate) {
        const ulong lower = base + (coordinate - 1) * stride;
        const ulong upper = lower + stride;
        threshold_table[lower] = max(threshold_table[lower], threshold_table[upper]);
    }
}

/* Exact O(1) update for deleting one maximal state b from a lower set:
 * only tau(pi(b)) changes, and it decreases by one.  ULONG_MAX is a no-op
 * sentinel used when a speculative GPU pass contains no unsafe generator. */
__kernel void threshold_decrement(
    __global uint* threshold_table,
    __global const ulong* threshold_key
) {
    if (get_global_id(0) != 0) return;
    const ulong key = threshold_key[0];
    if (key >= (ulong)@@THRESHOLD_TABLE_SIZE@@) return;
    const uint height = threshold_table[key];
    if (height > 0u) threshold_table[key] = height - 1u;
}
