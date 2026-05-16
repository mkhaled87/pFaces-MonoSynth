/*
 * bitmap_gfp_prefix.cl
 *
 * In-place lower-closure sweep for a bit-packed bitmap along one dimension.
 * Each work item owns one fiber and scans from high coordinate to low.
 */

inline uint bitmap_prefix_get(__global const uint* bitmap, size_t idx) {
    return (bitmap[idx >> 5] >> (uint)(idx & (size_t)31)) & 1u;
}

__kernel void bitmap_gfp_prefix(
    __global uint* bitmap,
    __global const int* sweep_params
) {
    const size_t stride = (size_t)sweep_params[0];
    const size_t n_dim = (size_t)sweep_params[1];
    const size_t fid = get_global_id(0);

    const size_t inner = fid % stride;
    const size_t outer = fid / stride;
    const size_t block = stride * n_dim;
    const size_t base = outer * block + inner;

    for (int c = (int)n_dim - 2; c >= 0; --c) {
        const size_t idx_lo = base + (size_t)c * stride;
        const size_t idx_hi = idx_lo + stride;
        if (bitmap_prefix_get(bitmap, idx_hi)) {
            atomic_or((volatile __global uint*)&bitmap[idx_lo >> 5],
                      1u << (uint)(idx_lo & (size_t)31));
        }
    }
}
